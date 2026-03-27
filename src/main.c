#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "crsf.h"

// --- Hardware Definitions ---
#define LED_PIN 26
#define TX_TIM_PIN 27
#define UART_ID uart0
#define BAUD_RATE 420000
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// --- Global Phase Lock & Timer Variables ---
volatile int64_t current_phase_correction = 0;
volatile int64_t normal_interval_us = 5000; 
bool timer_active = false;

// --- Global Buffers and DMA Channels ---
volatile uint8_t tx_buffer[26];
volatile int tx_ctrl_chan;
volatile int rx_ctrl_chan;

uint8_t rx_ring_buf[256] __attribute__((aligned(256)));
uint8_t rx_read_ptr = 0; 

// --- Global RC Channels & Animation ---
volatile uint16_t rc_channels[16];
volatile uint16_t sweep_val = 172; // CRSF minimum value (~988us)
volatile int sweep_dir = 10;       // How fast the channel moves

// --- Parser State Variables ---
uint8_t parser_state = 0;
uint8_t packet_len = 0;
uint8_t packet_idx = 0;
uint8_t rx_packet_buffer[CRSF_MAX_FRAME_SIZE];

// ============================================================================
// HARDWARE INIT & DMA CONTROL
// ============================================================================

void init_crsf_hardware() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // TX DMA Setup
    tx_ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_ctrl_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, DREQ_UART0_TX);

    dma_channel_configure(tx_ctrl_chan, &tx_cfg,
        &uart_get_hw(UART_ID)->dr, tx_buffer, 26, false);

    // RX DMA Setup (Ring Buffer)
    rx_ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_ctrl_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    channel_config_set_dreq(&rx_cfg, DREQ_UART0_RX);
    channel_config_set_ring(&rx_cfg, true, 8); 

    dma_channel_configure(rx_ctrl_chan, &rx_cfg,
        rx_ring_buf, &uart_get_hw(UART_ID)->dr, 0xFFFFFFFF, true);
} 

void trigger_crsf_tx() {
    if (!dma_channel_is_busy(tx_ctrl_chan)) {
        dma_channel_transfer_from_buffer_now(tx_ctrl_chan, tx_buffer, 26);
        gpio_put(LED_PIN, !gpio_get(LED_PIN)); 
    }
}

// ============================================================================
// TIMING & PHASE LOCK LOGIC
// ============================================================================

int64_t tx_alarm_callback(alarm_id_t id, void *user_data) 
{
    gpio_put(TX_TIM_PIN, !gpio_get(TX_TIM_PIN));

    // 1. Animate Channel 0
    sweep_val += sweep_dir;
    if (sweep_val > 1811) { sweep_val = 1811; sweep_dir = -10; } 
    else if (sweep_val < 172) { sweep_val = 172; sweep_dir = 10; }
    rc_channels[0] = sweep_val; 

    // 2. Repack and send
    crsf_pack_channels((uint8_t*)tx_buffer, (uint16_t*)rc_channels);
    trigger_crsf_tx();
    
    // 3. Calculate next tick with phase correction
    int64_t next_delay = normal_interval_us;
    if (current_phase_correction != 0) {
        next_delay -= current_phase_correction; 
        current_phase_correction = 0; // Apply ONCE
    }
    
    return -next_delay; // Negative ensures relative timing
}

// ============================================================================
// PACKET HANDLERS
// ============================================================================

void handle_opentx_sync_packet() {
    // Extract payload from OpenTX Sync frame
    uint32_t interval_raw = (rx_packet_buffer[6] << 24) | (rx_packet_buffer[7] << 16) | (rx_packet_buffer[8] << 8) | rx_packet_buffer[9];
    uint32_t phase_raw = (rx_packet_buffer[10] << 24) | (rx_packet_buffer[11] << 16) | (rx_packet_buffer[12] << 8) | rx_packet_buffer[13];

    int32_t phase_us = (int32_t)phase_raw / 10;
    
    // Clamp phase correction to prevent wild swings
    if (phase_us > 1000) phase_us = 1000;
    if (phase_us < -1000) phase_us = -1000;

    // Update phase lock globals
    normal_interval_us = (int32_t)interval_raw / 10;
    current_phase_correction = phase_us;

    // Start the alarm loop ONCE if it hasn't been started
    if (!timer_active) {
        add_alarm_in_us(5000, tx_alarm_callback, NULL, true);
        timer_active = true;
        trigger_crsf_tx();
        printf("Sync Locked! Alarm started.\r\n");
    }
}

// ============================================================================
// CRSF PARSER STATE MACHINE
// ============================================================================

void crsf_parse_byte(uint8_t b) {
    if (parser_state == 0) {
        // ACCEPT ALL VALID CRSF SYNC ADDRESSES
        if (b == CRSF_ADDRESS_FLIGHT_CONTROLLER || 
            b == CRSF_ADDRESS_CRSF_RADIO_TRANSMITTER || 
            b == CRSF_ADDRESS_CRSF_TRANSMITTER) { 
            rx_packet_buffer[0] = b;
            parser_state = 1;
        }
    } 
    else if (parser_state == 1) {
        if (b > (CRSF_MAX_FRAME_SIZE - 2) || b < 2) { 
            parser_state = 0; // Prevent overflow/garbage lengths
        } else {
            rx_packet_buffer[1] = b;
            packet_len = b;
            packet_idx = 2;
            parser_state = 2;
        }
    } 
    else if (parser_state == 2) {
        rx_packet_buffer[packet_idx++] = b;
        
        // If we have received the full packet + CRC byte
        if (packet_idx >= packet_len + 2) { 
            uint8_t calculated_crc = crsf_crc8(&rx_packet_buffer[2], packet_len - 1);
            uint8_t received_crc = rx_packet_buffer[packet_len + 1];
            
            if (calculated_crc == received_crc) {
                // Route valid packets based on Frame Type
                if (rx_packet_buffer[2] == CRSF_FRAMETYPE_RADIO_ID && 
                    rx_packet_buffer[3] == CRSF_ADDRESS_CRSF_RADIO_TRANSMITTER && 
                    rx_packet_buffer[5] == CRSF_FRAMETYPE_OPENTX_SYNC) 
                {
                    handle_opentx_sync_packet();
                }
                // TODO: Add Link Statistics or other telemetry handlers here later
            } 
            parser_state = 0; // Reset for next packet
        }
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

int main() {
    stdio_init_all();
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(TX_TIM_PIN);
    gpio_set_dir(TX_TIM_PIN, GPIO_OUT);
    
    init_crsf_hardware();
    
    sleep_ms(3000); // Wait for module to boot
    
    // Initialize all channels to MID
    for (int i = 0; i < 16; i++) {
        rc_channels[i] = CRSF_CHANNEL_MID;
    }
    crsf_pack_channels((uint8_t*)tx_buffer, (uint16_t*)rc_channels);
    
    printf("CRSF Hardware Initialized. Listening for Heartbeat...\r\n");
    trigger_crsf_tx(); // Kickoff

    while(1) {
        // Find how far the DMA has written into our ring buffer
        uint8_t dma_write_ptr = (uint8_t)(dma_hw->ch[rx_ctrl_chan].write_addr - (uint32_t)rx_ring_buf); 
        
        // Drain the buffer and parse every byte
        while (rx_read_ptr != dma_write_ptr) {
            crsf_parse_byte(rx_ring_buf[rx_read_ptr++]); 
        }
    }
}
