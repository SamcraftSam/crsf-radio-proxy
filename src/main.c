#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "crsf.h"

#define LED_PIN 26
#define TX_TIM_PIN 27
#define UART_ID uart0
#define BAUD_RATE 420000
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// --- Global Buffers and DMA Channels ---
volatile uint8_t tx_buffer[26];
volatile int tx_ctrl_chan;
volatile int rx_ctrl_chan;

// --- Global RC Channels for Animation ---
volatile uint16_t rc_channels[16];
volatile uint16_t sweep_val = 172; // CRSF minimum value (~988us)
volatile int sweep_dir = 10;       // How fast the channel moves

uint8_t rx_ring_buf[256] __attribute__((aligned(256)));
uint8_t rx_read_ptr = 0; 

struct repeating_timer crsf_tx_timer;
bool timer_active = false;


// --- Hardware Initialization ---

void init_crsf_hardware() {

    // 1. Setup UART

    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);


    // 2. Setup TX DMA (Fires on command)

    tx_ctrl_chan = dma_claim_unused_channel(true);

    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_ctrl_chan);

    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);

    channel_config_set_read_increment(&tx_cfg, true);

    channel_config_set_write_increment(&tx_cfg, false);

    channel_config_set_dreq(&tx_cfg, DREQ_UART0_TX);


    dma_channel_configure(

        tx_ctrl_chan, &tx_cfg,

        &uart_get_hw(UART_ID)->dr, // Write to UART

        tx_buffer,                 // Read from tx_buffer

        26,                        // Length of CRSF RC packet

        false                      // Do NOT start yet

    );


    // 3. Setup RX DMA (Runs continuously into a Ring Buffer)

    rx_ctrl_chan = dma_claim_unused_channel(true);

    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_ctrl_chan);

    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);

    channel_config_set_read_increment(&rx_cfg, false);

    channel_config_set_write_increment(&rx_cfg, true);

    channel_config_set_dreq(&rx_cfg, DREQ_UART0_RX);

    // Wrap the write address every 2^8 (256) bytes

    channel_config_set_ring(&rx_cfg, true, 8); 


    dma_channel_configure(

        rx_ctrl_chan, &rx_cfg,

        rx_ring_buf,               // Write to our ring buffer

        &uart_get_hw(UART_ID)->dr, // Read from UART

        0xFFFFFFFF,                // Run forever

        true                       // Start immediately

    );

} 

// --- Helper to trigger DMA ---
void trigger_crsf_tx() {
    if (!dma_channel_is_busy(tx_ctrl_chan)) {
        dma_channel_transfer_from_buffer_now(tx_ctrl_chan, tx_buffer, 26);
        gpio_put(LED_PIN, !gpio_get(LED_PIN)); 
    }
}

// --- The Timer Callback (Fires every 5000us) ---
bool tx_timer_callback(struct repeating_timer *t) 
{
    gpio_put(TX_TIM_PIN, !gpio_get(TX_TIM_PIN));

    // 1. Animate Channel 0 (Roll)
    sweep_val += sweep_dir;
    if (sweep_val > 1811) { // CRSF max
        sweep_val = 1811;
        sweep_dir = -10;    // Reverse direction
    } else if (sweep_val < 172) { // CRSF min
        sweep_val = 172;
        sweep_dir = 10;     // Reverse direction
    }
    rc_channels[0] = sweep_val; // Apply to Roll

    // 2. Repack the buffer with the new channel data
    // (Ensure your crsf_pack_channels function is fast enough for an IRQ)
    crsf_pack_channels((uint8_t*)tx_buffer, (uint16_t*)rc_channels);

    // 3. Send it
    trigger_crsf_tx();
    
    return true; // Keep repeating forever
}

// ... [Keep your init_crsf_hardware() exactly the same] ...

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(TX_TIM_PIN);
    gpio_set_dir(TX_TIM_PIN, GPIO_OUT);
    
    init_crsf_hardware();
    
    sleep_ms(3000);
    // Initialize all channels to MID (center stick)
    for (int i = 0; i < 16; i++) {
        rc_channels[i] = CRSF_CHANNEL_MID;
    }
    
    // Initial pack
    crsf_pack_channels((uint8_t*)tx_buffer, (uint16_t*)rc_channels);
    
    printf("CRSF Hardware Initialized. Listening for Heartbeat...\r\n");
    
    uint8_t parser_state = 0;
    uint8_t packet_len = 0;
    uint8_t packet_idx = 0;
    uint8_t rx_packet_buffer[CRSF_MAX_FRAME_SIZE];
    
    trigger_crsf_tx();

    while(1) 
    {
        uint8_t dma_write_ptr = (uint8_t)(dma_hw->ch[rx_ctrl_chan].write_addr - (uint32_t)rx_ring_buf); 
        
        while (rx_read_ptr != dma_write_ptr) 
        {
            uint8_t b = rx_ring_buf[rx_read_ptr++]; 

            if (parser_state == 0) {
                if (b == CRSF_BYTE_SYNC) { 
                    rx_packet_buffer[0] = b;
                    parser_state = 1;
                }
            } 
            else if (parser_state == 1) {
                if (b > 64 || b < 2) { 
                    parser_state = 0;
                } else {
                    rx_packet_buffer[1] = b;
                    packet_len = b;
                    packet_idx = 2;
                    parser_state = 2;
                }
            } 
            else if (parser_state == 2) {
                rx_packet_buffer[packet_idx++] = b;
                
                if (packet_idx >= packet_len + 2) 
                { 
                    uint8_t calculated_crc = crsf_crc8(&rx_packet_buffer[2], packet_len - 1);
                    uint8_t received_crc = rx_packet_buffer[packet_len + 1];
                    
                    if (calculated_crc == received_crc) 
                    {
                        //sweep_val += sweep_dir;
                        //if (sweep_val > 1811 || sweep_val < 172) sweep_dir *= -1;
                        //rc_channels[0] = sweep_val;
    
                        //crsf_pack_channels((uint8_t*)tx_buffer, (uint16_t*)rc_channels);
                        //trigger_crsf_tx();     
                        // --- RADIO ID PACKET (Sync Handshake) --
                        if (rx_packet_buffer[2] == CRSF_FRAMETYPE_RADIO_ID) 
                        {
                        if (rx_packet_buffer[3] == 0xEA)
                        {
                            uint8_t ext_rem_subtype = rx_packet_buffer[4]; 
                            if (rx_packet_buffer[5] == CRSF_FRAMETYPE_OPENTX_SYNC) 
                            {
                                uint32_t interval_raw = (rx_packet_buffer[6] << 24) |
                                    (rx_packet_buffer[7] << 16) |
                                    (rx_packet_buffer[8] << 8)  |
                                    rx_packet_buffer[9];

                                uint32_t phase_raw = (rx_packet_buffer[10] << 24) |
                                    (rx_packet_buffer[11] << 16) |
                                    (rx_packet_buffer[12] << 8) |
                                    rx_packet_buffer[13];

                            printf("OPENTX_SYNC:\r\n");
                            printf("  Packet interval raw: 0x%08X\n", interval_raw);
                            printf("  Interval us: %lu\n", interval_raw / 10);
                            printf("  Phase correction raw: 0x%08X\n", phase_raw);
                       

                            int32_t phase_us = (int32_t)phase_raw;
                            int32_t interval_us = (int32_t)interval_raw / 10;
                            // Clamp it so it doesn't go insane
                            if (phase_us > 1000) phase_us = 1000;
                            if (phase_us < -1000) phase_us = -1000;

                            int64_t adjusted_interval = interval_us - phase_us;
                            
                            cancel_repeating_timer(&crsf_tx_timer);
                            add_repeating_timer_us(-adjusted_interval, tx_timer_callback, NULL, &crsf_tx_timer);
                        }
                        else
                        {
                            printf("RADIO_ID frame, unknown subtype: 0x%02X\n", ext_rem_subtype);
                        }
                        }
                        // ONLY START THE TIMER ONCE! 
                        // This prevents phase drift and the "NO HANDSET" bug.
                        if (!timer_active) 
                        {
                            // Hardcoded to -5000us (200Hz)
                            add_repeating_timer_us(-5000, tx_timer_callback, NULL, &crsf_tx_timer);
                            timer_active = true;
                            printf("CRSF Sync Locked! Hardcoded to 5000us phase.\r\n");
                                
                            // Fire the first packet immediately to align phase with the module
                            trigger_crsf_tx();
                        }
                        }
                    } 
                    parser_state = 0; 
                }
            }
        }
    }
}
