#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "pico/stdio/driver.h"
#include "crsf.h"

// --- Hardware Definitions ---
#define LED_PIN 26
#define DBG_PIN 27
#define CRSF_UART uart0  // UART0 for CRSF
#define PC_UART   uart1  // UART1 for PC Link
#define BAUD_RATE 420000
#define PC_BAUD   420000 

// CRSF Pins (UART0)
#define CRSF_TX_PIN 0
#define CRSF_RX_PIN 1

// PC Link Pins (UART1)
#define PC_TX_PIN 4
#define PC_RX_PIN 5

// --- Timing & Phase Lock ---
volatile int64_t current_phase_correction = 0;
volatile int64_t normal_interval_us = 5000; // Default 200Hz
bool timer_active = false;

// --- Double Buffering for TX ---
uint8_t tx_buffer_A[CRSF_MAX_FRAME_SIZE];
uint8_t tx_buffer_B[CRSF_MAX_FRAME_SIZE];
volatile uint8_t* active_tx_ptr = tx_buffer_A; 
uint8_t* writing_tx_ptr = tx_buffer_B;         

volatile uint32_t last_usb_packet_ms = 0;      

// --- DMA Channels ---
volatile int tx_ctrl_chan;
volatile int rx_ctrl_chan;
uint8_t rx_ring_buf[256] __attribute__((aligned(256)));
uint8_t rx_read_ptr = 0; 

// --- Parser States ---
uint8_t uart_parser_state = 0, uart_packet_len = 0, uart_packet_idx = 0;
uint8_t uart_rx_buffer[CRSF_MAX_FRAME_SIZE];
uint8_t usb_parser_state = 0, usb_packet_len = 0, usb_packet_idx = 0;
uint8_t usb_rx_buffer[CRSF_MAX_FRAME_SIZE];

// ============================================================================
// HARDWARE & TRANSMIT
// ============================================================================

void init_hardware() 
{
    uart_init(CRSF_UART, BAUD_RATE);
    gpio_set_function(CRSF_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(CRSF_RX_PIN, GPIO_FUNC_UART);

    uart_init(PC_UART, PC_BAUD);
    gpio_set_function(PC_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PC_RX_PIN, GPIO_FUNC_UART);

    tx_ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_ctrl_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, DREQ_UART0_TX);
    dma_channel_configure(tx_ctrl_chan, &tx_cfg, &uart_get_hw(CRSF_UART)->dr, active_tx_ptr, 26, false);

    rx_ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_ctrl_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    channel_config_set_dreq(&rx_cfg, DREQ_UART0_RX);
    channel_config_set_ring(&rx_cfg, true, 8); 
    dma_channel_configure(rx_ctrl_chan, &rx_cfg, rx_ring_buf, &uart_get_hw(CRSF_UART)->dr, 0xFFFFFFFF, true);
}

void trigger_crsf_tx() 
{
    if (!dma_channel_is_busy(tx_ctrl_chan)) 
    {
        dma_channel_transfer_from_buffer_now(tx_ctrl_chan, (void*)active_tx_ptr, 26);
    }
}

void apply_failsafe() 
{
    //printf("Failsafe!\r\n");
    uint16_t fs_channels[16];
    for (int i = 0; i < 16; i++) fs_channels[i] = CRSF_CHANNEL_MID;
    //fs_channels[0] = CRSF_CHANNEL_MIN; //- ROLL
    //fs_channels[1] = CRSF_CHANNEL_MIN; //- PITCH
    fs_channels[2] = CRSF_CHANNEL_MIN; //- Throttle
    //fs_channels[3]  = CRSF_CHANNEL_MIN; //- YAW
    
    // ARMED when both are MAX!!!
    //fs_channels[4]  = CRSF_CHANNEL_MAX;
    //fs_channels[13] = CRSF_CHANNEL_MAX;

    crsf_pack_rc_channels(writing_tx_ptr, fs_channels);
    writing_tx_ptr[25] = crsf_crc8((uint8_t*)&writing_tx_ptr[2], 23);
    
    // Swap safely to active buffer
    uint8_t* temp = (uint8_t*)active_tx_ptr;
    active_tx_ptr = writing_tx_ptr;
    writing_tx_ptr = temp;
}

int64_t tx_alarm_callback(alarm_id_t id, void *user_data) 
{
    trigger_crsf_tx(); 
    
    int64_t next_delay = normal_interval_us;
    if (current_phase_correction != 0) 
    {
        next_delay -= current_phase_correction; 
        current_phase_correction = 0; 
    }
    return -next_delay; 
}

// ============================================================================
// PARSERS
// ============================================================================

void parse_user_in(uint8_t b) 
{
    if (usb_parser_state == 0) 
    {
        if (b == 0xC8 || b == 0xEE) { usb_rx_buffer[0] = b; usb_parser_state = 1; }
    } 
    else if (usb_parser_state == 1) 
    {
        if (b > (CRSF_MAX_FRAME_SIZE - 2) || b < 2) { usb_parser_state = 0; } 
        else { usb_rx_buffer[1] = b; usb_packet_len = b; usb_packet_idx = 2; usb_parser_state = 2; }
    } 
    else if (usb_parser_state == 2) 
    {
        usb_rx_buffer[usb_packet_idx++] = b;
        if (usb_packet_idx >= usb_packet_len + 2) 
        { 
            if (crsf_crc8(&usb_rx_buffer[2], usb_packet_len - 1) == usb_rx_buffer[usb_packet_len + 1]) {
                if (usb_rx_buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) 
                {
                    memcpy(writing_tx_ptr, usb_rx_buffer, usb_packet_len + 2);
                    uint8_t* temp = (uint8_t*)active_tx_ptr;
                    active_tx_ptr = writing_tx_ptr;
                    writing_tx_ptr = temp;
                    last_usb_packet_ms = to_ms_since_boot(get_absolute_time());
                }
            }
            usb_parser_state = 0;
        }
    }
}

void parse_crsf_in(uint8_t b) 
{
    if (uart_parser_state == 0) 
    {
        if (b == CRSF_ADDRESS_FLIGHT_CONTROLLER || b == CRSF_ADDRESS_CRSF_RADIO_TRANSMITTER || b == CRSF_ADDRESS_CRSF_TRANSMITTER) 
        { 
            uart_rx_buffer[0] = b; uart_parser_state = 1; 
        }
    } 
    else if (uart_parser_state == 1) 
    {
        if (b > (CRSF_MAX_FRAME_SIZE - 2) || b < 2) { uart_parser_state = 0; } 
        else { uart_rx_buffer[1] = b; uart_packet_len = b; uart_packet_idx = 2; uart_parser_state = 2; }
    } else if (uart_parser_state == 2) {
        uart_rx_buffer[uart_packet_idx++] = b;
        if (uart_packet_idx >= uart_packet_len + 2) { 
            if (crsf_crc8(&uart_rx_buffer[2], uart_packet_len - 1) == uart_rx_buffer[uart_packet_len + 1]) {
                if (uart_rx_buffer[2] == CRSF_FRAMETYPE_RADIO_ID && uart_rx_buffer[5] == CRSF_FRAMETYPE_OPENTX_SYNC) {
                    uint32_t interval_raw = (uart_rx_buffer[6] << 24) | (uart_rx_buffer[7] << 16) | (uart_rx_buffer[8] << 8) | uart_rx_buffer[9];
                    uint32_t phase_raw = (uart_rx_buffer[10] << 24) | (uart_rx_buffer[11] << 16) | (uart_rx_buffer[12] << 8) | uart_rx_buffer[13];
                    
                    normal_interval_us = (int32_t)interval_raw / 10;
                    current_phase_correction = (int32_t)phase_raw / 10;
                    // Clamp phase
                    if (current_phase_correction > 1000) current_phase_correction = 1000;
                    if (current_phase_correction < -1000) current_phase_correction = -1000;
                } 
                else 
                {
                    static int cnt = 0;
     
for (int i = 0; i < uart_packet_len + 2; i++) {
    uart_putc(PC_UART, uart_rx_buffer[i]);
}                }
            } 
            uart_parser_state = 0; 
        }
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

int main() 
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(DBG_PIN);
    gpio_set_dir(DBG_PIN, GPIO_OUT);
    init_hardware();
    sleep_ms(500); // Quick settling

    // 1. Prepare safe disarmed packet
    apply_failsafe();
    last_usb_packet_ms = to_ms_since_boot(get_absolute_time());

    add_alarm_in_us(5000, tx_alarm_callback, NULL, true);
    timer_active = true;

    while(1) 
    {

        while (uart_is_readable(PC_UART)) 
        {
            uint8_t c = uart_getc(PC_UART);
            gpio_put(DBG_PIN, !gpio_get(DBG_PIN));

            parse_user_in(c);
        }
        // Handle UART Telemetry
        uint8_t dma_write_ptr = (uint8_t)(dma_hw->ch[rx_ctrl_chan].write_addr - (uint32_t)rx_ring_buf); 
        while (rx_read_ptr != dma_write_ptr) 
        {

            parse_crsf_in(rx_ring_buf[rx_read_ptr++]); 

        }
        // failsafe check with 2000 ms timeout
        if (to_ms_since_boot(get_absolute_time()) - last_usb_packet_ms > 2000) 
        {
            apply_failsafe();

            last_usb_packet_ms = to_ms_since_boot(get_absolute_time()); 
        } 
    }

    sleep_us(10);

} 
