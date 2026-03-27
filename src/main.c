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


//timer timing
volatile uint32_t crsf_interval_us = 5000;

// 256-byte aligned buffer for DMA Ring Buffer
uint8_t rx_ring_buf[256] __attribute__((aligned(256)));
uint8_t rx_read_ptr = 0; // Where the CPU is currently reading

// --- Global Timer for CRSFShot ---
struct repeating_timer crsf_tx_timer;
bool timer_active = false;


// --- Quick helper to send the packet via DMA ---
void trigger_crsf_tx() {
    // Only trigger if previous transfer is done
    if (!dma_channel_is_busy(tx_ctrl_chan)) {
        //dma_channel_set_read_addr(tx_ctrl_chan, tx_buffer, true);
        dma_channel_transfer_from_buffer_now(tx_ctrl_chan, tx_buffer, 26);
        gpio_put(LED_PIN, !gpio_get(LED_PIN)); // Toggle LED on TX
    }
}

// This fires exactly at the requested frequency (e.g., every 5000us)
bool tx_timer_callback(struct repeating_timer *t) 
{
    gpio_put(TX_TIM_PIN, !gpio_get(TX_TIM_PIN));
    trigger_crsf_tx();
    return true; // Keep repeating
}

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

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(TX_TIM_PIN);
    gpio_set_dir(TX_TIM_PIN, GPIO_OUT);
    // Initialize UART and DMAs
    init_crsf_hardware();

    // Setup Dummy Channels and Pack the Buffer ONCE using your lib
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) {
        channels[i] = CRSF_CHANNEL_MID;
    }
    
    // Using your library function to pack data and generate CRC
    crsf_pack_channels(tx_buffer, channels);
    
    gpio_put(TX_TIM_PIN, 1);
    sleep_ms(3000); //debug
    gpio_put(TX_TIM_PIN, 0);
    printf("CRSF Hardware Initialized. Listening for Heartbeat...\r\n");

    trigger_crsf_tx();
    // --- Variables for the RX Parser State Machine ---
    uint8_t parser_state = 0;
    uint8_t packet_len = 0;
    uint8_t packet_idx = 0;
    uint8_t rx_packet_buffer[CRSF_MAX_FRAME_SIZE];

    while(1) 
    {
        // Find out where the DMA is currently writing
        // Cast to uint8_t to handle wrapping perfectly (0-255)
        uint8_t dma_write_ptr = (uint8_t)(dma_hw->ch[rx_ctrl_chan].write_addr - (uint32_t)rx_ring_buf); 
        // Process all new bytes collected by the DMA
        while (rx_read_ptr != dma_write_ptr) 
        {
            //printf("RX read ptr: %u Value: 0x%x\r\n", rx_read_ptr, rx_ring_buf[rx_read_ptr]);
            uint8_t b = rx_ring_buf[rx_read_ptr++]; // ++ automatically wraps at 255 due to uint8_t

            // --- Simple CRSF Parser State Machine ---
            if (parser_state == 0) 
            {
                // Looking for Sync Byte from ELRS Module (Usually 0xEE)
                if (b == CRSF_BYTE_SYNC) 
                { 
                    rx_packet_buffer[0] = b;
                    parser_state = 1;
                }
                else
                {
                    printf("[MISMATCH] Detected packet address is: 0x%x\r\n", b);
                }
            } 
            else if (parser_state == 1) 
            {
                // Reading Length
                if (b > 64 || b < 2) 
                { // Invalid CRSF length, reset
                    printf("Invalid CRSF lenght, reset\r\n");
                    parser_state = 0;
                } 
                else 
                {
                    rx_packet_buffer[1] = b;
                    packet_len = b;
                    packet_idx = 2;
                    parser_state = 2;
                }
            } 
            else if (parser_state == 2) 
            {
                // Reading Payload & CRC
                rx_packet_buffer[packet_idx++] = b;
                
                // Have we received the whole packet? (+2 accounts for Sync and Len bytes)
                if (packet_idx >= packet_len + 2) 
                { 
                    
                    // Verify CRC
                    uint8_t calculated_crc = crsf_crc8(&rx_packet_buffer[2], packet_len - 1);
                    uint8_t received_crc = rx_packet_buffer[packet_len + 1];
                    
                    if (calculated_crc == received_crc) 
                    {
                        printf("Congratulation!\r\n");
                        // WE GOT A VALID HEARTBEAT! 
                        // Instantly fire the RC channels packet back to the module.
                        if (rx_packet_buffer[2] == CRSF_FRAMETYPE_RADIO_ID)
                        {
                            // 1. Extract the 32-bit interval (Big Endian) from bytes 6, 7, 8, 9
                            uint32_t interval_raw = (rx_packet_buffer[6] << 24) |
                                                    (rx_packet_buffer[7] << 16) |
                                                    (rx_packet_buffer[8] << 8) |
                                                    (rx_packet_buffer[9]);

                            // 2. Convert to actual microseconds
                            uint64_t interval_us = (int64_t)interval_raw / 10;
                            uint64_t interval_ms = interval_raw / 1000;
                            // 3. Setup or update the hardware timer
                            if (timer_active) 
                            {
                                cancel_repeating_timer(&crsf_tx_timer);
                            }
                            else
                            {
                                 trigger_crsf_tx();

                            }
                            // Negative delay means the timer fires exactly every X us, 
                            // regardless of how long the callback takes to execute
                            add_repeating_timer_us(-interval_us, tx_timer_callback, NULL, &crsf_tx_timer);
                            timer_active = true;
                                                       
                            printf("CRSFShot Synced! Locked to %lu us interval.\r\n", interval_us/1000);
                            printf("Additional info: int_raw: %u int_us: %lu int_ms: %lu\r\n", interval_raw, interval_us, interval_ms);
                        
                            // Fire one packet immediately to keep it happy
                            ///trigger_crsf_tx();
                        }
                        //trigger_crsf_tx();
                        
                        // TODO: If rx_packet_buffer[2] == CRSF_FRAMETYPE_LINK_STATISTICS
                        // Call your crsf_parse_telemetry() function here!
                    }
                    else
                    {
                        printf("Bad CRC\r\n");
                    }
                    
                    parser_state = 0; // Reset for next packet
                }
            }
        }
        
        // This loop runs extremely fast. It parses bytes exactly as DMA drops them in.
        // You have plenty of time here to handle USB stuff.
    }
}
