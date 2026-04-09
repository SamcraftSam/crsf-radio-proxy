/* Copyright (c) 2026 Alexander @SamcraftSam, MIT License */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include <bsp/board_api.h>
#include "tusb.h"
#include "tusb_config.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "pico/stdio/driver.h"
#include "crsf.h"
#include "channelmap.h"

#include "pico/multicore.h"
#include "pico/util/queue.h"

// --- Hardware Definitions ---
#define LED_PIN 26
#define DBG_PIN 27
#define CRSF_UART uart1  // UART0 for CRSF
#define BAUD_RATE 420000

// CRSF Pins (UART0)
#define CRSF_TX_PIN 4
#define CRSF_RX_PIN 5

// --- Timing & Phase Lock ---
volatile int64_t phase_correction = 0;
volatile int64_t interval_us = 5000; // Default 200Hz
bool timer_active = false;

// --- Double Buffering for TX ---
uint8_t tx_buffer_A[CRSF_MAX_FRAME_SIZE];
uint8_t tx_buffer_B[CRSF_MAX_FRAME_SIZE];
volatile uint8_t* active_tx_ptr = tx_buffer_A; 
volatile uint8_t* writing_tx_ptr = tx_buffer_B;         

volatile uint32_t last_usb_packet_ms = 0;      

volatile bool tx_flag = false;

// --- DMA Channels ---
volatile int tx_ctrl_chan;
volatile int rx_ctrl_chan;
uint8_t rx_ring_buf[256] __attribute__((aligned(256)));
uint8_t rx_read_ptr = 0; 

crsf_parser_t pc_parser = {0};
crsf_parser_t rx_parser = {0};
// ============================================================================
// HARDWARE & TRANSMIT
// ============================================================================

void init_hardware() 
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(DBG_PIN);
    gpio_set_dir(DBG_PIN, GPIO_OUT);
    uart_init(CRSF_UART, BAUD_RATE);
    gpio_set_function(CRSF_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(CRSF_RX_PIN, GPIO_FUNC_UART);

    tx_ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config tx_cfg = dma_channel_get_default_config(tx_ctrl_chan);
    channel_config_set_transfer_data_size(&tx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_cfg, true);
    channel_config_set_write_increment(&tx_cfg, false);
    channel_config_set_dreq(&tx_cfg, DREQ_UART1_TX);
    dma_channel_configure(tx_ctrl_chan, &tx_cfg, &uart_get_hw(CRSF_UART)->dr, active_tx_ptr, 26, false);

    rx_ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config rx_cfg = dma_channel_get_default_config(rx_ctrl_chan);
    channel_config_set_transfer_data_size(&rx_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_cfg, false);
    channel_config_set_write_increment(&rx_cfg, true);
    channel_config_set_dreq(&rx_cfg, DREQ_UART1_RX);
    channel_config_set_ring(&rx_cfg, true, 8); 
    dma_channel_configure(rx_ctrl_chan, &rx_cfg, rx_ring_buf, &uart_get_hw(CRSF_UART)->dr, 0xFFFFFFFF, true);
}

void trigger_crsf_tx() 
{
    if (!dma_channel_is_busy(tx_ctrl_chan)) 
    {
        dma_channel_set_read_addr(tx_ctrl_chan, (void*)active_tx_ptr, true); // ?
        dma_channel_transfer_from_buffer_now(tx_ctrl_chan, (void*)active_tx_ptr, 26);
    }
}

// Writes p2 to p1 and p1 to p2
void safe_ptr_swap(volatile uint8_t ** p1, volatile uint8_t ** p2)
{
    uint32_t ints = save_and_disable_interrupts();
    uint8_t * tmp = (uint8_t *)*p1;
    *p1 = *p2;
    *p2 = tmp;
    restore_interrupts(ints);
}

void prepare_failsafe_packet() 
{
    uint16_t fs_channels[RC_CHANNELS_NUM];
    for (int i = 0; i < 16; i++) fs_channels[i] = CRSF_CHANNEL_MID;
    fs_channels[RC_CHAN_THROTTLE]  = CRSF_CHANNEL_MIN; //- Throttle
    fs_channels[RC_CHAN_DISARM_AUX1]  = CRSF_CHANNEL_MIN;
    fs_channels[RC_CHAN_DISARM_AUX10] = CRSF_CHANNEL_MIN;

    crsf_pack_rc_channels((uint8_t *)writing_tx_ptr, fs_channels);
    writing_tx_ptr[25] = crsf_crc8((uint8_t*)&writing_tx_ptr[2], 23);
    
    // Swap safely to active buffer
    safe_ptr_swap(&active_tx_ptr, &writing_tx_ptr);
}

int64_t tx_alarm_callback(alarm_id_t id, void *user_data) 
{
    tx_flag = true;
    gpio_put(DBG_PIN, !gpio_get(DBG_PIN));

    int64_t next_delay = interval_us;
    if (phase_correction != 0) 
    {
#define DBG_CH(s) { uart_puts(PC_UART, s); }


        next_delay -= phase_correction; 
        phase_correction = 0; 
    }
    return -next_delay; 
}

// ============================================================================
// MAIN LOOP
// ============================================================================

int main() 
{
    board_init();
    tusb_init();
    
    if (board_init_after_tusb) 
    {
        board_init_after_tusb();
    }

    //stdio_init_all();
    stdio_usb_init();

    init_hardware(); 
     
    prepare_failsafe_packet();
    last_usb_packet_ms = to_ms_since_boot(get_absolute_time());
    add_alarm_in_us(interval_us, tx_alarm_callback, NULL, true);
    timer_active = true;

    while(1) 
    {
        tud_task();

        static uint8_t ready_to_flush = 0;
        int8_t ret = 0;

        // Handle PC USB in (send to radio)
        if (tud_cdc_n_available(1))
        {
            uint8_t buf[64];
            uint32_t count = tud_cdc_n_read(1, buf, sizeof(buf));

            for (uint32_t i = 0; i < count; i++)
            {
                ret = crsf_collect_byte(buf[i], &pc_parser);

                if(ret && pc_parser.rxbuf[TYPE] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
                {
                    memcpy((uint8_t *)writing_tx_ptr, pc_parser.rxbuf, pc_parser.len + 2);
                    // SAFE SWAP
                    safe_ptr_swap(&active_tx_ptr, &writing_tx_ptr);
                    last_usb_packet_ms = to_ms_since_boot(get_absolute_time());
                }
                
            }
        }

        // Handle UART Telemetry (get sync / send telemetry to PC)
        uint8_t dma_write_ptr = (uint8_t)(dma_hw->ch[rx_ctrl_chan].write_addr - (uint32_t)rx_ring_buf); 

        while (rx_read_ptr != dma_write_ptr) 
        {
            ret = crsf_collect_byte(rx_ring_buf[rx_read_ptr++], &rx_parser);

            if (ret)
            {
                ret = crsf_parse_sync(rx_parser.rxbuf, &rx_parser.len, (int64_t *)&interval_us, (int64_t *)&phase_correction);

                if (tud_cdc_write_available() >= (rx_parser.len + 2))
                {
                    tud_cdc_n_write(1, rx_parser.rxbuf, rx_parser.len + 2);
                    ready_to_flush = 1;
                }
            }
        }
        
        // failsafe check with 2000 ms timeout
        if (to_ms_since_boot(get_absolute_time()) - last_usb_packet_ms > 500) 
        {
            prepare_failsafe_packet();
            last_usb_packet_ms = to_ms_since_boot(get_absolute_time()); 
        } 
        
        if(ready_to_flush)
        {
            tud_cdc_write_flush();
            ready_to_flush = 0;
        }

        
        if (tx_flag && !dma_channel_is_busy(tx_ctrl_chan)) 
        {
            tx_flag = false;
            trigger_crsf_tx();
        }
    } 
}

// [TODO] USE THIS CALLBACK FOR BETTER USB INPUT PROCESSING
// Might be useful with custom commands strategy

void tud_cdc_rx_cb(uint8_t itf)
{
    return;
}

void __tud_cdc_rx_cb(uint8_t itf)
{
    // allocate buffer for the data in the stack
    uint8_t buf[CFG_TUD_CDC_RX_BUFSIZE];

    //printf("RX CDC %d\n", itf);

    // read the available data 
    // | IMPORTANT: also do this for CDC0 because otherwise
    // | you won't be able to print anymore to CDC0
    // | next time this function is called
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    // check if the data was received on the second cdc interface
    if (itf == 1 && to_ms_since_boot(get_absolute_time()) - last_usb_packet_ms > 100) {
        last_usb_packet_ms = to_ms_since_boot(get_absolute_time());
        // process the received data
        buf[count] = 0; // null-terminate the string
        // now echo data back to the console on CDC 0
        printf("Received on CDC 1: %s\n", buf);

        // and echo back OK on CDC 1
        //tud_cdc_n_write(itf, (uint8_t const *) "OK\r\n", 4);
        //tud_cdc_n_write_flush(itf);
    }
}
