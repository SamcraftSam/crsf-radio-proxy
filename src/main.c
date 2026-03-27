#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"

#include "crsf.h"

#define LED_PIN 25

#define UART_ID uart0
#define BAUD_RATE 420000
#define UART_TX_PIN 0
#define UART_RX_PIN 1


volatile bool led_state = false; 


int main()
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    //struct repeating_timer txtim_t;
    
    // Negative delay (-10) means the timer will fire every 10 ms
    // regardless of how long the ISR takes to execute
    // 100 Hz = 10 ms
    //add_repeating_timer_ms(-10, tx_timer_callback, NULL, &txtim_t);
    
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // --- THE INVERSION BIT ---
    // Uncomment these if your hardware actually requires inverted logic
    // gpio_set_outover(UART_TX_PIN, GPIO_OVERRIDE_INVERT);
    // gpio_set_inover(UART_RX_PIN, GPIO_OVERRIDE_INVERT);

    uint16_t channels[16];
    for (int i = 0; i < 16; i++)
    {
        channels[i] = CRSF_CHANNEL_MID;
    }
    
    uint8_t tx_buffer[26];
    tx_buffer[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    tx_buffer[1] = 24;
    tx_buffer[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    
    PACK_CRSF_CHANNELS(tx_buffer, channels);

    tx_buffer[25] = crsf_crc8(&tx_buffer[2], 23);

    int ctrl_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    // DREQ_UART0_TX ensures DMA only sends when UART FIFO has space
    channel_config_set_dreq(&c, DREQ_UART0_TX);

    while(1)
    {

        dma_channel_configure(
            ctrl_chan,
            &c,
            &uart_get_hw(UART_ID)->dr,
            tx_buffer,
            26,
            true
        );

        dma_channel_wait_for_finish_blocking(ctrl_chan);

        printf("CRSF Packet Sent via DMA!\r\n");

        printf("DMA transfer_count before: %lu\r\n", dma_hw->ch[ctrl_chan].transfer_count);
        printf("PING\r\n");

        sleep_ms(5000);
    }
}
