#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"

#define LED_PIN 25

volatile bool led_state = false; 

bool tx_timer_callback(struct repeating_timer *t)
{
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
    return true;
}

int main()
{
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
       
    struct repeating_timer txtim_t;
    
    // Negative delay (-10) means the timer will fire every 10 ms
    // regardless of how long the ISR takes to execute
    // 100 Hz = 10 ms
    add_repeating_timer_ms(-10, tx_timer_callback, NULL, &txtim_t);

    while(1)
    {
        //            
    }
}
