#include "rcc_stdlib.h"
using namespace std;

typedef enum{
    LED_OFF, //0
    LED_ON   //1
} state_t;

int main(){
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1); //turns on led, but just for a moment lol

    //setup timing variables
    int64_t current_time = time_us_64(); 
    int64_t previous_time = time_us_64(); 

    //400 milliseconds converted to microseconds
    int blink_time = 400000; 

    state_t led_state = LED_OFF; //initial is OFF

    while(true){
        //update current time
        int64_t current_time = time_us_64(); 

        switch(led_state){
            case LED_ON:
                cyw43_arch_gpio_put(0, 1);  // led on

                //transition condition: has blink_time elapsed
                if(current_time - previous_time >= blink_time){
                    led_state = LED_OFF;
                    previous_time = current_time; 
                }
                break;

            case LED_OFF:
                cyw43_arch_gpio_put(0, 0); //led off

                //transition condition: has blink_time elapsed
                if(current_time - previous_time >= blink_time){
                    led_state = LED_ON;
                    previous_time = current_time; 
                }
                break;
        }
    }
}