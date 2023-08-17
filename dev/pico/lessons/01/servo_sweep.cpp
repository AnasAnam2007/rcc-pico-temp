#include "rcc_stdlib.h"
using namespace std;



int main(void)
{
    stdio_init_all();
    sleep_ms(1000);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1);//led on

    rcc_init_pushbutton();

    Servo s3; //struct
    ServoInit(&s3, 18, false, 50); //setup on pin 18
    ServoOn(&s3); //enables PWM

    int pos = 0;

    while(true){
        if(!gpio_get(RCC_PUSHBUTTON)) //if NOT gpio (if gpio is low)
        {  
          pos = pos + 10;
        }
        if(pos == 100){
            pos = 0;
        }

        ServoPosition(&s3, pos);
        sleep_ms(100);
    }

}