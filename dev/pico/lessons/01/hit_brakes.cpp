#include "rcc_stdlib.h"
using namespace std;

int main()
{   
    stdio_init_all();
    sleep_ms(100);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1); //turns on led

    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    rcc_init_pushbutton();

    Motor motors;
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);
    MotorsOn(&motors);
    

    while(true)
    {
        if(!gpio_get(RCC_PUSHBUTTON))
        {
            MotorPower(&motors, 68, 72); //going forward
        }
        uint16_t dist = getFastReading(&lidar);
        if (dist < 300){ //checks for objects
            MotorPower(&motors, 0, 0); //stops
            
        }
        sleep_ms(100);
    }

}