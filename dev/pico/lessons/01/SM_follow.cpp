#include "rcc_stdlib.h"
using namespace std;

typedef enum{
    FWD,
    STOP,
    REV
} state_t;

int main(){
    stdio_init_all();
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1); //turns on led

    //init robot's sensors and actuators here~~
    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    Motor motors; //struct setup
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000); //setup 
    MotorsOn(&motors); //enable PWM
    uint16_t dist;

    int goal_dist = 200; //mm --> 20 cm
    int tolerance = 50; //mm --> 5 cm 

    //rename state_name to something more informative
    state_t car_state = STOP; //initial state

    while(true){

        dist = getFastReading(&lidar); 

        switch(car_state){

            case STOP:
                MotorPower(&motors, 0, 0); //stop
                //transition conditions:
                if(dist >= (goal_dist + tolerance)){ 
                    car_state = FWD;
                }
                if(dist <= (goal_dist - tolerance)){ 
                    car_state = REV;
                }
                break;

            case FWD:
                MotorPower(&motors, 68, 70); //forwards
                //transition conditions: 
                if(dist <= (goal_dist + tolerance)){ 
                    car_state = STOP;
                }
                break;

            case REV:
                MotorPower(&motors, -68, -70); //backwards
                //transition conditions:
                if(dist >= (goal_dist - tolerance)){ 
                    car_state = STOP;
                }
                break;
        }

    }

}
