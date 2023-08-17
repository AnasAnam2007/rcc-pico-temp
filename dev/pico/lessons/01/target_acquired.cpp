#include "rcc_stdlib.h"
using namespace std;

/*

int main(void)
{
    stdio_init_all();
    sleep_ms(1000);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1);//led on

    rcc_init_pushbutton();

    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    Servo s3; //struct
    ServoInit(&s3, 18, false, 50); //setup on pin 18
    ServoOn(&s3); //enables PWM

    Motor motors;
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);
    MotorsOn(&motors);
    printf("AFTER MOTORS");

    Left_Odom left;
    Right_Odom right;
    printf("AFTER ODOM");

    int left_count = 0;
    int right_count = 0;
    float scale_factor = 1.2; //can vary greatly between motors
    int base_power = 50;

    int pos = 0;
    int smallest_num = 100000;
    int turn_pos;
    while(true){
        left_count = left.getCount();
        right_count = right.getCount();






        
        if(!gpio_get(RCC_PUSHBUTTON)) //if NOT gpio (if gpio is low)
        {  
            uint16_t dist = getFastReading(&lidar);//takes in the distance for that servo psoition
            if(smallest_num < dist){
                smallest_num = dist;
                cout << smallest_num;
                turn_pos = pos;
                //degree turn with pos
            }
            pos = pos + 10; //changes servo pos
        }
        //if(pos == 100){
        //    pos = 0;
        //}

        





    if(!gpio_get(RCC_PUSHBUTTON)){

        for(int i = 0; i <= 11; i++) {
            pos = pos + 10;
            uint16_t dist = getFastReading(&lidar);//takes in the distance for that servo psoition
            if(smallest_num > dist){
                smallest_num = dist;
                turn_pos = pos;//degree turn with pos
            }
            if(turn_pos == 50){
                MotorPower(&motors, 68, 70);
            }
            else if(turn_pos == 90){
                MotorPower(&motors, 0, 70);
                if(left_count >= 79){
                MotorPower(&motors, 0, 0);
            }
            else if(turn_pos == 60){
                MotorPower(&motors, 0, 70);
            }


        ServoPosition(&s3, pos);
        sleep_ms(200);
    }
}
}
}
*/

int main(void)
{
    stdio_init_all();
    sleep_ms(1000);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0, 1);//led on

    rcc_init_pushbutton();

    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    Servo s3; //struct
    ServoInit(&s3, 18, false, 50); //setup on pin 18
    ServoOn(&s3); //enables PWM

    Motor motors;
    MotorInit(&motors, RCC_ENA, RCC_ENB, 1000);
    MotorsOn(&motors);
    printf("AFTER MOTORS");

    Left_Odom left;
    Right_Odom right;
    printf("AFTER ODOM");

    int left_count = 0;
    int right_count = 0;
    float scale_factor = 1.2; //can vary greatly between motors
    int base_power = 50;

    int pos = 0;
    int smallest_num = 100000;
    int turn_pos;
    while(true){
        left_count = left.getCount();
        right_count = right.getCount();
        if(!gpio_get(RCC_PUSHBUTTON)) //if NOT gpio (if gpio is low)
        {  

            for(int i = 0; i < 10; i++){
                ServoPosition(&s3, pos);
                sleep_ms(200);
                pos = pos + 10;
                uint16_t dist = getFastReading(&lidar);//takes in the distance for that servo psoition
                if(smallest_num > dist){
                smallest_num = dist;
                turn_pos = pos;//degree turn with pos
            }
            }
            if(turn_pos > 50){
                MotorPower(&motors, 70, 0);
                left_count = left.getCount();
                if(left_count >= 79){
                MotorPower(&motors, 0, 0); //stop
            }
            }
        }  

    }
}