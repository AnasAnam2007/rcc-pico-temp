#include "rcc_stdlib.h"
using namespace std;



int main(void){
    stdio_init_all();
    cyw43_arch_init();
    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar);
    Motor motors;
    MotorInit(&motors, RCC_ENA, RCC_ENB, 20000);
    MotorsOn(&motors);
    MotorPower(&motors, 0, 0);

    //control system variables
    float desired = 200;
    float actual;
    uint16_t lidar_reading;
    float error; 
    float kp = 1;
    float power;

    while(true){
        //get dist from lidar
        lidar_reading = getFastReading(&lidar);
        //convert the lidar dist from uint to float
        actual = static_cast<float>(lidar_reading);
        //calc error
        error = desired - actual;
        //calc the controller output
        power  = kp*error;
        //saturate the controller output
        power = max(min(power, 100.f), -100.f);
        //convert power to an integer
        int power_int = static_cast<int>(power);
        //Apply the controller output(power) to motors
        MotorPower(&motors, power_int, power_int);
        cout<<"des: "<<desired << " act: " << actual << " err: " << error << " power: " << power_int << "\n";
    }
}