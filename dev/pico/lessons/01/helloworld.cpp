#include "rcc_stdlib.h"
using namespace std;
PID_control_config_t config;
typedef enum{
    STRAIGHT,
    LEFT,
    RIGHT
} setpoint_t;
int main() {
    stdio_init_all();
    rcc_init_i2c();
    cyw43_arch_init();
    sleep_ms(1500);
    MPU6050 imu;
    imu.begin(i2c1);
    imu.calibrate();
    Motor motors;
    MotorInit(&motors, RCC_ENA, RCC_ENB, 20000);
    MotorsOn(&motors);
    cyw43_arch_gpio_put(0, 1);
    
    config.kp = 0.4;
    config.ki = 0.0;
    config.kd = 0.000;
    config.ts = 0.01; // 10ms
    config.sigma = 0.1;
    config.lowerLimit = -25;
    config.upperLimit = 25;
    config.antiWindupEnabled = true;
    
    //Controller vars
    PID_control controller(config);
    int bp = 75; // Base power
    int output;
    float setpoint, theta;
    theta = 0;
    // Sm and timing vars
    uint32_t cur, ctrl_prev, sm_prev;
    cur = time_us_32();
    ctrl_prev = sm_prev = cur;
    setpoint_t state;
    uint32_t change_dt = 3e6;
    while(true) {
        cur = time_us_32();
        switch(state){
            case STRAIGHT:{
                setpoint = 0.0;
                if(cur - sm_prev >= change_dt) {
                    state = LEFT;
                    sm_prev = cur;
                } 
                break;
            }
            case LEFT:{                
                setpoint = 90.0;
                if(cur - sm_prev >= change_dt) {
                    state = RIGHT;
                    sm_prev = cur;
                } 
                break;
            }
            case RIGHT:{
                setpoint = -90.0;
                if(cur - sm_prev >= change_dt) {
                    state = STRAIGHT;
                    sm_prev = cur;
                } 
                break;
            }
        }
        // Control loop
        if(cur - ctrl_prev >= config.ts*1e6) {
            imu.update_pico();
            //Theta
            theta += imu.getAngVelZ()*config.ts;
            //Calc controller output
            output = controller.pid(setpoint, theta);
            //Apply differential output to motors
            MotorPower(&motors, bp-output, bp+output);
            ctrl_prev = cur;
        }
        //print
        cout << "theta: " << theta << " setpoint: " << setpoint << '\n';
    }
}