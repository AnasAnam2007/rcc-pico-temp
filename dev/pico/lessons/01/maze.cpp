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
    
    config.kp = 0.6;
    config.ki = 0.00;
    config.kd = 0.000;
    config.ts = 0.01; // 10ms
    config.sigma = 0.1;
    config.lowerLimit = -25;
    config.upperLimit = 25;
    config.antiWindupEnabled = true;
    
    //Controller vars
    PID_control controller(config);
    int bp = 40; // Base power
    int output;
    float setpoint, theta;
    theta = 0;
    // Sm and timing vars

    Left_Odom left;
    Right_Odom right;
    //printf("AFTER ODOM");

    int left_count = 0;
    int right_count = 0;

    Servo s3;
    ServoInit(&s3, 18, false, 50);
    ServoOn(&s3);

    rcc_init_i2c(); //setup pico i2c
    VL53L0X lidar; //class 
    rcc_init_lidar(&lidar); //setup lidar

    uint32_t cur, ctrl_prev, sm_prev;
    cur = time_us_32();
    ctrl_prev = sm_prev = cur;
    setpoint_t state = STRAIGHT;
    uint32_t change_dt = 3e6;

    int drive = 0;
    uint16_t dist;

    while(true) {
        cur = time_us_32();
        left_count = left.getCount();
        right_count = right.getCount();
        dist = getFastReading(&lidar);

        cout << "Dist: " << dist << "\n";

        switch(state){
        case STRAIGHT: {
            ServoPosition(&s3, 50); //faces forward
            sleep_ms(500);
            dist = getFastReading(&lidar);
                if (dist > 200){
                MotorPower(&motors, 50, 50);
                    cout << "driving straight";
                }
                if(dist <= 200) { //checks for object
                    //cout<< "Scanning forward";
                    state = LEFT;
                    cout << "Entering SCAN_LEFT\n";
                    sm_prev = cur;
                    drive = 0;
                } 
                break;

        }
        case LEFT: {
            ServoPosition(&s3, 100);
            dist = getFastReading(&lidar);
            sleep_ms(500);
                if(dist <= 200) { //Checking if there is an object to it's left
                    //cout<< "scanning left";
                    state = RIGHT; //If there is, it scans the right side to look for another option
                    cout << "Entering SCAN_RIGHT\n";
                    sm_prev = cur;
                    drive = 0;
                }
                if(dist >= 200){
                    ServoPosition(&s3, 50); // Center servo
                    while (right.getCount() < 30) {
                        MotorPower(&motors, 0, 60);
                    }
                    // drive forwards
                    MotorPower(&motors, 60, 60);
                    cout<< "DRIVE RIGHT\n"; 
                }
            break;
        }

        case RIGHT: {
            ServoPosition(&s3, 0);
            dist = getFastReading(&lidar);
                if(dist <= 200) { //Checking if there is an object to it's left
                    //cout<< "scanning left";
                    state = STRAIGHT;//undefined; //If there is, it scans the right side to look for another option
                    cout << "Entering SCAN_STRAIGHT\n";
                    sm_prev = cur;
                    drive = 0;
                }
                if(dist >= 200){
                    ServoPosition(&s3, 50); // Center servo
                    while (left.getCount() < 30) {
                        MotorPower(&motors, 60, 0);
                    }
                    // drive forwards
                    MotorPower(&motors, 60, 60);
                    cout<< "DRIVE RIGHT\n"; 
                }
            break;
        }

        if(drive) {
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
        } else {
            MotorPower(&motors, 0, 0);
        }
        // Control loop   
    }
}
}