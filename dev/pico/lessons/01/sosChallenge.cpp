#include "rcc_stdlib.h"
using namespace std;
PID_control_config_t config;

typedef enum{
    STRAIGHT,
    SCAN_LEFT,
    SCAN_RIGHT,
    BACKWARDS,
    LEFT_WAIT,
    RIGHT_WAIT,
    MOVE_LEFT,
    MOVE_RIGHT
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
    int bp = 75; // Base power
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

        //cout << "Dist: " << dist << "\n";

        switch(state){
            case STRAIGHT:{ //Go straight until an object is in front of it
                cout << "DRIVE STRAIGHT\n";
                drive = 1;
                setpoint = 0.0;
                ServoPosition(&s3, 50); //faces forward
                if(dist <= 200) { //checks for object
                    //cout<< "Scanning forward";
                    state = SCAN_LEFT;
                    cout << "Entering SCAN_LEFT\n";
                    sm_prev = cur;
                    drive = 0;
                } 
                break;
            }
            case SCAN_LEFT:{   
                cout << "SCAN_LEFT\n";             
                //setpoint = 90.0;
                //drive = 0;
                //cout<< "Stopping";
                ServoPosition(&s3, 100); //Faces left
                state = LEFT_WAIT;
                break;
            }
            case MOVE_LEFT:{
                dist = getFastReading(&lidar);
                if(dist <= 200) { //Checking if there is an object to it's left
                    //cout<< "scanning left";
                    state = SCAN_RIGHT; //If there is, it scans the right side to look for another option
                    cout << "Entering SCAN_RIGHT\n";
                    sm_prev = cur;
                    drive = 0;
                }
                if(dist >= 200) {
                    drive = 1;
                    setpoint = 90.0;
                    cout<< "DRIVE LEFT\n";
                    //If there is not an opject left of it we keep moving
                    //cout<<"moving left";
                    //ServoPosition(&s3, 0); //Looks at the side where we want to end
                    /*if(cur - sm_prev >= change_dt){ //Checks every 3 seconds to see if there is a path to go straight
                        if(dist <= 200){
                        setpoint = 90.0;
                        }
                        if(dist >= 200){
                            state = STRAIGHT;
                            sm_prev = cur;
                        }
                    }*/
                    

                    //sleep_ms(3000);
                    /*if(dist <= 200){
                        setpoint = 90.0;
                    }
                    if(dist >= 200){
                            state = STRAIGHT;
                            cout << "Entering STRAIGHT\n";
                            sm_prev = cur;
                    } */
                    //check lidar every 2 seconds
                }
                break;
            }
            
            case SCAN_RIGHT:{
                cout << "SCAN_RIGHT\n";
                //if(left_count >= 145 || right_count >= 145) {
                //    state = STRAIGHT;
                //    sm_prev = cur;
                //} 
                // cout << dist << "\n";
                //drive = 0;
                ServoPosition(&s3, 0); //faces forward
                //sleep_ms(500);
                state = RIGHT_WAIT;
                break;
            }
            case MOVE_RIGHT:{
                dist = getFastReading(&lidar);
                if(dist <= 200){
                    //drive = 0;
                    state = BACKWARDS;//CHANGE1
                    sm_prev = cur;
                    drive = 0;
                    //sleep_ms(500);//CHANGE2
                }
                if(dist >= 200){
                    //ServoPosition(&s3, 50); //CHANGE3
                    drive = 1;
                    setpoint = -90.0;
                }

                break;
            }
            case BACKWARDS:{
                cout << "BACKWARDS\n";
                //make robot go backwards for three seconds and repeat the search here
                ServoPosition(&s3, 50); //faces forward
                cout << "go backwards";
                if(cur - sm_prev >= change_dt) {
                    state = SCAN_LEFT;
                }

                break;
            }
            case LEFT_WAIT:{
                sleep_ms(500);
                state = MOVE_LEFT;
                break;
            }
            case RIGHT_WAIT:{
                sleep_ms(500);
                state = MOVE_RIGHT;
                break;
            }
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