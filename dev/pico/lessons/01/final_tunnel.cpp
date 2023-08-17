#include "rcc_stdlib.h"
using namespace std;
PID_control_config_t config;

typedef enum{
    STRAIGHT,
    SCAN_LEFT,
    SCAN_RIGHT,
    STRAIGHT_WAIT,
    LEFT_WAIT,
    RIGHT_WAIT,
    MOVE_LEFT,
    MOVE_RIGHT,
    L_WAIT,
    L_GO,
    R_WAIT,
    R_GO,
    SCAN_STRAIGHT
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
    config.kd = 0.001;
    config.ts = 0.01; // 10ms
    config.sigma = 0.1;
    config.lowerLimit = -25;
    config.upperLimit = 25;
    config.antiWindupEnabled = true;
    
    //Controller vars
    PID_control controller(config);
    // controller.setDeadbands(-50, 50);
    int bp = 75; // Base power
    int output;
    float setpoint, theta;
    setpoint=0;
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
    setpoint_t state = SCAN_STRAIGHT;
    uint32_t change_dt = 3e6; // MGHELP: This might be too long thats why robot drives into object

    int drive = 0;
    uint16_t dist;

    while(true) {
        cur = time_us_32();
        left_count = left.getCount();
        right_count = right.getCount();
        dist = getFastReading(&lidar);

        // cout << "Dist: " << dist << "\n";

        switch(state){
            case SCAN_STRAIGHT:{ //Go straight until an object is in front of it
                //cout << "DRIVE STRAIGHT\n";
                //drive = 1;
                //setpoint = 0.0;
                ServoPosition(&s3, 50); //faces forward
                state = STRAIGHT_WAIT;
                break;
            }
            case STRAIGHT:{
                dist = getFastReading(&lidar);
                if (dist > 250){
                    drive = 1;
                    // setpoint = 0.0;
                    // cout << "driving straight";
                }
                if(dist <= 250) { //checks for object
                    //cout<< "Scanning forward";
                    state = SCAN_LEFT;
                    // cout << "Entering SCAN_LEFT\n";
                    sm_prev = cur;
                    drive = 0;
                } 
                break;
            }
            case SCAN_LEFT:{   
                // cout << "SCAN_LEFT\n";             
                //setpoint = 90.0;
                //drive = 0;
                //cout<< "Stopping";
                ServoPosition(&s3, 100); //Faces left
                state = LEFT_WAIT;
                break;
            }
            case MOVE_LEFT:{
                dist = getFastReading(&lidar);
                if(dist <= 250) { //Checking if there is an object to it's left
                    //cout<< "scanning left";
                    state = SCAN_RIGHT; //If there is, it scans the right side to look for another option
                    // cout << "Entering SCAN_RIGHT\n";
                    sm_prev = cur;
                    drive = 0;
                }
                
                if(dist > 250){
                    state = L_WAIT;
                }
                break;
            }
            case L_GO:{
                ServoPosition(&s3, 50); //Faces left
                // setpoint = setpoint + 90.0; // MGHELP: This was getting called very often
                drive = 1;                
                // cout<< "DRIVE LEFT\n"; 
                if(cur - sm_prev >= change_dt) {
                    if (dist <= 250){
                        state = SCAN_STRAIGHT; //CHANGE
                        sm_prev = cur;
                        drive = 0;
                    }
                }
                break;
            }
            
            case SCAN_RIGHT:{
                // cout << "SCAN_RIGHT\n";
                ServoPosition(&s3, 0); //faces forward
                //sleep_ms(500);
                state = RIGHT_WAIT;
                break;
            }
            case MOVE_RIGHT:{
                dist = getFastReading(&lidar);
                if(dist <= 250){
                    //drive = 0;
                    state = STRAIGHT_WAIT;//CHANGE1
                    sm_prev = cur;
                    drive = 0;
                    //sleep_ms(500);//CHANGE2
                }
                if(dist > 250){
                    state = R_WAIT;
                }
                break;
            }
            case R_GO:{
                ServoPosition(&s3, 50); //Faces left
                drive = 1;
                // setpoint = setpoint-90; // MGHELP: This was getting called very often!
                if(cur - sm_prev >= change_dt) {
                    if (dist <= 250){
                        state = SCAN_STRAIGHT; //If there is, it scans the right side to look for another option
                        sm_prev = cur;
                        drive = 0;
                    }
                }
                break;
            }
            case STRAIGHT_WAIT:{
                MotorPower(&motors, 0, 0);
                sleep_ms(500);
                state = STRAIGHT;
                break;
            }
            case LEFT_WAIT:{
                MotorPower(&motors, 0, 0);
                sleep_ms(500);
                state = MOVE_LEFT;
                break;
            }
            case RIGHT_WAIT:{
                MotorPower(&motors, 0, 0);
                sleep_ms(500);
                state = MOVE_RIGHT;
                break;
            }
            case L_WAIT:{
                MotorPower(&motors, 0, 0);
                sleep_ms(500);
                state = L_GO;
                setpoint = setpoint + 90; // MGHELP: Adjust setpoint here once before moving
                break;
            }
            case R_WAIT:{
                MotorPower(&motors, 0, 0);
                sleep_ms(500);
                state = R_GO;
                setpoint = setpoint - 90; // MGHELP: Adjjust setpoint here once before moving
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
            cout << "drive: " << drive << " state: " << state << " theta: " << theta << " setpoint: " << setpoint <<
             " powers: " << bp-output << " | " << bp+output << "\n";
        }
        } else {
            MotorPower(&motors, 0, 0);
        }
        // Control loop   
    }
}