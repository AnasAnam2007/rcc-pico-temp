/*#include "rcc_stdlib.h"
using namespace std;

typedef enum{
    WAIT,        //0
    SEARCHING,   //1
    FOUND,       //2
    ROTATING_L,  //3
    ROTATING_R,  //4
    APPROACHING //5
} state_t;

void print_state(state_t state){
    cout << state << '\n';
}

int main()
{
    stdio_init_all();    
    sleep_ms(1500);
    cyw43_arch_init();
    cyw43_arch_gpio_put(0,1);

    rcc_init_pushbutton();

    Servo s3;
    ServoInit(&s3, 18, false, 50);
    ServoOn(&s3);

    Motor motors;
    MotorInit(&motors, RCC_ENB, RCC_ENA, 1000);
    MotorsOn(&motors);

    rcc_init_i2c();

    Left_Odom left;
    Right_Odom right;

    VL53L0X lidar;
    rcc_init_lidar(&lidar);

    int left_count = 0;
    int right_count = 0;
    float scale_factor = 1.2; //can vary greatly between motors
    int base_power = 50;

    int positions[100]; //array for servo positions

    for (int i = 0; i < 100; ++i) {
        positions[i] = i;  //filling array 0-99
    }

    int i = 0;
    uint16_t distance;

    state_t state = WAIT;

    while(true) {   
        print_state(state); 

        left_count = left.getCount();
        right_count = right.getCount();

        if(!gpio_get(RCC_PUSHBUTTON))
        {
            state = SEARCHING;
        }

        distance = getFastReading(&lidar); 

        switch(state){
            case WAIT:{
                MotorPower(&motors, 0, 0); //stop
                if(distance > 200){
                    state = SEARCHING;
                }
            break;}

            case SEARCHING:
                ServoPosition(&s3, positions[i]);
                sleep_ms(100);
                i++;
                if (i >= 100){
                    i = 0; //reset counter
                }
                //transition condition
                if(distance <= 200){
                    state = FOUND;
                }
                break;

            case FOUND:
                ServoPosition(&s3, 100);
                MotorPower(&motors, 68, 70);

                if (distance <= 200){  //object was on left
                    int backward_counts = left_count/2;
                    state = ROTATING_R;
                }

                break;

            case ROTATING_L:
                MotorPower(&motors, 50, 0); //rotate left
                if(left_count <= 40){
                    state = APPROACHING;
                }
                break;

            case ROTATING_R:
                int backward_counts = left_count/2;
                MotorPower(&motors, -68, -70); //rotate right

                if(left_count || right_count >= backward_counts){
                    state = ROTATING_L;
                }
                break;

            case APPROACHING:
               MotorPower(&motors, 68, 70); //forwards
                if(distance <= 100){
                    state = WAIT;
                
                break;
            }
        }


    }

}
*/

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
    STRAIGHT_WAIT,
    MOVE_LEFT,
    MOVE_RIGHT,
    CHECK_PATH,
    CHECK_PATH_LEFT,
    LEFT_LEFT_WAIT
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
    uint32_t change_dt = 2e6;

    int drive = 0;
    uint16_t dist;

    while(true) {
        cur = time_us_32();
        left_count = left.getCount();
        right_count = right.getCount();
        dist = getFastReading(&lidar);

        cout << "Dist: " << dist << "\n";

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
                if(dist <= 300) { //Checking if there is an object to it's left
                    //cout<< "scanning left";
                    state = SCAN_RIGHT; //If there is, it scans the right side to look for another option
                    cout << "Entering SCAN_RIGHT\n";
                    sm_prev = cur;
                    drive = 0;
                }
                if(dist >= 300) {
                    state = CHECK_PATH_LEFT;
                    /*drive = 1;
                    setpoint = 90.0;
                    ServoPosition(&s3, 0); 
                    if(cur - sm_prev >= change_dt) {
                        state = CHECK_PATH;
                    }*/
                }
                break;
            }
            case CHECK_PATH_LEFT:{
                drive = 1;
                setpoint = 90.0; 
                cout << "DRIVE LEFT";
                ServoPosition(&s3, 0); 
                if(cur - sm_prev >= change_dt) {
                state = CHECK_PATH;
                drive = 0;
                cout << "CHECKING OPENING";
                }
                break;
            }
            case CHECK_PATH:{
                if (dist >= 300){
                    state = STRAIGHT_WAIT;
                    cout << "GOING THRU OPENING";
                }
                if (dist <= 300) {
                    state = LEFT_LEFT_WAIT;
                    cout << "STILL GOING LEFT";
                }
            }
            case SCAN_RIGHT:{
                cout << "SCAN_RIGHT\n";
                ServoPosition(&s3, 0); //faces forward
                //sleep_ms(500);
                state = RIGHT_WAIT;
                break;
            }
            case MOVE_RIGHT:{
                dist = getFastReading(&lidar);
                if(dist <= 300){
                    state = BACKWARDS;
                    sm_prev = cur;
                    drive = 0;
                }
                if(dist >= 300){
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
            case STRAIGHT_WAIT:{
                ServoPosition(&s3, 50); //faces forward
                sleep_ms(500);
                state = STRAIGHT;
                break;
            }
            case LEFT_LEFT_WAIT:{
                sleep_ms(500);
                state = CHECK_PATH_LEFT;
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
