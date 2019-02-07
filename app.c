/**
 * This program balances a two-wheeled Segway type robot.
 * This program is based on the source code of gyroboy program in ev3rt sdk
 * Develped by: Anupam Godse (angodse@ncsu.edu) and Brayden McDonald (bwmcdon2@ncsu.edu)
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"
#include <ctype.h>

#define abs(x) x>0?x:-x
#define max(a, b) a>b?a:b;
#define min(a, b) a>b?b:a;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * Define the connection ports of the gyro sensor and motors.
 * By default, the Gyro Boy robot uses the following ports:
 * Gyro sensor: Port 2
 * Left motor:  Port A
 * Right motor: Port D
 */
const int color_sensor=EV3_PORT_3, touch_sensor = EV3_PORT_1, gyro_sensor = EV3_PORT_2, left_motor = EV3_PORT_A, right_motor = EV3_PORT_D;

/**
 * Constants for the self-balance control algorithm.
 */
const float KSTEER=-0.25;
const float EMAOFFSET = 0.0005f, KGYROANGLE = 7.5f, KGYROSPEED = 1.15f, KPOS = 0.07f, KSPEED = 0.1f, KDRIVE = -0.02f;
const float WHEEL_DIAMETER = 5.6;
const uint32_t WAIT_TIME_MS = 5;
const uint32_t FALL_TIME_MS = 1000;
const float INIT_GYROANGLE = -0.25;
const float INIT_INTERVAL_TIME = 0.014;


/**
 * Global variables used by the self-balance control algorithm.
 */
static int motor_diff, motor_diff_target;
static int loop_count, motor_control_drive; //motor_control_steer;
static float gyro_offset, gyro_speed, gyro_angle, interval_time, motor_control_steer;
static float motor_pos, motor_speed;
uint8_t c=0;
int is_pressed=0, gyro_active=0, direction = 1, turning_count=0, turning = 0, target_count;
int val = 0;


static FILE *bt = NULL;


SYSTIM start_b, end_b, start_g, end_g, start_m, end_m, max_b, max_g, max_m, start_t, end_t, max_t;


void task_activator(intptr_t tskid) {
    ER ercd = act_tsk(tskid);
    assert(ercd == E_OK);
}


/* this function processes the commands stored in variable "c" by the background
 * task (main) which reads the bluetooth console file to get user inputs.
 * It sets the motor_control_drive and motor_control_steer variables according
 * to the input commands to navigate
 */
void bluetooth_task(intptr_t unused) {

    get_utm(&start_b); 

    /*if the segway is busy turning then return*/
    if(turning && turning_count < target_count) {
        turning_count++;
        get_utm(&end_b); 
        max_b = max_b<(end_b-start_b)?end_b-start_b:max_b;
        return;
    } 
    else {//else reset the related variables
        turning = 0;
        turning_count=0;
        motor_control_steer=0;
    }

    //switch based on the user command stored in "c" by main
    switch(c) {
        /*move forward: set segway to move in forward direction
         * with some initial speed
         */
        case 'f':
            /*if not moving or moving backward then set inital speed
             * in forward direction, if moving forward then keep 
             * the same speed
             */
            motor_control_drive = max(motor_control_drive, 200);
            direction = 1;
            fprintf(bt, "motor_control_drive: %d\n", motor_control_drive);
            break;

        /*move backward: set segway to move in backward direction
         * with some initial speed
         * */
        case 'b':
            /*if not moving or moving forward then set inital speed
             * in backward direction, if moving backward then keep 
             * the same speed
             */
            motor_control_drive = min(motor_control_drive, -200);
            direction = -1;
            fprintf(bt, "motor_control_drive: %d\n", motor_control_drive);
            break;

        /*speed up: speed up segway in ongoing direction
         * if not moving forward/backward do nothing
         */
        case 'u':
            if(motor_control_drive != 0)
                motor_control_drive += 200*direction;
            fprintf(bt, "motor_control_drive: %d\n", motor_control_drive);
            break;

        /*speed down: speed down segway in ongoing direction
         * if not moving forward/backward do nothing
         */
        case 'd':
            if(motor_control_drive != 0)
                motor_control_drive -= 200*direction;
            fprintf(bt, "motor_control_drive: %d\n", motor_control_drive);
            break;

        /* set the motor_control_steer variable according to the speed of the
         * segway to turn left by 90 degrees
         * Also sets "turning" and "target_count" to ensure periodically calling
         * bluetooth task does not mess with the current turn movement until
         * complete
         */
        case 'l':
            motor_control_steer = 50;
            turning = 1;
            if(abs(motor_control_drive)>400)
                target_count = 50;
            else 
                target_count = 35;
            fprintf(bt, "motor_control_steer: %f\n", motor_control_steer);
            break;

        /* set the motor_control_steer variable according to the speed of the
         * segway to turn right by 90 degrees
         * Also sets "turning" and "target_count" to ensure periodically calling
         * bluetooth task does not mess with the current turn movement until
         * complete
         */
        case 'r':
            motor_control_steer = -50;
            turning = 1;
            if(abs(motor_control_drive)>400)
                target_count = 50;
            else 
                target_count = 35;
            fprintf(bt, "motor_control_steer: %f\n", motor_control_steer);
            break;

        /* print the help menu */
        case 'h':
            fprintf(bt, "==========================\n");
            fprintf(bt, "Usage:\n");
            fprintf(bt, "Press 'f/F' to move forward\n");
            fprintf(bt, "Press 'b/B' to move backword\n");
            fprintf(bt, "Press 'u/U' to speed up\n");
            fprintf(bt, "Press 'd/D' to slow down\n");
            fprintf(bt, "Press 'l/L' to turn left\n");
            fprintf(bt, "Press 'r/R' to turn right\n");
            fprintf(bt, "Press 'h' for this message\n");
            fprintf(bt, "==========================\n");
            break;


    }
    /*once command is proceed set c to 0 to ensure main does not take further commands
     * unless previous command is processed
     */
    if(c!=0)
        c=0;

    get_utm(&end_b); 
    max_b = max_b<(end_b-start_b)?end_b-start_b:max_b;
    return;
}

/*
 * This function accesses the touch sensor.
 * It updates the value of the global is_pressed variable; 1 if the sensor is currently being pressed, 0 if it is not
 * It also toggles the value of gyro_active whenever is_pressed goes from 1 to 0 
 */
//touch sensor periodic task
void touch_sensor_task(intptr_t unused) {
    get_utm(&start_t); 
    bool_t val = ev3_touch_sensor_is_pressed(touch_sensor);
    if(val && is_pressed==0) {//if the button is pressed, set the is_pressed variable
        is_pressed = 1;
    }
    else if(!val && is_pressed==1) {//when the button is released, toggle the rover_active variable
        if(gyro_active == 1) {
            gyro_active = 0;
        }
        else {
            gyro_active= 1;
        }
        is_pressed = 0;
    }
    get_utm(&end_t); 
    max_t = max_t<(end_t-start_t)?end_t-start_t:max_t;
    return;
}

/**
 * Calculate the initial gyro offset for calibration.
 */
static ER calibrate_gyro_sensor() {
    int gMn = 1000, gMx = -100, gSum = 0;
    for (int i = 0; i < 200; ++i) {
        int gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
        gSum += gyro;
        if (gyro > gMx)
            gMx = gyro;
        if (gyro < gMn)
            gMn = gyro;
        tslp_tsk(4);
    }
    if(!(gMx - gMn < 2)) { // TODO: recheck the condition, '!(gMx - gMn < 2)' or '(gMx - gMn < 2)'
        gyro_offset = gSum / 200.0f;
        return E_OK;
    } else {
        return E_OBJ;
    }
}

/**
 * Calculate the average interval time of the main loop for the self-balance control algorithm.
 * Units: seconds
 */
static void update_interval_time() {
    static SYSTIM start_time;

    //fprintf("start time = %d")

    if(loop_count++ == 0) { // Interval time for the first time (use 6ms as a magic number)
        //interval_time = 0.006;
        interval_time = INIT_INTERVAL_TIME;
        ER ercd = get_tim(&start_time);
        assert(ercd == E_OK);
    } else {
        SYSTIM now;
        ER ercd = get_tim(&now);
        assert(ercd == E_OK);
        interval_time = ((float)(now - start_time)) / loop_count / 1000;
    }
}

/**
 * Update data of the gyro sensor.
 * gyro_offset: the offset for calibration.
 * gyro_speed: the speed of the gyro sensor after calibration.
 * gyro_angle: the angle of the robot.
 */
//the gyro sesnor periodic task
void update_gyro_data(intptr_t unused) {
    
    get_utm(&start_g); 
    int gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    gyro_offset = EMAOFFSET * gyro + (1 - EMAOFFSET) * gyro_offset;
    gyro_speed = gyro - gyro_offset;
    gyro_angle += gyro_speed * interval_time;
    
    get_utm(&end_g); 
    max_g = max_g<(end_g-start_g)?end_g-start_g:max_g;
    return;
}

/**
 * Update data of the motors
 */
//to be used in balance task 
static void update_motor_data() {
    static int32_t prev_motor_cnt_sum, motor_cnt_deltas[4];

    if(loop_count == 1) { // Reset
        motor_pos = 0;
        prev_motor_cnt_sum = 0;
        motor_cnt_deltas[0] = motor_cnt_deltas[1] = motor_cnt_deltas[2] = motor_cnt_deltas[3] = 0;
    }

    int32_t left_cnt = ev3_motor_get_counts(left_motor);
    int32_t right_cnt = ev3_motor_get_counts(right_motor);
    int32_t motor_cnt_sum = left_cnt + right_cnt;
    motor_diff = right_cnt - left_cnt; // TODO: with diff
    int32_t motor_cnt_delta = motor_cnt_sum - prev_motor_cnt_sum;

    prev_motor_cnt_sum = motor_cnt_sum;
    motor_pos += motor_cnt_delta;
    motor_cnt_deltas[loop_count % 4] = motor_cnt_delta;
    motor_speed = (motor_cnt_deltas[0] + motor_cnt_deltas[1] + motor_cnt_deltas[2] + motor_cnt_deltas[3]) / 4.0f / interval_time;
}

/**
 * Control the power to keep balance.
 * Return false when the robot has fallen.
 */
static bool_t keep_balance() {
    val++;
    static SYSTIM ok_time;

    if(loop_count == 1) // Reset ok_time
        get_tim(&ok_time);

    float ratio_wheel = WHEEL_DIAMETER / 5.6;

    // Apply the drive control value to the motor position to get robot to move.
    motor_pos -= motor_control_drive * interval_time;

    // This is the main balancing equation
    int power = (int)((KGYROSPEED * gyro_speed +               // Deg/Sec from Gyro sensor
                KGYROANGLE * gyro_angle) / ratio_wheel + // Deg from integral of gyro
            KPOS       * motor_pos +                // From MotorRotaionCount of both motors
            KDRIVE     * motor_control_drive +       // To improve start/stop performance
            KSPEED     * motor_speed);              // Motor speed in Deg/Sec

    // Check fallen
    SYSTIM time;
    get_tim(&time);
    if(power > -100 && power < 100)
        ok_time = time;
    else if(time - ok_time >= FALL_TIME_MS)
        return false;

    // Steering control
    motor_diff_target += motor_control_steer * interval_time;

    int left_power, right_power;

    // TODO: support steering and motor_control_drive
    int power_steer = motor_control_steer;//steer_amount;//(int)(KSTEER * (motor_diff_target - motor_diff));
    left_power = power + power_steer;
    right_power = power - power_steer;
    if(left_power > 100)
        left_power = 100;
    if(left_power < -100)
        left_power = -100;
    if(right_power > 100)
        right_power = 100;
    if(right_power < -100)
        right_power = -100;


    //set motor powers to balance/navigate segway
    ev3_motor_set_power(left_motor, (int)left_power);
    ev3_motor_set_power(right_motor, (int)right_power);

    return true;
}

//initialize gyro sensor and motors once at start
void balance_init() {
    ER ercd;
    /*RESET*/

    loop_count = 0;
    motor_control_drive = 0;
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
    //TODO: reset the gyro sensor
    ev3_gyro_sensor_reset(gyro_sensor);

    /**
     * Calibrate the gyro sensor and set the led to green if succeeded.
     */
    _debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor."));
    //_debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor2."));
    for(int i = 10; i > 0; --i) { // Max retries: 10 times.
        //_debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor3."));
        ercd = calibrate_gyro_sensor();
        //_debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor4."));
        if(ercd == E_OK) break;

        //_debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor5."));
        if(i != 1)
            syslog(LOG_ERROR, "Calibration failed, retry.");
        else {
           // _debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor6."));
            _debug(syslog(LOG_ERROR, "Max retries for calibration exceeded, exit."));
            return;
        }
    }
    _debug(syslog(LOG_INFO, "Calibration succeed, offset is %de-3.", (int)(gyro_offset * 1000)));
   // _debug(syslog(LOG_NOTICE, "Start calibration of the gyro sensor7."));
    gyro_angle = INIT_GYROANGLE;
    ev3_led_set_color(LED_GREEN);
}

/*
 *balances the segway using parameters set by gyro task
 *updates motor according to drive and steer variables
 *set by bluetooth console task
 */
void balance_task(intptr_t unused) {
    //if not active then stop the motors
    if(!gyro_active) {
        ev3_motor_set_power(left_motor, 0);
        ev3_motor_set_power(right_motor, 0);
        return;
    }

    get_utm(&start_m); 

    /**
     * Main loop for the self-balance control algorithm
     */

    // Update the interval time
    update_interval_time();

    // Update data of the gyro sensor
    //update_gyro_data();

    // Update data of the motors
    update_motor_data();

    // Keep balance
    if(!keep_balance()) {
        ev3_motor_stop(left_motor, false);
        ev3_motor_stop(right_motor, false);
        ev3_led_set_color(LED_RED); // TODO: knock out
        //syslog(LOG_NOTICE, "Knock out!");
        get_utm(&end_m); 
        max_m = max_m<(end_m-start_m)?end_m-start_m:max_m;
        return;
    }

    get_utm(&end_m); 
    max_m = max_m<(end_m-start_m)?end_m-start_m:max_m;
    //tslp_tsk(WAIT_TIME_MS);
}

static void button_clicked_handler(intptr_t button) {
    switch(button) {
        case BACK_BUTTON:
            syslog(LOG_NOTICE, "Back button clicked.");
            break;
        case LEFT_BUTTON:
            syslog(LOG_NOTICE, "Left button clicked.");
    }
}


/*not used*/
void idle_task(intptr_t unused) {
    while(1) {
        fprintf(bt, "Press 'h' for usage instructions.\n");
        tslp_tsk(1000);
    }
}

void main_task(intptr_t unused) {
    // Draw information
    lcdfont_t font = EV3_FONT_MEDIUM;
    ev3_lcd_set_font(font);
    int32_t fontw, fonth;
    ev3_font_get_size(font, &fontw, &fonth);
    char lcdstr[100];
    ev3_lcd_draw_string("App: Gyroboy", 0, 0);
    sprintf(lcdstr, "Port%c:Gyro sensor", '1' + gyro_sensor);
    ev3_lcd_draw_string(lcdstr, 0, fonth);
    sprintf(lcdstr, "Port%c:Left motor", 'A' + left_motor);
    ev3_lcd_draw_string(lcdstr, 0, fonth * 2);
    sprintf(lcdstr, "Port%c:Right motor", 'A' + right_motor);
    ev3_lcd_draw_string(lcdstr, 0, fonth * 3);

    // Register button handlers
    ev3_button_set_on_clicked(BACK_BUTTON, button_clicked_handler, BACK_BUTTON);
    ev3_button_set_on_clicked(ENTER_BUTTON, button_clicked_handler, ENTER_BUTTON);
    ev3_button_set_on_clicked(LEFT_BUTTON, button_clicked_handler, LEFT_BUTTON);

    // Configure sensors
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);

    // Configure motors
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    // Open Bluetooth file
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);
    setvbuf(bt, NULL, _IONBF, 0);

    //initialize gyro variables
    balance_init();

    //sleep while bluetooth is not connected
    while (!ev3_bluetooth_is_connected()) tslp_tsk(100);

    //_debug(syslog(LOG_NOTICE, "made it 8"));


    // Start periodic task for getting touch sensor readings
    ev3_sta_cyc(CYC_TOUCH_SENSOR_TASK);
    //_debug(syslog(LOG_NOTICE, "made it 9"));


    /*touch sensor task monitors if the button is pressed and 
     * sets the gyro_active variable to start the segway
     */
    while(!gyro_active) {
        tslp_tsk(10);
    }

    //_debug(syslog(LOG_NOTICE, "made it 10"));

    //start periodic task for getting gyro readings
    ev3_sta_cyc(CYC_GYRO_TASK);

    // Start periodic task for self-balancing
    ev3_sta_cyc(CYC_BALANCE_TASK);

    // Start periodic task for bluetooth console
    ev3_sta_cyc(CYC_BLUETOOTH_TASK);


    //_debug(syslog(LOG_NOTICE, "made it 11"));

    /*touch sensor task monitors if the button is pressed and 
     * resets the gyro_active variable to stop the segway
     */
    while(gyro_active) {
        while (!ev3_bluetooth_is_connected()) tslp_tsk(100);
        /* read from bluetooth console and store to global variable
         * gets processed by bluetooth console task
         * do not take further commands until previous command is processed
         */
        if(c == 0)
            c = tolower(fgetc(bt));
    }

    //stop all the cyclic tasks once the segway is stopped

    //stop task for getting gyro readings
    ev3_stp_cyc(CYC_GYRO_TASK);

    // Stop task for self-balancing
    ev3_stp_cyc(CYC_BALANCE_TASK);

    // Stop task for bluetooth console
    ev3_stp_cyc(CYC_BLUETOOTH_TASK);

    // Stop task for bluetooth console
    ev3_stp_cyc(CYC_TOUCH_SENSOR_TASK);
    

    //stop motors
    ev3_motor_set_power(left_motor, 0);
    ev3_motor_set_power(right_motor, 0);
    
    //fprintf(bt, "max_b: %lu\n", max_b);
    //fprintf(bt, "max_t: %lu\n", max_t);
    //fprintf(bt, "max_m: %lu\n", max_m);
    //fprintf(bt, "max_g: %lu\n", max_g);

}
