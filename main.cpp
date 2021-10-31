/*
 * File    : main.cpp
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : September 5, 2021
 *
 * Description:
 *   Main program entry point for RIT MSD Team P21009 microcontroller software.
 *   NXT FRDM-K64F MCU target.
 */
 
#include <mbed.h>
#include "msd.h"

// Instances
I2C i2c(PTE25, PTE24); //SDA,SCL
#ifdef PI
BufferedSerial serial(D1, D0, 576000);  // tx, rx
#else
BufferedSerial serial(USBTX, USBRX, 576000);  // tx, rx
#endif
Adafruit_BNO055 ob_head = Adafruit_BNO055(0, BNO055_ADDRESS_A, &i2c);
Adafruit_BNO055 ob_body = Adafruit_BNO055(1, BNO055_ADDRESS_B, &i2c);
AnalogIn pcb_in(A0);
DigitalOut pcb_s1(PTD0);
DigitalOut pcb_s0(PTC4);
InterruptIn btn(SW2);
DigitalOut led_red(LED_RED);
DigitalOut led_green(LED_GREEN);
DigitalOut led_blue(LED_BLUE);

// Create static event queue
static EventQueue queue(0);
// Queue events
auto e_collect_ob = make_user_allocated_event(collect_ob, 0, 3);
auto e_collect_scale = make_user_allocated_event(collect_scale, 4, 4);
auto e_collect_fsr = make_user_allocated_event(collect_fsr, 5, 12);
auto e_collect_imu = make_user_allocated_event(collect_imu, 13, 16);
auto e_calibrate = make_user_allocated_event(calibrate);
auto e_send = make_user_allocated_event(send);
auto e_startstop = make_user_allocated_event(print_startstop);

// Global variables
static systemMode mode = SETUP;
static bool calibrated = false;
static bool logging = false;


FileHandle *mbed::mbed_override_console(int fd) {
    return &serial;
}

/*
 *
 */
void collect_ob(int start_idx, int end_idx) {
    Adafruit_BNO055 *ob;
    sensorInstance *sensor;
    // Iterate over each Orientation Board
    for (int i = start_idx; i <= end_idx; i++) {
        sensor = &sensors[i];
        if (sensor->enabled) {
            // Get pointer to correct BN055 instance.
            if (sensor->location == HEAD)
                ob = &ob_head;
            else if (sensor->location == BODY)
                ob = &ob_body;
            // Retrieve acceleration or euler vector and store.
            if (sensor->datatype == ACCELERATION)
                ob_get_accel(ob, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else if (sensor->datatype == EULER)
                ob_get_euler(ob, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            sensor->newdata = true;
        }
    }
}

/*
 *
 */
void collect_scale(int start_idx, int end_idx) {
    sensorInstance *sensor;
    // Iterate over each Scale
    for (int i = start_idx; i <= end_idx; i++) {
        sensor = &sensors[i];
        if (sensor->enabled) {
            // Store the data.
            sensor->data[0] = 0;
            sensor->newdata = true;
        }
    }
}

/*
 *
 */
void collect_fsr(int start_idx, int end_idx) {
    sensorInstance *sensor;
    // Iterate over each FSR
    for (int i = start_idx; i <= end_idx; i++) {
        sensor = &sensors[i];
        if (sensor->enabled) {
            // Set mux selects going to the PCB.
            switch (sensor->location) {
                case FOREARM_LEFT:
                case RIB_LEFT:
                    pcb_s1 = 0;
                    pcb_s0 = 0;
                    break;
                case FOREARM_RIGHT:
                case RIB_RIGHT:
                    pcb_s1 = 0;
                    pcb_s0 = 1;
                    break;
                case KNEE_LEFT:
                case HIP_LEFT:
                    pcb_s1 = 1;
                    pcb_s0 = 0;
                    break;
                case KNEE_RIGHT:
                case HIP_RIGHT:
                    pcb_s1 = 1;
                    pcb_s0 = 1;
                    break;
                default:
                    pcb_s1 = 0;
                    pcb_s0 = 0;
            }
            // Read the analog value (average from ADC_SAMPLES).
            float samples[10];
            samples[0] = pcb_in.read();
            for (int i = 1; i < ADC_SAMPLES; i++) {
                samples[i] = (pcb_in.read() + samples[i-1])/2.0;
            }
            // Store the data.
            sensor->data[0] = samples[ADC_SAMPLES-1];
            sensor->newdata = true;
        }
    }
}

/*
 *
 */
void collect_imu(int start_idx, int end_idx) {
    uint8_t addr;
    sensorInstance *sensor;
    // Iterate over each IMU
    for (int i = start_idx; i <= end_idx; i++) {
        sensor = &sensors[i];
        if (sensor->enabled) {
            // Get correct IMU (LSM6DSOX) I2C address.
            if (sensor->location == LEG_LEFT)
                addr = LSM6DSOX_ADDR_A;
            else if (sensor->location == LEG_RIGHT)
                addr = LSM6DSOX_ADDR_B;
            // Retrieve acceleration or gyroscope vector and store.
            if (sensor->datatype == ACCELERATION)
                imu_get_accel(addr, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else if (sensor->datatype == GYROSCOPE)
                imu_get_gyro(addr, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            sensor->newdata = true;
        }
    }    
}

/*
 *
 */
void calibrate() {
    // Orientation Board #1 calibration.
    uint8_t system, gyro, accel, mag = 0;
    sensorInstance ob = sensors[0];
    if (ob.enabled) {
        ob_head.getCalibration(&system, &gyro, &accel, &mag);
        ob.data[0] = (double)system;
        ob.data[1] = (double)gyro;
        ob.data[2] = (double)accel;
        ob.data[3] = (double)mag;
        print_datum(&ob);
        if ((system & gyro & accel & mag) == 3) {
            calibrated = true;
            mode = SETUP;
            led_blue = 1;
            queue.cancel(&e_calibrate);
            post_events();
        }
    }
    // Orientation Board #2.
    ob = sensors[1];
    if (ob.enabled) {
        ob_body.getCalibration(&system, &gyro, &accel, &mag);
        ob.data[0] = (double)system;
        ob.data[1] = (double)gyro;
        ob.data[2] = (double)accel;
        ob.data[3] = (double)mag;
        print_datum(&ob);
        if ((system & gyro & accel & mag) == 3) {
            calibrated = true;
            mode = SETUP;
            led_blue = 1;
            queue.cancel(&e_calibrate);
            post_events();
        }
    }
}

/* 
 * Send event.
 */
void send() {
    for (int i = 0; i < 17; i++) {
        if (sensors[i].enabled && sensors[i].newdata) {
            print_datum(&sensors[i]);
            sensors[i].newdata = false;
        }
    }
}

/*
 * 
 */
void post_events() {
    sensorInstance *sensor;
    switch (mode) {
        case SETUP:
            // Initialize sensors.
            // OB (Head)
            ob_init(&ob_head);
            sensor = &sensors[0];
            sensor->enabled = 1;
            sensor = &sensors[1];
            sensor->enabled = 1;
            // OB (Body)
            ob_init(&ob_body);
            sensor = &sensors[2];
            sensor->enabled = 0;
            sensor = &sensors[3];
            sensor->enabled = 0;
            // Scale
            sensor = &sensors[4];            
            sensor->enabled = 1;
            // FSR
            for (int i = 5; i <= 12; i++) {
                sensor = &sensors[i];            
                sensor->enabled = 1;
            }
            // IMU (Left)
            imu_init(LSM6DSOX_ADDR_A);
            sensor = &sensors[13];
            sensor->enabled = 1;
            sensor = &sensors[14];
            sensor->enabled = 1;
            // IMU (Right)
            imu_init(LSM6DSOX_ADDR_B);
            sensor = &sensors[15];
            sensor->enabled = 0;
            sensor = &sensors[16];
            sensor->enabled = 0;
            // Event periods.
            e_collect_ob.period(OB_PERIOD_MS);
            e_collect_scale.period(SCALE_PERIOD_MS);
            e_collect_fsr.period(FSR_PERIOD_MS);
            e_collect_imu.period(IMU_PERIOD_MS);
            e_send.period(SEND_PERIOD_MS);
            mode = RUN;
        case RUN:
            // Start events.
            e_collect_ob.try_call_on(&queue);
            e_collect_scale.try_call_on(&queue);
            e_collect_fsr.try_call_on(&queue);
            e_collect_imu.try_call_on(&queue);
            e_send.try_call_on(&queue);
            break;
        case CALIBRATE:
            // Start calibration routine event.
            led_blue = 0;
            e_calibrate.period(200);
            e_calibrate.try_call_on(&queue);
            break;
        default:
            // IDLE mode. Suspend all events.
            queue.cancel(&e_collect_ob);
            queue.cancel(&e_collect_scale);
            queue.cancel(&e_collect_fsr);
            queue.cancel(&e_collect_imu);
            queue.cancel(&e_send);
            queue.cancel(&e_calibrate);
            break;
    }
}

void print_startstop() {
    if (logging) {
        printf("STOP\n");
    } else {
        printf("START\n");
    }
    logging = !logging;
}

void startstop_handler() {
    led_red = !logging;
    led_green = logging;
    e_startstop.try_call_on(&queue);
}

/*
 * Main program entry point.
 */
int main() {
    /* Initialize modules */
    // Configure I2C.
    i2c.frequency(400000);
    // Button interrupt.
    btn.rise(startstop_handler);
    // LEDs
    led_red = 1;
    led_green = 1;
    led_blue = 1;
    // Event queue thread
    Thread event_thread;
    event_thread.start(callback(post_events));
    queue.dispatch_forever();
    event_thread.join();

    /* Infinite loop */
    while (true) {
    }
}

/*
 *
 */
void print_datum(sensorInstance *sensor) {
    char buff[256];
    switch (sensor->datanum) {
        case 1:
            printf("%s>%s>%s>%f\n", STYPE_STR(sensor->type), SLOC_STR(sensor->location), SDTYPE_STR(sensor->datatype), sensor->data[0]);
            break;
        case 2:
            printf("%s>%s>%s>%f>%f\n", STYPE_STR(sensor->type), SLOC_STR(sensor->location), SDTYPE_STR(sensor->datatype), sensor->data[0], sensor->data[1]);
            break;
        case 3:
            printf("%s>%s>%s>%f>%f>%f\n", STYPE_STR(sensor->type), SLOC_STR(sensor->location), SDTYPE_STR(sensor->datatype), sensor->data[0], sensor->data[1], sensor->data[2]);
            break;
        case 4:
            printf("%s>%s>%s>%f>%f>%f>%f\n", STYPE_STR(sensor->type), SLOC_STR(sensor->location), SDTYPE_STR(sensor->datatype), sensor->data[0], sensor->data[1], sensor->data[2], sensor->data[3]);
            break;
    }
}


/*
 * I2C Read helper function.   
 */
void readReg(int address, uint8_t subaddress, char *data, int length = 1) {
    // Set register to read
    char subaddr[1] = {subaddress};
    i2c.write(address, subaddr, 1);
    // Read register
    i2c.read(address, data, length);
}

/*
 * I2C Write helper function.    
 */
void writeReg(int address, uint8_t subaddress, uint8_t command) {
    // Write register
    char wdata[2] = {subaddress, command};
    i2c.write(address, wdata, 2);
}

/*
 * I2C Set bits function.    
 */
void setBits(int address, uint8_t subaddress, uint8_t mask) {
    char data[1];
    readReg(address, subaddress, data, 1);
    data[0] |= mask;
    writeReg(address, subaddress, data[0]);
}

/*
 * I2C Clear bits function.    
 */
void clearBits(int address, uint8_t subaddress, uint8_t mask) {
    char data[1];
    readReg(address, subaddress, data, 1);
    data[0] &= ~mask;
    writeReg(address, subaddress, data[0]);
}