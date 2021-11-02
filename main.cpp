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
I2C i2c(I2C_SDA, I2C_SCL);
BufferedSerial serial(UART_TX, UART_RX, UART_BAUD);
// Console override. Required for non-USB printf stream.
FileHandle *mbed::mbed_override_console(int fd) { return &serial; }
Adafruit_BNO055 ob_head = Adafruit_BNO055(0, BNO055_ADDRESS_A, &i2c);
Adafruit_BNO055 ob_body = Adafruit_BNO055(1, BNO055_ADDRESS_B, &i2c);
AnalogIn pcb_in(PCB_ANALOG_IN);
DigitalOut pcb_s1(PCB_S1_OUT);
DigitalOut pcb_s0(PCB_S0_OUT);
DigitalOut led_red(LED_RED);

// Create static event queue
static EventQueue queue(0);
// Queue events
auto e_collect_ob = make_user_allocated_event(collect_ob);
auto e_collect_scale = make_user_allocated_event(collect_scale);
auto e_collect_fsr = make_user_allocated_event(collect_fsr);
auto e_collect_imu = make_user_allocated_event(collect_imu);
auto e_poll_cmd = make_user_allocated_event(poll_cmd, &serial);

/*
 *
 */
void collect_ob() {
    Adafruit_BNO055 *ob;
    DeviceInstance *sensor;
    // Iterate over each Orientation Board
    for (int i = OB_FIRST_IDX; i <= OB_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Get pointer to the correct BN055 instance.
            if (sensor->loc == HEAD)
                ob = &ob_head;
            else if (sensor->loc == BODY)
                ob = &ob_body;
            // Retrieve acceleration or euler vector and store.
            if (sensor->func == ACCELERATION)
                ob_get_accel(ob, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else if (sensor->func == EULER)
                ob_get_euler(ob, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            // Log the data.
            send_data(sensor, &devices[0]);
        }
    }
}

/*
 *
 */
void collect_scale() {
    DeviceInstance *sensor;
    // Iterate over each Scale
    for (int i = SCALE_FIRST_IDX; i <= SCALE_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Store the data.
            sensor->data[0] = 0; // TODO: scale needs to be implemented.
            // Log the data.
            send_data(sensor, &devices[0]);
        }
    }
}

/*
 *
 */
void collect_fsr() {
    DeviceInstance *sensor;
    // Iterate over each FSR
    for (int i = FSR_FIRST_IDX; i <= FSR_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Set mux selects going to the PCB.
            switch (sensor->loc) {
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
            float samples[ADC_SAMPLES];
            samples[0] = pcb_in.read();
            for (int i = 1; i < ADC_SAMPLES; i++) {
                samples[i] = (pcb_in.read() + samples[i-1])/2.0;
            }
            // Store the data.
            sensor->data[0] = samples[ADC_SAMPLES-1];
            // Log the data.
            send_data(sensor, &devices[0]);
        }
    }
}

/*
 *
 */
void collect_imu() {
    uint8_t addr;
    DeviceInstance *sensor;
    // Iterate over each IMU
    for (int i = IMU_FIRST_IDX; i <= IMU_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Get correct IMU (LSM6DSOX) I2C address.
            if (sensor->loc == LEG_LEFT)
                addr = LSM6DSOX_ADDR_A;
            else if (sensor->loc == LEG_RIGHT)
                addr = LSM6DSOX_ADDR_B;
            // Retrieve acceleration or gyroscope vector and store.
            if (sensor->func == ACCELERATION)
                imu_get_accel(addr, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else if (sensor->func == GYROSCOPE)
                imu_get_gyro(addr, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            // Log the data.
            send_data(sensor, &devices[0]);
        }
    }    
}

/*
 * 
 */
void post_events() {
    DeviceInstance *sensor;
    // Initialize devices.
    // System Logging
    devices[0].enabled = false;
    // OB (Head+Body)
    ob_init(&ob_head);
    ob_init(&ob_body);
    for (int i = OB_FIRST_IDX; i <= OB_LAST_IDX; i++) {
       sensor = &devices[i];
       sensor->enabled = 1;
    }
    // Scale
    for (int i = SCALE_FIRST_IDX; i <= SCALE_LAST_IDX; i++) {
       sensor = &devices[i];
       sensor->enabled = 1;
    }
    // FSR
    for (int i = FSR_FIRST_IDX; i <= FSR_LAST_IDX; i++) {
        sensor = &devices[i];            
        sensor->enabled = 1;
    }
    // IMU (Left+Right)
    imu_init(LSM6DSOX_ADDR_A);
    imu_init(LSM6DSOX_ADDR_B);
    for (int i = IMU_FIRST_IDX; i <= IMU_LAST_IDX; i++) {
        sensor = &devices[i];            
        sensor->enabled = 1;
    }
    // Event periods.
    e_collect_ob.period(OB_PERIOD_MS);
    e_collect_scale.period(SCALE_PERIOD_MS);
    e_collect_fsr.period(FSR_PERIOD_MS);
    e_collect_imu.period(IMU_PERIOD_MS);
    e_poll_cmd.period(POLL_CMD_PERIOD_MS);
    // Start events.
    e_collect_ob.try_call_on(&queue);
    e_collect_scale.try_call_on(&queue);
    e_collect_fsr.try_call_on(&queue);
    e_collect_imu.try_call_on(&queue);
    e_poll_cmd.try_call_on(&queue);
    // LED indicator.
    led_red = 0;
}

/*
 * 
 */
void suspend_events() {
    // Suspend all events.
    queue.cancel(&e_collect_ob);
    queue.cancel(&e_collect_scale);
    queue.cancel(&e_collect_fsr);
    queue.cancel(&e_collect_imu);
}

/*
 * Main program entry point.
 */
int main() {
    /* Initialize modules */
    // Configure I2C.
    i2c.frequency(400000);
    // Configure UART.
    serial.set_blocking(false);
    // LEDs
    led_red = 1;
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
