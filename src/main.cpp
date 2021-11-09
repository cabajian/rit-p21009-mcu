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
#include "ethernet.h"
#include "i2c.h"
#include "orientation_board.h"
#include "imu.h"

// Instances
BufferedSerial serial(UART_TX, UART_RX, UART_BAUD);
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
auto e_send_data = make_user_allocated_event(send_data);

// Data buffer.
char data_buff[MAX_STR_LEN];
int buff_idx;
Mutex buff_mutex;

// Devices.
static DeviceInstance devices[] = {
    // Enabled  Device             Location       Function      Num Data  Data       Num Offsets  Offsets
    {false,     SYSTEM,            GLOBAL,        LOGGING,      0,        {0,0,0,0}, 0,           {0,0,0}},
    {false,     ORIENTATION_BOARD, HEAD,          ACCELERATION, 3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     ORIENTATION_BOARD, HEAD,          EULER,        3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     ORIENTATION_BOARD, BODY,          ACCELERATION, 3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     ORIENTATION_BOARD, BODY,          EULER,        3,        {0,0,0,0}, 3,           {0,0,0}},
    {false,     SCALE,             CHEST,         FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               RIB_LEFT,      FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               RIB_RIGHT,     FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               FOREARM_LEFT,  FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}}, 
    {false,     FSR,               FOREARM_RIGHT, FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               HIP_LEFT,      FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               HIP_RIGHT,     FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               KNEE_LEFT,     FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     FSR,               KNEE_RIGHT,    FORCE,        1,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_LEFT,      ACCELERATION, 3,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_LEFT,      GYROSCOPE,    3,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_RIGHT,     ACCELERATION, 3,        {0,0,0,0}, 1,           {0,0,0}},
    {false,     IMU,               LEG_RIGHT,     GYROSCOPE,    3,        {0,0,0,0}, 1,           {0,0,0}}
};

/**
 * Collects data from all enabled Orientation Boards and
 * appends the encoded data to data_buff.
 */
void collect_ob() {
    DeviceInstance *sensor;
    // Iterate over each Orientation Board
    for (int i = OB_FIRST_IDX; i <= OB_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Get pointer to the correct BN055 instance.
            int instance;
            if (sensor->loc == HEAD)
                instance = OB_HEAD;
            else if (sensor->loc == BODY)
                instance = OB_BODY;
            else
                continue;
            // Retrieve acceleration or euler vector and store.
            if (sensor->func == ACCELERATION)
                OB_get_accel(instance, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else if (sensor->func == EULER)
                OB_get_euler(instance, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else
                continue;
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f%s%f%s%f\n", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), sensor->data[0], DELIM.c_str(), sensor->data[1], DELIM.c_str(), sensor->data[2]);
            buff_mutex.unlock();
        }
    }
}

/**
 * Collects data from all enabled scales and
 * appends the encoded data to data_buff.
 */
void collect_scale() {
    DeviceInstance *sensor;
    // Iterate over each Scale
    for (int i = SCALE_FIRST_IDX; i <= SCALE_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Store the data.
            sensor->data[0] = 0; // TODO: scale needs to be implemented.
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f\n", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), sensor->data[0]);
            buff_mutex.unlock();
        }
    }
}

/*
 * Collects data from all enabled FSR's and
 * appends the encoded data to data_buff.
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
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f\n", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), sensor->data[0]);
            buff_mutex.unlock();
        }
    }
}

/*
 * Collects data from all enabled IMU's and
 * appends the encoded data to data_buff.
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
            else
                continue;
            // Retrieve acceleration or gyroscope vector and store.
            if (sensor->func == ACCELERATION)
                IMU_get_accel(addr, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else if (sensor->func == GYROSCOPE)
                IMU_get_gyro(addr, &(sensor->data[0]), &(sensor->data[1]), &(sensor->data[2]));
            else
                continue;
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f%s%f%s%f\n", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), sensor->data[0], DELIM.c_str(), sensor->data[1], DELIM.c_str(), sensor->data[2]);
            buff_mutex.unlock();
        }
    }    
}

/**
 * Sends the sensor data contained in data_buff to be 
 * transmitted via Ethernet if system logging is enabled.
 */
void send_data() {
    if (devices[0].enabled) {
        data_buff[buff_idx] = '\0';
        ethernet_send(data_buff, buff_idx);
        buff_idx = 0;
    }
}

/*
 * 
 */
void post_events() {
    DeviceInstance *sensor;
    // Initialize devices.
    // System Logging
    devices[0].enabled = true;
    // OB (Head+Body)
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
    for (int i = IMU_FIRST_IDX; i <= IMU_LAST_IDX; i++) {
        sensor = &devices[i];            
        sensor->enabled = 1;
    }
    // Event periods.
    e_collect_ob.period(OB_PERIOD_MS);
    e_collect_scale.period(SCALE_PERIOD_MS);
    e_collect_fsr.period(FSR_PERIOD_MS);
    e_collect_imu.period(IMU_PERIOD_MS);
    // e_poll_cmd.period(POLL_CMD_PERIOD_MS);
    e_send_data.period(SEND_PERIOD_MS);
    // Start events.
    e_collect_ob.try_call_on(&queue);
    e_collect_scale.try_call_on(&queue);
    e_collect_fsr.try_call_on(&queue);
    e_collect_imu.try_call_on(&queue);
    // e_poll_cmd.try_call_on(&queue);
    e_send_data.try_call_on(&queue);
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
    queue.cancel(&e_send_data);
}

/*
 * Main program entry point.
 */
int main() {
    /* Initialize modules */
    // Initialize communication modules.
    I2C_init();
    ethernet_init();
    // Initialize sensors.
    OB_init(OB_HEAD);
    OB_init(OB_BODY);
    IMU_init(LSM6DSOX_ADDR_A);
    IMU_init(LSM6DSOX_ADDR_B);
    // Event queue thread
    Thread event_thread;
    event_thread.start(callback(post_events));
    queue.dispatch_forever();
    event_thread.join();

    /* Infinite loop */
    while (true) {}
}
