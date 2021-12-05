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
#include "scale.h"
#include "fsr.h"

// Devices.
static DeviceInstance devices[] = {
    // Enabled  Device             Location       Function      Offsets
    {false,     SYSTEM,            GLOBAL,        ENABLE,       {0,0,0}},
    {false,     ORIENTATION_BOARD, HEAD,          ACCELERATION, {0,0,0}},
    {false,     ORIENTATION_BOARD, HEAD,          EULER,        {0,0,0}},
    {false,     ORIENTATION_BOARD, BODY,          ACCELERATION, {0,0,0}},
    {false,     ORIENTATION_BOARD, BODY,          EULER,        {0,0,0}},
    {false,     SCALE,             CHEST,         FORCE,        {0,0,0}},
    {false,     FSR,               RIB_LEFT,      FORCE,        {0,0,0}},
    {false,     FSR,               RIB_RIGHT,     FORCE,        {0,0,0}},
    {false,     FSR,               FOREARM_LEFT,  FORCE,        {0,0,0}}, 
    {false,     FSR,               FOREARM_RIGHT, FORCE,        {0,0,0}},
    {false,     FSR,               HIP_LEFT,      FORCE,        {0,0,0}},
    {false,     FSR,               HIP_RIGHT,     FORCE,        {0,0,0}},
    {false,     FSR,               KNEE_LEFT,     FORCE,        {0,0,0}},
    {false,     FSR,               KNEE_RIGHT,    FORCE,        {0,0,0}},
    {false,     IMU,               LEG_LEFT,      ACCELERATION, {0,0,0}},
    {false,     IMU,               LEG_LEFT,      GYROSCOPE,    {0,0,0}},
    {false,     IMU,               LEG_RIGHT,     ACCELERATION, {0,0,0}},
    {false,     IMU,               LEG_RIGHT,     GYROSCOPE,    {0,0,0}}
};
// Instances
BufferedSerial serial(UART_TX, UART_RX, UART_BAUD);

// Data buffer.
char data_buff[MAX_STR_LEN];
int buff_idx;
Mutex buff_mutex;
// Create static event queue
static EventQueue queue(0);
// Queue events
auto e_collect_ob = make_user_allocated_event(collect_ob);
auto e_collect_scale = make_user_allocated_event(collect_scale);
auto e_collect_fsr = make_user_allocated_event(collect_fsr);
auto e_collect_imu = make_user_allocated_event(collect_imu);
auto e_poll_cmd = make_user_allocated_event(poll_cmd, &serial);
auto e_send_data = make_user_allocated_event(send_data, &(devices[0].enabled), data_buff, &buff_idx, &serial);


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
            // Collect the sample.
            double data[3];
            if (!OB_collect(sensor, data)) {
                continue;
            }
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f%s%f%s%f\n\r", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), data[0], DELIM.c_str(), data[1], DELIM.c_str(), data[2]);
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
            double data = scale_read();
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f\n\r", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), data);
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
            double data = FSR_collect(sensor);
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f\n\r", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), data);
            buff_mutex.unlock();
        }
    }
}

/*
 * Collects data from all enabled IMU's and
 * appends the encoded data to data_buff.
 */
void collect_imu() {
    DeviceInstance *sensor;
    // Iterate over each IMU
    for (int i = IMU_FIRST_IDX; i <= IMU_LAST_IDX; i++) {
        sensor = &devices[i];
        if (sensor->enabled) {
            // Collect the sample.
            double data[3];
            if (!IMU_collect(sensor, data)) {
                continue;
            }
            // Add the data to the buffer.
            buff_mutex.lock();
            buff_idx += snprintf(data_buff+buff_idx, MAX_STR_LEN-buff_idx, "%s%s%s%s%s%s%f%s%f%s%f\n\r", to_char(sensor->dev), DELIM.c_str(), to_char(sensor->loc), DELIM.c_str(), to_char(sensor->func), DELIM.c_str(), data[0], DELIM.c_str(), data[1], DELIM.c_str(), data[2]);
            buff_mutex.unlock();
        }
    }    
}

/*
 * Post (submit) all events in the queue.
 */
void post_events() {
    DeviceInstance *sensor;
    // Initialize devices.
    // System Logging
    devices[0].enabled = true;
    // OB (Head+Body)
    for (int i = OB_FIRST_IDX; i <= OB_LAST_IDX; i++) {
        sensor = &devices[i];
        sensor->enabled = false;
    }
    // Scale
    for (int i = SCALE_FIRST_IDX; i <= SCALE_LAST_IDX; i++) {
        sensor = &devices[i];
        sensor->enabled = false;
    }
    // FSR
    for (int i = FSR_FIRST_IDX; i <= FSR_LAST_IDX; i++) {
        sensor = &devices[i];            
        sensor->enabled = false;
    }
    // IMU (Left+Right)
    for (int i = IMU_FIRST_IDX; i <= IMU_LAST_IDX; i++) {
        sensor = &devices[i];            
        sensor->enabled = false;
    }
    // Event periods.
    e_collect_ob.period(OB_PERIOD_MS);
    e_collect_scale.period(SCALE_PERIOD_MS);
    e_collect_fsr.period(FSR_PERIOD_MS);
    e_collect_imu.period(IMU_PERIOD_MS);
    e_poll_cmd.period(POLL_CMD_PERIOD_MS);
    e_send_data.period(SEND_PERIOD_MS);
    // Start events.
    e_collect_ob.try_call_on(&queue);
    e_collect_scale.try_call_on(&queue);
    e_collect_fsr.try_call_on(&queue);
    e_collect_imu.try_call_on(&queue);
    e_poll_cmd.try_call_on(&queue);
    e_send_data.try_call_on(&queue);
}

/*
 * Suspend all events in the event queue.
 */
void suspend_events() {
    // Suspend all events.
    queue.cancel(&e_collect_ob);
    queue.cancel(&e_collect_scale);
    queue.cancel(&e_collect_fsr);
    queue.cancel(&e_collect_imu);
    queue.cancel(&e_send_data);
}

DeviceInstance* get_device_instance(int index) {
    return &devices[index];
}

DeviceInstance* get_device_instance(Device dev, Location loc, Function func, int* index) {
    for (int i = 0; i < NUM_DEVICES; i++) {
        if (func == NO_FUNCTION) {
            if ((devices[i].dev == dev) && (devices[i].loc == loc)) {
                *index = i;
                return &devices[i];
            }
        } else {
            if ((devices[i].dev == dev) && (devices[i].loc == loc) && (devices[i].func == func)) {
                *index = i;
                return &devices[i];
            }
        }
    }
    return NULL;
}

/*
 * Main program entry point.
 */
int main() {
    /* Initialize modules */
    // Initialize communication modules.
    I2C_init();
    #ifndef DEBUG
        ethernet_init();
    #endif
    // Initialize sensors.
    OB_init(OB_HEAD);
    OB_init(OB_BODY);
    scale_init();
    IMU_init(LSM6DSOX_ADDR_A);
    IMU_init(LSM6DSOX_ADDR_B);
    serial.set_blocking(false);
    
    // Event queue thread
    Thread event_thread;
    event_thread.start(callback(post_events));
    queue.dispatch_forever();
    event_thread.join();

    /* Infinite loop */
    while (true) {}
}
