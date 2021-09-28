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
Adafruit_BNO055 ob_head = Adafruit_BNO055(OB_HEAD, BNO055_ADDRESS_A, &i2c);
Adafruit_BNO055 ob_body = Adafruit_BNO055(OB_BODY, BNO055_ADDRESS_B, &i2c);
AnalogIn fsr_in(A0);

// Create static event queue
static EventQueue queue(0);
// Queue events
auto e_obh = make_user_allocated_event(ob_collect, OB_HEAD);
auto e_obb = make_user_allocated_event(ob_collect, OB_BODY);
auto e_scal = make_user_allocated_event(scale_collect);
auto e_fsr = make_user_allocated_event(fsr_collect);
auto e_imul = make_user_allocated_event(imu_collect, IMU_LEG_L);
auto e_imur = make_user_allocated_event(imu_collect, IMU_LEG_R);
auto e_cal = make_user_allocated_event(calibrate);
auto e_send = make_user_allocated_event(send_data);
// UserAllocatedEvent sensor_events[6] = {&e_obh, &e_obb, &e_scal, &e_fsr, &e_imul, &e_imur};

// Global variables
static sensorSample_t obh_acc, obh_eul, obb_acc, obb_eul, scal_frc, fsr_plr, fsr_prr, fsr_plh, fsr_prh,
                        fsr_slf, fsr_srf, fsr_slk, fsr_srk, imul_acc, imul_gyr, imur_acc, imur_gyr;
static systemMode_t mode = SETUP;

/*
 *
 */
void ob_collect(int id) {
    Adafruit_BNO055 *ob;
    sensorSample_t *acc_sample, *eul_sample;
    if (id == OB_HEAD) {
        ob = &ob_head;
        acc_sample = &obh_acc;
        eul_sample = &obh_eul;
    } else {
        ob = &ob_body;
        acc_sample = &obb_acc;
        eul_sample = &obb_eul;
    }
    if (acc_sample->enabled)
        ob_get_accel(ob, &(acc_sample->data[0]), &(acc_sample->data[1]), &(acc_sample->data[2]));
    if (eul_sample->enabled)
        ob_get_euler(ob, &(eul_sample->data[0]), &(eul_sample->data[1]), &(eul_sample->data[2]));
}

/*
 *
 */
void scale_collect() {
    if (scal_frc.enabled)
        scal_frc.data[0] = 0;
}

/*
 *
 */
void fsr_collect() {
    float frc;
    if (fsr_plr.enabled) {
        frc = fsr_in.read();
        fsr_plr.data[0] = frc;
    }
    if (fsr_prr.enabled) {
        frc = fsr_in.read();
        fsr_prr.data[0] = frc;
    }
    if (fsr_plh.enabled) {
        frc = fsr_in.read();
        fsr_plh.data[0] = frc;
    }
    if (fsr_prh.enabled) {
        frc = fsr_in.read();
        fsr_prh.data[0] = frc;
    }
    if (fsr_slf.enabled) {
        frc = fsr_in.read();
        fsr_slf.data[0] = frc;
    }
    if (fsr_srf.enabled) {
        frc = fsr_in.read();
        fsr_srf.data[0] = frc;
    }
    if (fsr_slk.enabled) {
        frc = fsr_in.read();
        fsr_slk.data[0] = frc;
    }
    if (fsr_srk.enabled) {
        frc = fsr_in.read();
        fsr_srk.data[0] = frc;
    }
}

/*
 *
 */
void imu_collect(int id) {
    uint8_t addr;
    sensorSample_t *acc_sample, *gyr_sample;
    if (id == IMU_LEG_L) {
        addr = IMU_LEG_L_ADDR;
        acc_sample = &imul_acc;
        gyr_sample = &imul_gyr;
    } else {
        addr = IMU_LEG_R_ADDR;
        acc_sample = &imur_acc;
        gyr_sample = &imur_gyr;
    }
    if (acc_sample->enabled)
        imu_get_accel(addr, &(acc_sample->data[0]), &(acc_sample->data[1]), &(acc_sample->data[2]));
    if (gyr_sample->enabled)
        imu_get_gyro(addr, &(gyr_sample->data[0]), &(gyr_sample->data[1]), &(gyr_sample->data[2]));
}

/*
 *
 */
void calibrate() {
    uint8_t system, gyro, accel, mag = 0;
    ob_head.getCalibration(&system, &gyro, &accel, &mag);
    printf("OBH>CAL>%d>%d>%d>%d\n", system, gyro, accel, mag);
    ob_body.getCalibration(&system, &gyro, &accel, &mag);
    printf("OBB>CAL>%d>%d>%d>%d\n", system, gyro, accel, mag);
}

/* 
 * Send event.
 */
void send_data() {
    send_datum(&obh_acc);
    send_datum(&obh_eul);
    send_datum(&obb_acc);
    send_datum(&obh_acc);
    send_datum(&obb_eul);
    send_datum(&scal_frc);
    send_datum(&fsr_plr);
    send_datum(&fsr_prr);
    send_datum(&fsr_plh);
    send_datum(&fsr_prh);
    send_datum(&fsr_slf);
    send_datum(&fsr_srf);
    send_datum(&fsr_slk);
    send_datum(&fsr_srk);
    send_datum(&imul_acc);
    send_datum(&imul_gyr);
    send_datum(&imur_acc);
    send_datum(&imur_gyr);
}

/*
 * 
 */
void post_events() {
    switch (mode) {
        case SETUP:
            // Initialize sensors and start events.
            ob_init(&ob_head);
            obh_acc.enabled = true;
            obh_eul.enabled = true;
            e_obh.period(10);
            // ob_init(&ob_body);
            // scale_init();
            // fsr_init();
            imu_init(IMU_LEG_L_ADDR);
            imul_acc.enabled = true;
            imul_gyr.enabled = true;
            e_imul.period(10);
            e_send.period(10);
            // imu_init(IMU_LEG_R_ADDR);
            // obh_acc.enable = 1;
            e_obh.try_call_on(&queue);
            // e_obb.try_call_on(&queue);
            // e_scal.try_call_on(&queue);
            // e_fsr.try_call_on(&queue);
            e_imul.try_call_on(&queue);
            // e_imur.try_call_on(&queue);
            e_send.try_call_on(&queue);
            break;
        case RUN:
            // Start events.
            e_obh.try_call_on(&queue);
            // e_obb.try_call_on(&queue);
            // e_scal.try_call_on(&queue);
            // e_fsr.try_call_on(&queue);
            e_imul.try_call_on(&queue);
            // e_imur.try_call_on(&queue);
            e_send.try_call_on(&queue);
            break;
        case CALIBRATE:
            // Start calibration routine event.
            e_cal.try_call_on(&queue);
            break;
        default:
            // IDLE mode. Suspend all events.
            queue.cancel(&e_obh);
            queue.cancel(&e_obb);
            queue.cancel(&e_scal);
            queue.cancel(&e_fsr);
            queue.cancel(&e_imul);
            queue.cancel(&e_imur);
            queue.cancel(&e_send);
            queue.cancel(&e_cal);
            break;
    }
}

/*
 * Main program entry point.
 */
int main() {
    /* Initialize modules */
    // Configure I2C.
    i2c.frequency(400000);
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
void send_datum(sensorSample_t *sample) {
    if (sample->enabled) {
        if (sample->datanum == 1) {
            printf("%s>%s>%f\n", sample->shortname, sample->datatype, sample->data[0]);
        } else if (sample->datanum == 2) {
            printf("%s>%s>%f>%f\n", sample->shortname, sample->datatype, sample->data[0], sample->data[1]);
        } else {
            printf("%s>%s>%f>%f>%f\n", sample->shortname, sample->datatype, sample->data[0], sample->data[1], sample->data[2]);
        }
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