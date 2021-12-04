/*
 * File    : ethernet.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 9, 2021
 *
 * Description:
 *   Ethernet communication functions.
 */

#include "ethernet.h"
#include "mbed.h"
#include "EthernetInterface.h"
#include "LWIPStack.h"

// Static IP network variables
static const char* PI_IP   = "169.254.180.193";
static const char* MBED_IP = "192.168.0.38";
static const char* NETMASK = "255.255.0.0";
static const char* GATEWAY = "0.0.0.0";
static const int   PORT    = 37;

// Ethernet/socket objects.
static EthernetInterface eth;
static UDPSocket cs;
static SocketAddress ss;

/**
 * Initializes a UDP socket to the Raspberry Pi's IP address.
 */
void ethernet_init() {
    // Connect this client to the ethernet interface.
    eth.set_network(MBED_IP, NETMASK, GATEWAY);
    eth.connect();
    cs.open(&eth);
    cs.bind(PORT);
    cs.set_blocking(false); // Prevent blocking.
    // Set server IP and port.
    ss.set_ip_address(PI_IP);
    ss.set_port(PORT);
}

/* 
 * Send data array via Ethernet socket.
 */
void ethernet_send(const char *data, const int size) {
    cs.sendto(ss, data, size);
}
