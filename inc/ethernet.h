/*
 * File    : ethernet.h
 * Author  : Chris Abajian (cxa6282@rit.edu)
 * Created : November 9, 2021
 *
 * Description:
 *   Ethernet communication header.
 */

#pragma once

#ifndef __ETHERNET_INCLUDE_H__
#define __ETHERNET_INCLUDE_H__

/* Ethernet Function Prototypes */
void ethernet_init();
void ethernet_send(const char *data, const int size);

#endif
