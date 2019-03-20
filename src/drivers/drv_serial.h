
#ifndef _DRV_SERIAL_H
#define _DRV_SERIAL_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"


#define SERIAL_BASE_DEVICE_PATH		"/dev/serial_com"
#define SERIAL_DEVICE_PATH			"/dev/serial_com0"


//#define RANGE_FINDER_BASE_DEVICE_PATH	"/dev/range_finder"
//#define RANGE_FINDER0_DEVICE_PATH	"/dev/range_finder0"

//"/dev/ttyS6"
/*
 * ioctl() definitions
 *
 * Rangefinder drivers also implement the generic sensor driver
 * interfaces from drv_sensor.h
 */

//#define _SERIALIOCBASE			(0x7800)
//#define __SERIALIOC(_n)		(_IOC(_SERIALIOCBASE, _n))


#endif /* _DRV_SERIAL_H */
