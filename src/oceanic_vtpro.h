/* 
 * libdivecomputer
 * 
 * Copyright (C) 2008 Jef Driesen
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#ifndef OCEANIC_VTPRO_H
#define OCEANIC_VTPRO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "device.h"

#define OCEANIC_VTPRO_MEMORY_SIZE 0x8000
#define OCEANIC_VTPRO_PACKET_SIZE 0x10

device_status_t
oceanic_vtpro_device_open (device_t **device, const char* name);

device_status_t
oceanic_vtpro_device_keepalive (device_t *device);

device_status_t
oceanic_vtpro_device_calibrate (device_t *abstract, unsigned char data[], unsigned int size);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* OCEANIC_VTPRO_H */