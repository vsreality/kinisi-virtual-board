#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void sim_transport_init(void);
void sim_transport_expire(void);
void sim_transport_step(void);
int sim_usb_feed(const uint8_t *data, size_t length);
bool sim_usb_send(uint8_t *data, uint8_t length);
void sim_usb_disconnect(void);
int sim_usb_lost(void);
int sim_usb_ready(void);
