#ifndef __BUSINESS_H
#define __BUSINESS_H

#include "stm32f10x.h"
extern unsigned char inv;

extern short halt_limit;
extern unsigned char halt_change;

extern short temperature;  
extern short last_temperature;

#define MAX_SELECT_MODE	4

void task_temperature(void);

void task_keyscan(void);
void check_remote_key(void);

void display_update(void);

void task_uart_mngr(void);

void set_bund(int bund);

void task_alarm(void);
void handle_beep(void);

void task_report(void);

void task_clock_show(void);

#define MQTT_CLIENT_ID "client002"
#define MQTT_USERNAME "client0002"
#define MQTT_PASSWORD "sky231439"

#define MQTT_SUBTOPIC "mcu_report_0001"
#define MQTT_PUBTOPIC "mcu_control_0001"

int broker_init(int32_t uart_index);
void broker_loop(void);

void update_param(u16 id,u32 value);
void can_task(void);
void update_param(u16 id,u32 value);

#endif
