#pragma once
#include "main.h"  
#include <stdint.h>

#define END_FLAG   30000U
#define START_FLAG 5000U
#define MAX_NUM    100U
#define FRAME_GAP  200U   

extern const uint16_t LG_Monitor_IR[];
extern const uint16_t LG_AC_ON_IR[];
extern const uint16_t LG_AC_OFF_IR[];
extern const uint8_t LG_Monitor_IR_LEN;
extern const uint8_t LG_AC_ON_IR_LEN;
extern const uint8_t LG_AC_OFF_IR_LEN;

extern volatile uint8_t  receiving;
extern volatile uint8_t  ir_key_ready;
extern volatile uint16_t ir_times[MAX_NUM];
extern volatile uint8_t  ir_count;
extern volatile uint32_t last_frame_tick;

void ir_reset(void);
void ir_print(void);
void print_status(const char *msg);
void ir_check(uint16_t period);
void ir_exti_callback(uint16_t GPIO_Pin);