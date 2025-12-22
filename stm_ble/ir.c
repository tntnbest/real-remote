#include "ir.h"
#include "main.h"   
#include <stdio.h>

extern TIM_HandleTypeDef htim2;

const uint16_t LG_Monitor_IR[] = {
	9077, 4662, 482, 713, 502, 655, 493, 1822, 472, 691, 477, 681, 513, 650, 482, 675, 493, 681, 
	502, 1796, 508, 1829, 503, 681, 492, 1822, 487, 1822, 472, 1828, 471, 1829, 518, 1797, 445, 
	712, 466, 692, 467, 701, 481, 1818, 450, 743, 451, 655, 524, 676, 497, 738, 424, 1828, 481, 
	1828, 455, 1849, 493, 681, 502, 1818, 503, 1796, 481, 1834, 492, 1802, 477
};

const uint16_t LG_AC_ON_IR[] = {
    8624,4066,484,1578,462,546,454,560,452,558,454,1570,454,556,456,556,
    454,556,454,558,454,558,454,556,454,558,454,556,454,556,454,556,
    454,558,454,1546,476,558,452,1544,480,1546,474,560,450,1544,502,534,
    452,558,452,1548,478,1542,478,1544,480,1546,478
};

const uint16_t LG_AC_OFF_IR[] = {
    8648,4036,500,1536,508,498,476,536,452,562,450,1572,478,532,474,536,
    464,550,478,1546,476,1546,488,522,454,558,450,560,454,558,480,530,
    452,558,474,536,454,564,476,530,474,536,476,534,476,1550,508,500,
    452,1576,502,502,452,558,452,558,452,1572,478
};

const uint8_t LG_Monitor_IR_LEN = sizeof(LG_Monitor_IR) / sizeof(LG_Monitor_IR[0]);
const uint8_t LG_AC_ON_IR_LEN = sizeof(LG_AC_ON_IR) / sizeof(LG_AC_ON_IR[0]);
const uint8_t LG_AC_OFF_IR_LEN = sizeof(LG_AC_OFF_IR) / sizeof(LG_AC_OFF_IR[0]);

volatile uint8_t  receiving = 0;
volatile uint8_t  ir_key_ready = 0;
volatile uint16_t ir_times[MAX_NUM];
volatile uint8_t  ir_count = 0;
volatile uint32_t last_frame_tick = 0;

void ir_reset(void) {
    receiving = 0;
    ir_key_ready = 0;
    ir_count = 0;
}

void ir_print(void) {
    printf("Received %d:\r\n", ir_count);
    for (uint8_t i = 0; i < ir_count; i++) {
        printf("%u ", ir_times[i]);
    }
    printf("\r\n");
    ir_reset();
}

void ir_check(uint16_t period) {
    if (!receiving) {
        if (period > START_FLAG && period < END_FLAG) {
            receiving = 1;
            ir_count  = 0;
            ir_times[ir_count++] = period;
        }
    } else {
        if (period > END_FLAG) {
            ir_key_ready = 1;
            receiving = 0;
            last_frame_tick = HAL_GetTick();
        } else if (ir_count < MAX_NUM) {
            ir_times[ir_count++] = period;
        }
    }
}

void ir_exti_callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin != GPIO_PIN_5) return;

    uint32_t now = HAL_GetTick();
    if (!receiving && (now - last_frame_tick) < FRAME_GAP) {
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        return;
    }

    uint32_t delta = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    ir_check(delta);
}