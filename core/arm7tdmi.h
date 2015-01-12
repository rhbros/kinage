#ifndef _ARM7TDMI_H_
#define _ARM7TDMI_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "mmu.h"

void arm7_reset();
void arm7_update_gprs();
void arm7_step();

uint8_t arm7_readb(uint32_t ptr);
uint16_t arm7_readh(uint32_t ptr);
uint32_t arm7_read(uint32_t ptr);
void arm7_writeb(uint32_t ptr, uint8_t value);
void arm7_writeh(uint32_t ptr, uint16_t value);
void arm7_write(uint32_t ptr, uint32_t value);

#endif // _ARM7TDMI_H_
