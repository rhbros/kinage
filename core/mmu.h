#ifndef _MMU_H_
#define _MMU_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define BANK_CAN_BYTE 1
#define BANK_CAN_WORD 2
#define BANK_CAN_HWORD 4

typedef uint8_t (*mmu_read_hook_byte)(uint32_t address);
typedef uint16_t (*mmu_read_hook_hword)(uint32_t address);
typedef uint32_t (*mmu_read_hook_word)(uint32_t address);
typedef void (*mmu_write_hook_byte)(uint32_t address, uint8_t value);
typedef void (*mmu_write_hook_hword)(uint32_t address, uint16_t value);
typedef void (*mmu_write_hook_word)(uint32_t address, uint32_t value);

typedef struct mmu_bank
{
        uint8_t* ptr; 				// where the bank lays in our real memory
        uint8_t read_width_attr;  	// contains information about supported read sizes (8, 16, 32)
        uint8_t write_width_attr; 	// contains information about supported write sizes (8, 16, 32)
        uint8_t cycles_byte; 		// how many cycles it takes to access one byte
        uint8_t cycles_hword; 		// how many cycles it takes to access one hword
        uint8_t cycles_word; 		// how many cycles it takes to access one word
        mmu_read_hook_byte read_hook_byte;		// optional function to be called when reading byte (original method will no longer write data itself)
        mmu_read_hook_hword read_hook_hword;	// optional function to be called when reading hword (original method will no longer write data itself)
        mmu_read_hook_word read_hook_word;		// optional function to be called when reading word (original method will no longer write data itself)
        mmu_write_hook_byte write_hook_byte;	// optional function to be called when writing byte (original method will no longer write data itself)
        mmu_write_hook_hword write_hook_hword;	// optional function to be called when writing hword (original method will no longer write data itself)
        mmu_write_hook_word write_hook_word;	// optional function to be called when writing word (original method will no longer write data itself)
} mmu_bank;

void mmu_setup(char* rom);
uint8_t mmu_readbyte(uint32_t address);
uint16_t mmu_readhword(uint32_t address);
uint32_t mmu_readword(uint32_t address);
void mmu_writebyte(uint32_t address, uint8_t value);
void mmu_writehword(uint32_t address, uint16_t value);
void mmu_writeword(uint32_t address, uint32_t value);

#endif // _MMU_H_
