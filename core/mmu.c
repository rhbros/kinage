#include "mmu.h"

mmu_bank banks[10];
int cycles;

// TEST
uint8_t wram[0x40000];
uint8_t iram[0x8000];
uint8_t pal[0x400];
uint8_t vram[0x18000];
uint8_t* rom;
uint8_t io[0x3FF];

void mmu_setup(char* rom)
{
	FILE* f = fopen(rom, "rb");
    int size;
    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    rom = malloc(size);
    fread(rom, 1, size, f);
    fclose(f);
    
	banks[2].ptr = wram;
	banks[2].read_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	banks[2].write_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	
	banks[3].ptr = iram;
	banks[3].read_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	banks[3].write_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	
	banks[4].ptr = io;
	banks[4].read_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	banks[4].write_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	
	banks[5].ptr = pal;
	banks[5].read_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	banks[5].write_width_attr = BANK_CAN_HWORD | BANK_CAN_WORD;
	
	banks[6].ptr = vram;
	banks[6].read_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	banks[6].write_width_attr = BANK_CAN_HWORD | BANK_CAN_WORD;
	
	banks[8].ptr = rom;
	banks[8].read_width_attr = BANK_CAN_BYTE | BANK_CAN_HWORD | BANK_CAN_WORD;
	banks[8].write_width_attr = 0;
}

/* Maybe check for functions not going recursive in some cases? */

uint8_t mmu_readbyte(uint32_t address)
{
	mmu_bank bank = banks[address >> 24]; 		// get the bank to use
	uint32_t real_addr = address & 0x00FFFFFF; 	// get address relative to bank start
	if (bank.read_hook_byte == NULL) { 			// is there no hook?
		return bank.ptr[real_addr];
	} else { 									// there is a hook..
		return bank.read_hook_byte(real_addr);
	}
	cycles += bank.cycles_byte; 				// increase mmu cycles counter
}

uint16_t mmu_readhword(uint32_t address)
{
	mmu_bank bank = banks[address >> 24]; 					// get the bank to use
	if (bank.read_width_attr & BANK_CAN_HWORD) { 			// are we able to read a hword from this bank?
		uint32_t real_addr = (address & 0x00FFFFFF) & ~(1); // get (aligned) address relative to bank start
		if (bank.read_hook_hword == NULL) { 				// is there no hook?
			return *(uint16_t*)(bank.ptr + real_addr);
		} else { 											// there is a hook..
			return bank.read_hook_hword(real_addr);
		}
		cycles += bank.cycles_hword; 						// increase mmu cycles counter
	} else { 												// attempt to read a byte
		return mmu_readbyte(address);
	}
}

uint32_t mmu_readword(uint32_t address)
{
	mmu_bank bank = banks[address >> 24]; 					// get the bank to use
	if (bank.read_width_attr & BANK_CAN_WORD) { 			// are we able to read a word from this bank?
		uint32_t real_addr = (address & 0x00FFFFFF) & ~(3); // get (aligned) address relative to bank start
		if (bank.read_hook_word == NULL) { 					// is there no hook?
			return *(uint32_t*)(bank.ptr + real_addr);
		} else { 											// there is a hook..
			return bank.read_hook_word(real_addr);
		}
		cycles += bank.cycles_word; 						// increase mmu cycles counter
	} else { 												// attempt to read a byte
		return mmu_readbyte(address);
	}
}

void mmu_writebyte(uint32_t address, uint8_t value)
{
	mmu_bank bank = banks[address >> 24]; 			// get the bank to use
	if (bank.write_width_attr & BANK_CAN_BYTE) { 	// are we able to write a byte to this bank?
		uint32_t real_addr = address & 0x00FFFFFF; 	// get address relative to bank start
		if (bank.write_hook_byte == NULL) { 		// is there no hook?
			bank.ptr[real_addr] = value;
		} else { 									// there is a hook..
			bank.write_hook_byte(real_addr, value);
		}
		cycles += bank.cycles_byte; 				// increase mmu cycles counter
	} else {
		mmu_writehword(address, (value << 8) | value);
	}
}

void mmu_writehword(uint32_t address, uint16_t value)
{
	mmu_bank bank = banks[address >> 24]; 					// get the bank to use
	if (bank.write_width_attr & BANK_CAN_HWORD) { 			// are we able to write a hword to this bank?
		uint32_t real_addr = (address & 0x00FFFFFF) & ~(1); // get (aligned) address relative to bank start
		if (bank.write_hook_hword == NULL) { 				// is there no hook?
			*(uint16_t*)(bank.ptr + real_addr) = value;
		} else { 											// there is a hook..
			bank.write_hook_hword(real_addr, value);
		}
		cycles += bank.cycles_hword; 						// increase mmu cycles counter
	} else { 												// attempt to write a byte
		mmu_writebyte(address, value & 0xFF);
	}
}

void mmu_writeword(uint32_t address, uint32_t value)
{
	mmu_bank bank = banks[address >> 24]; 					// get the bank to use
	if (bank.write_width_attr & BANK_CAN_WORD) { 			// are we able to write a word to this bank?
		uint32_t real_addr = (address & 0x00FFFFFF) & ~(3); // get (aligned) address relative to bank start
		if (bank.write_hook_word == NULL) { 				// is there no hook?
			*(uint32_t*)(bank.ptr + real_addr) = value;
		} else { 											// there is a hook..
			bank.write_hook_word(real_addr, value);
		}
		cycles += bank.cycles_word; 						// increase mmu cycles counter
	} else { 												// attempt to write a byte
		mmu_writebyte(address, value & 0xFF);
	}
}
