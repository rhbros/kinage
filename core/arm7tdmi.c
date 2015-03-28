#include "arm7tdmi.h"
#include <time.h>

#define reg(i) *gprs[i]

#define THUMB (cpsr & 0b100000) == 0b100000

#define BSIGN ((cpsr & FLAG_SIGN) == FLAG_SIGN)
#define BZERO ((cpsr & FLAG_ZERO) == FLAG_ZERO)
#define BCARRY ((cpsr & FLAG_CARRY) == FLAG_CARRY)
#define BOVERFLOW ((cpsr & FLAG_OVERFLOW) == FLAG_OVERFLOW)

#define CARRY ((cpsr >> 29) & 1)

#define FLAG_SIGN 0x80000000
#define FLAG_ZERO 0x40000000
#define FLAG_CARRY 0x20000000
#define FLAG_OVERFLOW 0x10000000

#define bool_carry(n) {\
    if ((n)) cpsr |= FLAG_CARRY;\
        else cpsr &= ~(FLAG_CARRY);\
}

#define bool_overflow(n) {\
    if ((n)) cpsr |= FLAG_OVERFLOW;\
        else cpsr &= ~(FLAG_OVERFLOW);\
}

#define update_sign(n) {\
    if ((n) & 0x80000000) cpsr |= FLAG_SIGN;\
        else cpsr &= ~(FLAG_SIGN);\
}

#define update_zero(n) {\
    if ((n) == 0) cpsr |= FLAG_ZERO;\
        else cpsr &= ~(FLAG_ZERO);\
}

#define cond_overflow_add(r, op1, op2) ((op1) >> 31 == (op2) >> 31) && ((r) >> 31 != (op2) >> 31)
#define cond_overflow_sub(r, op1, op2) ((op1) >> 31 != (op2) >> 31) && ((r) >> 31 == (op2) >> 31)

#define LSL(rd,rs,n) {\
    if ((n) > 0) {\
        bool_carry( (reg(rs) << ((n) - 1)) & 0x80000000 );\
        reg(rd) = reg(rs) << (n);\
    } else {\
        reg(rd) = reg(rs);\
    }\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

#define LSR(rd,rs,n) {\
    if ((n) > 0)\
    {\
        bool_carry( (reg(rs) >> ((n) - 1)) & 1 );\
        reg(rd) = reg(rs) >> (n);\
    } else {\
        reg(rd) = reg(rs);\
    }\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

#define ASR(rd,rs,n) {\
    if ((n) > 0)\
    {\
        int32_t result = (int32_t)reg(rs) >> (int32_t)(n);\
        bool_carry( (reg(rs) >> ((n) - 1)) & 1 );\
        reg(rd) = (uint32_t)result;\
    } else {\
        reg(rd) = reg(rs);\
    }\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

#define ROR(rd,rs,n) {\
    if ((n) > 0)\
    {\
        bool_carry( (reg(rs) >> ((n) - 1)) & 1 );\
        reg(rd) =  (reg(rs) << (32 - (n))) | (reg(rs) >> (n));\
    } else {\
        reg(rd) = reg(rs);\
    }\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

#define ADDS(rd,rs,n) {\
    uint64_t result64 = (uint64_t)reg(rs) + (uint64_t)(n);\
    uint32_t result = reg(rs) + (n);\
    bool_carry(result64 & 0x100000000);\
    bool_overflow( cond_overflow_add(result, reg(rs), n) );\
    reg(rd) = result;\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

// Check overflow stuff, NO$GBA seems to differ here :)
#define SUBS(rd,rs,n) {\
    uint32_t result = reg(rs) - (n);\
    bool_carry(reg(rs) >= (n));\
    bool_overflow( cond_overflow_sub(result, reg(rs), n) );\
    reg(rd) = result;\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

// Check overflow stuff, NO$GBA seems to differ here :)
#define CMP(rs,n) {\
    uint32_t result = reg(rs) - (n);\
    bool_carry(reg(rs) >= (n));\
    bool_overflow( cond_overflow_sub(result, reg(rs), n) );\
    update_sign(result);\
    update_zero(result);\
}


// This MAY not be the correct way..
#define ADCS(rd,rs,n) {\
    uint64_t result64 = (uint64_t)reg(rs) + (uint64_t)(n) + (uint64_t)CARRY;\
    uint32_t result = reg(rs) + (n) + CARRY;\
    bool_carry(result64 & 0x100000000);\
    bool_overflow( cond_overflow_add(result, reg(rs), n) );\
    reg(rd) = result;\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

// Check overflow stuff, NO$GBA seems to differ here :)
// This MAY not be the correct way..
#define SBCS(rd,rs,n) {\
    uint32_t result = reg(rs) - (n) - !CARRY;\
    bool_carry(reg(rs) >= (n));\
    bool_overflow( cond_overflow_sub(result, reg(rs), n) );\
    reg(rd) = result;\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

// Check overflow stuff, NO$GBA seems to differ here :)
#define RSBS(rd,rs,n) {\
    uint32_t result = (n) - reg(rs);\
    bool_carry((n) >= reg(rs));\
    bool_overflow( cond_overflow_sub(result, n, reg(rs)) );\
    reg(rd) = result;\
    update_sign(reg(rd));\
    update_zero(reg(rd));\
}

#define CMN(rs,n) {\
    uint64_t result64 = (uint64_t)reg(rs) + (uint64_t)(n);\
    uint32_t result = reg(rs) + (n);\
    bool_carry(result64 & 0x100000000);\
    bool_overflow( cond_overflow_add(result, reg(rs), n) );\
    update_sign(result);\
    update_zero(result);\
}

uint32_t* gprs[16];
uint32_t r0;
uint32_t r1;
uint32_t r2;
uint32_t r3;
uint32_t r4;
uint32_t r5;
uint32_t r6;
uint32_t r7;
uint32_t r8;
uint32_t r9;
uint32_t r10;
uint32_t r11;
uint32_t r12;
uint32_t r13;
uint32_t r14;
uint32_t r15;
uint32_t r8_fiq;
uint32_t r9_fiq;
uint32_t r10_fiq;
uint32_t r11_fiq;
uint32_t r12_fiq;
uint32_t r13_fiq;
uint32_t r14_fiq;
uint32_t r13_svc;
uint32_t r14_svc;
uint32_t r13_abt;
uint32_t r14_abt;
uint32_t r13_irq;
uint32_t r14_irq;
uint32_t r13_und;
uint32_t r14_und;

uint32_t cpsr;
uint32_t spsr_fiq;
uint32_t spsr_svc;
uint32_t spsr_abt;
uint32_t spsr_irq;
uint32_t spsr_und;
uint32_t spsr_dummy; // this is a dummy for code safety (cause sys / usr dont have a spsr)
uint32_t* pspsr; // shorthand

uint32_t op_pipe1;
uint32_t op_pipe2;
uint32_t op_pipe3;
uint8_t pipe_state;
bool branched;

/* This resets the virtual cpu to its origin state */
void arm7_reset()
{
    gprs[0] = &r0;
    gprs[1] = &r1;
    gprs[2] = &r2;
    gprs[3] = &r3;
    gprs[4] = &r4;
    gprs[5] = &r5;
    gprs[6] = &r6;
    gprs[7] = &r7;
    gprs[15] = &r15;

    cpsr = 0x1F;
    //cpsr |= 0b100000;
    arm7_update_regs();

    for (int i = 0; i < 16; i++)
        *gprs[i] = 0;

    pipe_state = 0;

    //reg(0) = 0x08000109;
    //reg(13) = 0x03007F00;
    //r15 = 0x8000108;
    ////reg(0) = 0x08000101;
    ////reg(13) = 0x03007F00;
    ////r15 = 0x8000100;
    r15 = 0x08000000;
}

/* This function applies mode specific register mapping */
void arm7_update_regs()
{
    switch (cpsr & 0x1F) // which mode?
    {
    case 0x10: // user
        gprs[8] = &r8;
        gprs[9] = &r9;
        gprs[10] = &r10;
        gprs[11] = &r11;
        gprs[12] = &r12;
        gprs[13] = &r13;
        gprs[14] = &r14;
        pspsr = &spsr_dummy;
        break;
    case 0x11: // fiq
        gprs[8] = &r8_fiq;
        gprs[9] = &r9_fiq;
        gprs[10] = &r10_fiq;
        gprs[11] = &r11_fiq;
        gprs[12] = &r12_fiq;
        gprs[13] = &r13_fiq;
        gprs[14] = &r14_fiq;
        pspsr = &spsr_fiq;
        break;
    case 0x12: // irq
        gprs[8] = &r8;
        gprs[9] = &r9;
        gprs[10] = &r10;
        gprs[11] = &r11;
        gprs[12] = &r12;
        gprs[13] = &r13_irq;
        gprs[14] = &r14_irq;
        pspsr = &spsr_irq;
        break;
    case 0x13: // supervisor (swi)
        gprs[8] = &r8;
        gprs[9] = &r9;
        gprs[10] = &r10;
        gprs[11] = &r11;
        gprs[12] = &r12;
        gprs[13] = &r13_svc;
        gprs[14] = &r14_svc;
        pspsr = &spsr_svc;
        break;
    case 0x17: // abort
        gprs[8] = &r8;
        gprs[9] = &r9;
        gprs[10] = &r10;
        gprs[11] = &r11;
        gprs[12] = &r12;
        gprs[13] = &r13_abt;
        gprs[14] = &r14_abt;
        pspsr = &spsr_abt;
        break;
    case 0x1B: // undefined
        gprs[8] = &r8;
        gprs[9] = &r9;
        gprs[10] = &r10;
        gprs[11] = &r11;
        gprs[12] = &r12;
        gprs[13] = &r13_und;
        gprs[14] = &r14_und;
        pspsr = &spsr_und;
        break;
    case 0x1F: // system
        gprs[8] = &r8;
        gprs[9] = &r9;
        gprs[10] = &r10;
        gprs[11] = &r11;
        gprs[12] = &r12;
        gprs[13] = &r13;
        gprs[14] = &r14;
        pspsr = &spsr_dummy;
        break;
    default:
        // throw cpu exception?
        break;
    }
}

void arm7_regdump()
{
    for (int i = 0; i < 16; i++)
    {
        printf("r%d: 0x%x\n", i, reg(i));
    }
    printf("cpsr: %x\n", cpsr);
}

void arm7_execute_thumb(uint32_t op)
{
	if ((op & 0xF800) < 0x1800) // THUMB.1 Move shifted register
	{
		uint32_t rd = op & 7;
		uint32_t rs = (op >> 3) & 7;
		uint32_t imm5 = (op >> 6) & 0x1F;
		switch ((op >> 11) & 3)
		{
		case 0b00: // LSL
			LSL(rd, rs, imm5);
			break;
		case 0b01: // LSR
			LSR(rd, rs, imm5);
			break;
		case 0b10: // ASR
			ASR(rd, rs, imm5);
			break;
		}
	} else if ((op & 0xF800) == 0x1800) { // THUMB.2 Add/subtract
		uint32_t rd = op & 7;
		uint32_t rs = (op >> 3) & 7;
		uint32_t field3 = (op >> 6) & 7;
		switch ((op >> 9) & 3)
		{
		case 0b00: // ADD REG
			ADDS(rd, rs, reg(field3));
			break;
		case 0b01: // SUB REG
			SUBS(rd, rs, reg(field3));
			break;
		case 0b10: // ADD IMM
			ADDS(rd, rs, field3);
			break;
		case 0b11: // SUB IMM
			SUBS(rd, rs, field3);
			break;
		}
	} else if ((op & 0xE000) == 0x2000) { // THUMB.3 Move/compare/add/subtract immediate
		uint32_t imm8 = op & 0xFF;
		uint32_t rd = (op >> 8) & 7;
		switch ((op >> 11) & 3)
		{
		case 0b00: // MOV
			update_sign(false);
			update_zero(imm8);
			reg(rd) = imm8;
			break;
		case 0b01: // CMP
			CMP(rd, imm8);
			break;
		case 0b10: // ADD
			ADDS(rd, rd, imm8);
			break;
		case 0b11: // SUB
			SUBS(rd, rd, imm8);
			break;
		}
	} else if ((op & 0xFC00) == 0x4000) { // THUMB.4 ALU operations
		uint32_t rd = op & 7;
		uint32_t rs = (op >> 3) & 7;
		switch ((op >> 6) & 0xF)
		{
		case 0b0000: // AND
			reg(rd) &= reg(rs);
			update_sign(reg(rd));
			update_zero(reg(rd));
			break;
		case 0b0001: // EOR
			reg(rd) ^= reg(rs);
			update_sign(reg(rd));
			update_zero(reg(rd));
			break;
		case 0b0010: // LSL
			LSL(rd, rd, reg(rs));
			break;
		case 0b0011: // LSR
			LSR(rd, rd, reg(rs));
			break;
		case 0b0100: // ASR
			ASR(rd, rd, reg(rs));
			break;
		case 0b0101: // ADC
			ADCS(rd, rd, reg(rs));
			break;
		case 0b0110: // SBC
			SBCS(rd, rd, reg(rs));
			break;
		case 0b0111: // ROR
			ROR(rd, rd, reg(rs));
			break;
		case 0b1000: // TST
		{
			uint32_t result = reg(rd) & reg(rs);
			update_sign(result);
			update_zero(result);
			break;
		}
		case 0b1001: // NEG
			RSBS(rd, rs, 0); // kinda hacky
			break;
		case 0b1010: // CMP
			CMP(rd, reg(rs));
			break;
		case 0b1011: // CMN
			CMN(rd, reg(rs));
			break;
		case 0b1100: // ORR
			reg(rd) |= reg(rs);
			update_sign(reg(rd));
			update_zero(reg(rd));
			break;
		case 0b1101: // MUL (check for correctnes)
			if ( (reg(rd) & 0xFFFF0000) && (reg(rs) & 0xFFFF0000) ) // We can't do all at once since we would loose bits!
	        {
	            uint64_t hi = ((reg(rd) & 0xFFFF0000) * reg(rs)) & 0xFFFFFFFF;
	            uint64_t lo = ((reg(rd) & 0x0000FFFF) * reg(rs)) & 0xFFFFFFFF;
	            reg(rd) = (hi + lo) & 0xFFFFFFFF;
	        } else {
	            reg(rd) *= reg(rs);
	        }
	        update_sign(reg(rd));
	        update_zero(reg(rd));
	        bool_carry(false);
			break;
		case 0b1110: // BIC
			reg(rd) &= ~(reg(rs));
			update_sign(reg(rd));
			update_zero(reg(rd));
			break;
		case 0b1111: // MVN
			reg(rd) = ~(reg(rs));
			update_sign(reg(rd));
			update_zero(reg(rd));
			break;
		}
	} else if ((op & 0xFC00) == 0x4400) { // THUMB.5 Hi register operations/branch exchange
		uint32_t rdhd = op & 7;
		uint32_t rshs = (op >> 3) & 7;
		switch ((op >> 6) & 0xF)
		{
		case 0b0001: // ADD RD HS
			reg(rdhd) += reg(rshs + 8);
			break;
		case 0b0010: // ADD HD RS
			reg(rdhd + 8) += reg(rshs);
			break;
		case 0b0011: // ADD HD HS
			reg(rdhd + 8) += reg(rshs + 8);
			break;
		case 0b0101: // CMP RD HS
			CMP(rdhd, reg(rshs + 8));
			break;
		case 0b0110: // CMP HD RS
			CMP(rdhd + 8, reg(rshs));
			break;
		case 0b0111: // CMP HD HS
			CMP(rdhd + 8, reg(rshs + 8));
			break;
		case 0b1001: // MOV RD HS
			reg(rdhd) = reg(rshs + 8);
			break;
		case 0b1010: // MOV HD RS
			reg(rdhd + 8) = reg(rshs);
			break;
		case 0b1011: // MOV HD HS
			reg(rdhd + 8) = reg(rshs + 8);
			break;
		case 0b1100: // BX RS
			if (!(reg(rshs) & 1))
	        {
	            cpsr &= 0xFFFFFFDF;
	            r15 = reg(rshs) & 0xFFFFFFFC;
	        } else {
	            r15 = reg(rshs) & 0xFFFFFFFE;
	        }
	        pipe_state = 0;
	        branched = true;
			break;
		case 0b1101: // BX HS
			if (!(reg(rshs + 8) & 1))
	        {
	            cpsr &= 0xFFFFFFDF;
	            r15 = reg(rshs + 8) & 0xFFFFFFFC;
	        } else {
	            r15 = reg(rshs + 8) & 0xFFFFFFFE;
	        }
	        pipe_state = 0;
	        branched = true;
			break;
		}
	} else if ((op & 0xF800) == 0x4800) { // THUMB.6 PC-relative load
		uint32_t imm8 = op & 0xFF;
		uint32_t rd = (op >> 8) & 7;
		reg(rd) = arm7_read(r15 + (imm8 << 2));
	} else if ((op & 0xF200) == 0x5000) { // THUMB.7 Load/store with register offset
		uint32_t rd = op & 7;
		uint32_t rb = (op >> 3) & 7;
		uint32_t ro = (op >> 6) & 7;
		switch ((op >> 10) & 3)
		{
		case 0b00: // STR
			arm7_write(reg(rb) + reg(ro), reg(rd));
			break;
		case 0b01: // STRB
			arm7_writeb(reg(rb) + reg(ro), reg(rd) & 0xFF);
			break;
		case 0b10: // LDR
			reg(rd) = arm7_read(reg(rb) + reg(ro));
			break;
		case 0b11: // LDRB	
			reg(rd) = arm7_readb(reg(rb) + reg(ro));
			break;
		}
	} else if ((op & 0xF200) == 0x5200) { // THUMB.8 Load/store sign-extended byte/halfword
		uint32_t rd = op & 7;
		uint32_t rb = (op >> 3) & 7;
		uint32_t ro = (op >> 6) & 7;
		switch ((op >> 10) & 3)
		{
		case 0b00: // STRH
			arm7_writeh(reg(rb) + reg(ro), reg(rd));
			break;	
		case 0b01: // LDSB
			reg(rd) = arm7_readb(reg(rb) + reg(ro));
			if (reg(rd) & 0x80)
				reg(rd) |= 0xFFFFFF00;
			break;
		case 0b10: // LDRH
			reg(rd) = arm7_readh(reg(rb) + reg(ro));
			break;
		case 0b11: // LDSH
			reg(rd) = arm7_readh(reg(rb) + reg(ro));
			if (reg(rd) & 0x8000)
				reg(rd) |= 0xFFFF0000;
			break;
		}
	} else if ((op & 0xE000) == 0x6000) { // THUMB.9 Load store with immediate offset
		uint32_t rd = op & 7;
		uint32_t rb = (op >> 3) & 7;
		uint32_t imm5 = (op >> 6) & 0x1F;
		switch ((op >> 11) & 3)
		{
		case 0b00: // STR
			arm7_write(reg(rb) + (imm5 << 2), reg(rd));
			break;
		case 0b01: // LDR
			reg(rd) = arm7_read(reg(rb) + (imm5 << 2));
			break;
		case 0b10: // STRB
			arm7_writeb(reg(rb) + imm5, reg(rd));
			break;
		case 0b11: // LDRB
			reg(rd) = arm7_readb(reg(rb) + imm5);
			break;
		}
	} else if ((op & 0xF000) == 0x8000) { // THUMB.10 Load/store halfword
		uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        if (op & (1 << 11)) // LDRH
        {
        	reg(rd) = arm7_readh(reg(rb) + (imm5 << 1));
        } else { // STRH
        	arm7_writeh(reg(rb) + (imm5 << 1), reg(rd));
        }
	} else if ((op & 0xF000) == 0x9000) { // THUMB.11 SP-relative load/store
		uint32_t imm8 = op & 0xFF;
		uint32_t rd = (op >> 8) & 7;
		if (op & (1 << 11)) // LDR
        {
        	reg(rd) = arm7_read(reg(13) + (imm8 << 2));
        } else { // STR
        	arm7_write(reg(13) + (imm8 << 2), reg(rd));
        }
	} else if ((op & 0xF000) == 0xA000) { // THUMB.12 Load address
		uint32_t imm8 = op & 0xFF;
		uint32_t rd = (op >> 8) & 7;
		if (op & (1 << 11)) // SP
		{
			reg(rd) = reg(13) + (imm8 << 2);
		} else { // PC
			reg(rd) = (r15 & 0xFFFFFFFD) + (imm8 << 2);
		}
	} else if ((op & 0xFF00) == 0xB000) { // THUMB.13 Add offset to stack pointer
		uint32_t imm7 = op & 0x7F;
		if (op & 0x80) reg(13) -= imm7;
			else reg(13) += imm7;
	} else if ((op & 0xF600) == 0xB200) { // THUMB.14 push/pop registers
		uint32_t rlist = op & 0xFF;
		if (op & (1 << 11)) // POP
		{
			// read registers
			for (int i = 7; i >= 0; i++)
	        {
	            if (rlist & (1 << i))
	            {
	                reg(i) = arm7_read(reg(13));
	                reg(13) += 4;
	            }
	        }
	        // restore pc if neccessary
	        if (op & (1 << 8))
	        {
	        	r15 = arm7_read(reg(13));
	        	branched = true;
	        	pipe_state = 0;
			}
		} else { // PUSH
			// save lr if neccessary
			if (op & (1 << 8))
			{
				reg(13) -= 4;
	        	arm7_write(reg(13), reg(14));
	        }
	        // write other registers
	        for (int i = 7; i >= 0; i--)
	        {
	            if (rlist & (1 << i))
	            {
	                reg(13) -= 4;
	                arm7_write(reg(13), reg(i));
	            }
	        }
		}
	} else if ((op & 0xF000) == 0xC000) { // THUMB.15 Multiple load/store
		uint32_t rlist = op & 0xFF;
		uint32_t rb = (op >> 8) & 7;
		if (op & (1 << 11)) // LDMIA
		{
			for (int i = 7; i >= 0; i--)
	        {
	            if (rlist & (1 << i))
	            {
	                reg(i) = arm7_read(reg(rb));
	                reg(rb) += 4;
	            }
	        }
		} else { // STMIA
			for (int i = 7; i >= 0; i--)
	        {
	            if (rlist & (1 << i))
	            {
	                arm7_write(reg(rb), reg(i));
	                reg(rb) += 4;
	            }
	        }
		}
	} else if ((op & 0xF000) == 0xD000) { // THUMB.16 Conditional branch
		uint32_t simm8 = op & 255;
		bool cond_met = false;

		// calculate corresponding condition
		switch ((op >> 8) & 0xF)
		{
		case 0b0000: cond_met = BZERO; break;
		case 0b0001: cond_met = !BZERO; break;
		case 0b0010: cond_met = BCARRY; break;
		case 0b0011: cond_met = !BCARRY; break;
		case 0b0100: cond_met = BSIGN; break;
		case 0b0101: cond_met = !BSIGN; break;
		case 0b0110: cond_met = BOVERFLOW; break;
		case 0b0111: cond_met = !BOVERFLOW; break;
		case 0b1000: cond_met = BCARRY && !BZERO; break;
		case 0b1001: cond_met = !BCARRY || BZERO; break;
		case 0b1010: cond_met = BSIGN == BOVERFLOW; break;
		case 0b1011: cond_met = BSIGN ^ BOVERFLOW; break; // check
		case 0b1100: cond_met = !BZERO && (BSIGN == BOVERFLOW); break;
		case 0b1101: cond_met = BZERO || (BSIGN ^ BOVERFLOW); break;
		} 

		if (cond_met)
		{
			if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
		}
	} else if ((op & 0xFF00) == 0xDF00) { // THUMB.17 Software Interrupt
		reg(14) = r15 - 2;
        spsr_svc = cpsr;
        r15 = 0x8;
        cpsr &= 0xFFFFFFDF;
        cpsr &= 0xFFFFFFE0;
        cpsr |= 0x13;
        arm7_update_regs();
        branched = true;
        pipe_state = 0;
	} else if ((op & 0xF800) == 0xE000) { // THUMB.18 Unconditional branch
        uint32_t imm11 = op & 2047;
        r15 += imm11 << 1;
        branched = true;
        pipe_state = 0;
    } else if ((op & 0xF000) == 0xF000) { // THUMB.19 Long branch with link
    	uint32_t imm11 = op & 2047;
    	if (op & (1 << 11)) // BH
    	{
    		uint32_t temp = r15 - 2;
	        r15 = reg(14) + (imm11 << 1);
	        reg(14) = temp | 1;
	        branched = true;
	        pipe_state = 0;
    	} else { // BL
    		reg(14) = r15 + (imm11 << 12);
    	}
    } else {
		ERROR("Invalid instruction pc=%x", r15);
		arm7_regdump();
	}

    //arm7_regdump();
    //char test[1337];
    //gets(test);
}



































void arm7_execute(uint32_t op)
{
    //uint32_t pc = r15 - 8;
    bool cond_given = false;

    switch (op >> 28)
    {
    case 0x0: cond_given = (cpsr & FLAG_ZERO) == FLAG_ZERO; break;
    case 0x1: cond_given = (cpsr & FLAG_ZERO) != FLAG_ZERO; break;
    case 0x2: cond_given = (cpsr & FLAG_CARRY) == FLAG_CARRY; break;
    case 0x3: cond_given = (cpsr & FLAG_CARRY) != FLAG_CARRY; break;
    case 0x4: cond_given = (cpsr & FLAG_SIGN) == FLAG_SIGN; break;
    case 0x5: cond_given = (cpsr & FLAG_SIGN) != FLAG_SIGN; break;
    case 0x6: cond_given = (cpsr & FLAG_OVERFLOW) == FLAG_OVERFLOW; break;
    case 0x7: cond_given = (cpsr & FLAG_OVERFLOW) != FLAG_OVERFLOW; break;
    case 0x8: cond_given = ((cpsr & FLAG_CARRY) == FLAG_CARRY) & ((cpsr & FLAG_ZERO) != FLAG_ZERO); break;
    case 0x9: cond_given = ((cpsr & FLAG_CARRY) != FLAG_CARRY) | ((cpsr & FLAG_ZERO) == FLAG_ZERO); break;
    case 0xA: cond_given = ((cpsr & FLAG_SIGN) == FLAG_SIGN) == ((cpsr & FLAG_OVERFLOW) == FLAG_OVERFLOW); break;
    case 0xB: cond_given = ((cpsr & FLAG_SIGN) == FLAG_SIGN) != ((cpsr & FLAG_OVERFLOW) == FLAG_OVERFLOW); break;
    case 0xC: cond_given = ((cpsr & FLAG_ZERO) != FLAG_ZERO) & (((cpsr & FLAG_SIGN) == FLAG_SIGN) == ((cpsr & FLAG_OVERFLOW) == FLAG_OVERFLOW)); break;
    case 0xD: cond_given = ((cpsr & FLAG_ZERO) == FLAG_ZERO) | (((cpsr & FLAG_SIGN) == FLAG_SIGN) != ((cpsr & FLAG_OVERFLOW) == FLAG_OVERFLOW)); break;
    case 0xE: cond_given = true; break;
    case 0xF: cond_given = false; break;
    }
    NOTICE("ARM EXECUTION @ %x", r15 - 8);
    if (cond_given)
    {
        //NOTICE("CONDITION MET!");
        switch ((op >> 25) & 7) // check bits 25 - 27
		{
		case 0b000:
            if ((op & 0x1FFFFF0) == 0x12FFF10) // Branch and Exchange
            {
                uint32_t rn = op & 0xF;
                if (reg(rn) & 1) // if mode switch to thumb
                {
                    cpsr |= 0b100000;
                    r15 = reg(rn) & ~(1);
                } else {
                    r15 = reg(rn) & ~(3);
                }
                pipe_state = 0;
                branched = true;
            } else if ((op & 0xF0) < 0b10010000) { // Data Processing / PSR
                if ((op & 0xFBF0FFF) == 0x10F0080) // MRS (transfer PSR contents to a register)
                {
                    if (op & (1 << 22)) reg((op >> 12) & 0xF) = *pspsr;
                        else reg((op >> 12) & 0xF) = cpsr;
                } else if ((op & 0xFBFFFF0) == 0x129F000) { // MSR (transfer register contents to PSR)
                    uint32_t rm = op & 0xF;
                    if (op & (1 << 22))
                    {
                        *pspsr = reg(rm);
                    } else {
                        if ((cpsr & 0x1f) == 0x10) 
                        {
                            cpsr = (cpsr & 0x0FFFFFFF) | (reg(rm) & 0xF0000000);
                        } else {
                            cpsr = reg(rm);
                            arm7_update_regs();
                        }
                    }
                } else if ((op & 0xFBFFFF0) == 0x128F000) { // MSR (transfer register contents to PSR flag bits only)
                    uint32_t rm = op & 0xF;
                    if (op & (1 << 22))
                    {
                        *pspsr = (*pspsr & 0x0FFFFFFF) | (reg(rm) & 0xF0000000);
                    } else {
                        cpsr = (cpsr & 0x0FFFFFFF) | (reg(rm) & 0xF0000000);
                    }
                } else { // Data Processing (operand2 is register)
                    uint32_t op2 = reg(op & 0xF);
                    uint32_t shift = (op >> 4) & 0xFF;
                    uint32_t rd = (op >> 12) & 0xF;
                    uint32_t rn = (op >> 16) & 0xF;
                    uint32_t amount = 0;
                    bool tmp_carry = (cpsr & FLAG_CARRY) == FLAG_CARRY;
                    bool flags = op & (1 << 20);

                    // decode amount
                    if (shift & 1) amount = (reg((shift >> 4) & 0xF)) & 0xFF;
                        else amount = (shift >> 3) & 0x1f;

                    // apply shift to opcode2 if neccessary
                    if (amount != 0)
                    {
                        switch ((shift >> 1) & 3)
                        {
                        case 0b00: // LSL
                        {
                            uint32_t result = op2 << amount;
                            tmp_carry = (op2 << (amount - 1)) & 0x80000000;
                            op2 = result;
                            break;
                        }
                        case 0b01: // LSR
                        {
                            uint32_t result = op2 >> amount;
                            tmp_carry = (op2 >> (amount - 1)) & 1;
                            op2 = result;
                            break; 
                        }
                        case 0b10: // ASR
                        {
                            int32_t result = (int32_t)op2 >> (int32_t)amount;
                            tmp_carry = (op2 >> (amount - 1)) & 1;
                            op2 = (uint32_t)result;
                            break;
                        }
                        case 0b11: // ROR
                        {
                            uint32_t result = (op << (32 - amount)) | (op2 >> amount);
                            tmp_carry = (op2 >> (amount - 1)) & 1;
                            op2 = result;
                            break;
                        }
                        }
                    }

                    // do the actual opcode magic
                    switch ((op >> 21) & 0xF)
                    {
                    case 0b0000: // AND
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) & op2;
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) & op2;
                        }
                        break;
                    }
                    case 0b0001: // EOR
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) ^ op2;
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) ^ op2;
                        }
                        break;
                    }
                    case 0b0010: // SUB
                    {
                        if (flags)
                        {
                            SUBS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) - op2;
                        }
                        break;
                    }
                    case 0b0011: // RSB
                    {
                        if (flags)
                        {
                            RSBS(rd, rn, op2);
                        } else {
                            reg(rd) = op2 + reg(rn);
                        }
                        break;
                    }
                    case 0b0100: // ADD
                    {
                        if (flags)
                        {
                            ADDS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) + op2;
                        }
                        break;
                    }
                    case 0b0101: // ADC
                    {
                        if (flags)
                        {
                            ADCS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) + op2 + CARRY;
                        }
                        break;
                    }
                    case 0b0110: // SBC
                    {
                        if (flags)
                        {
                            SBCS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) - reg(op2) + CARRY - 1;
                        }
                        break;
                    }
                    case 0b0111: // RSC
                    {
                        if (flags)
                        {
                            RSBS(rd, rn, op - CARRY); // this is hacky... maybe write custom code?
                        } else {
                            reg(rd) = op - reg(rn) + CARRY - 1;
                        }
                        break;
                    }
                    case 0b1000: // TST
                    {
                        uint32_t result = reg(rn) & op2;
                        update_sign(result);
                        update_zero(result);
                        break;
                    }
                    case 0b1001: // TEQ
                    {
                        uint32_t result = reg(rn) ^ op2;
                        update_sign(result);
                        update_zero(result);
                        break;
                    }
                    case 0b1010: // CMP
                    {
                        CMP(rn, op2);
                        break;
                    }
                    case 0b1011: // CMN
                    {
                        CMN(rn, op2);
                        break;
                    }
                    case 0b1100: // ORR
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) | op2;
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) | op2;
                        }
                        break;
                    }
                    case 0b1101: // MOV
                    {
                        if (flags) 
                        {
                            update_sign(op2);
                            update_zero(op2);
                            bool_carry(tmp_carry);
                        }
                        reg(rd) = op2;
                        break;
                    }
                    case 0b1110: // BIC
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) & ~(op2);
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) & ~(op2);
                        }
                        break;
                    }
                    case 0b1111: // MVN
                    {
                        if (flags) 
                        {
                            update_sign(~(op2));
                            update_zero(~(op2));
                            bool_carry(tmp_carry);
                        }
                        reg(rd) = ~(op2);
                        break;
                    }
                    }

                    // r15 specific stuff
                    if (rd == 15)
                    {
                        // flush pipeline
                        pipe_state = 0;
                        branched = true;
                        // mode switch if s bit set
                        if (flags)
                        {
                            cpsr = *pspsr;
                            arm7_update_regs();
                        }
                    }
                }
            } else if ((op & 0xF0) == 0b10010000) { // Multiply / Multiply Long / Single Data Swap
                ERROR("Multiply (Long) / Single Data Swap not implemented");
                //char test[1337];
                //gets(test);
            } else { // Halfword Data Transfer
                uint32_t sh = (op >> 5) & 3;
                uint32_t rd = (op >> 12) & 0xF;
                uint32_t rn = (op >> 16) & 0xF;
                bool load_bit = op & (1 << 20);
                bool write_bit = op & (1 << 21);
                bool immediate_bit = op & (1 << 22);
                bool add_bit = op & (1 << 23);
                bool pre_bit = op & (1 << 24);
                uint32_t tmp_base = reg(rn);

                if (pre_bit)
                {
                    if (immediate_bit)
                    {
                        uint32_t offset = ((op >> 4) & 0xF0) | (op & 0xF);
                        if (add_bit) tmp_base += offset;
                            else tmp_base -= offset;
                    } else {
                        uint32_t rm = op & 0xF;
                        if (add_bit) tmp_base += reg(rm);
                            else tmp_base -= reg(rm);
                    }
                }

                if (!load_bit)
                NOTICE("0x%x = 0x%x", tmp_base, reg(rd));

                switch (sh)
                {
                case 0b01:
                {
                    if (load_bit) reg(rd) = arm7_readh(tmp_base);
                        else arm7_writeh(tmp_base, reg(rd));
                    break;
                }
                case 0b10:
                {
                    if (load_bit)
                    {
                        uint32_t value = arm7_readb(tmp_base);
                        if (value & 0x80) value |= 0xFFFFFF00;
                        reg(rd) = value;
                    } else {
                        uint32_t value = reg(rd);
                        if (value & 0x80) value |= 0xFFFFFF00;
                        arm7_write(tmp_base, value);
                    }
                    break;
                }
                case 0b11:
                {
                    if (load_bit)
                    {
                        uint32_t value = arm7_readh(tmp_base);
                        if (value & 0x8000) value |= 0xFFFF0000;
                        reg(rd) = value;
                    } else {
                        uint32_t value = reg(rd);
                        if (value & 0x8000) value |= 0xFFFF0000;
                        arm7_write(tmp_base, value);
                    }
                    break;
                }
                default:
                {
                    ERROR("Undefined instruction");
                }
                }

                if (!pre_bit)
                {
                    if (immediate_bit)
                    {
                        uint32_t offset = ((op >> 4) & 0xF0) | (op & 0xF);
                        if (add_bit) tmp_base += offset;
                            else tmp_base -= offset;
                    } else {
                        uint32_t rm = op & 0xF;
                        if (add_bit) tmp_base += reg(rm);
                            else tmp_base -= reg(rm);
                    }
                }

                if (load_bit && rd == 15)
                {
                    pipe_state = 0;
                    branched = true;
                }

                if (write_bit)
                    reg(rn) = tmp_base;

                //NOTICE("END HALFWORD TRANSFER");

                //ERROR("Halfword Data Transfer not implemented");
            }
			break;
		case 0b001: // Immediate Data Transfer or MSR
            if ((op & 0xFBFF000) == 0x328F000) // MSR (transfer immediate value to PSR flag bits only)
            {
                uint32_t imm = op & 0xFF;
                uint32_t rotate = (op >> 8) & 0xF;
                imm = (imm << (32 - rotate)) | (imm >> rotate);
                if (op & (1 << 22))
                {
                    *pspsr = (*pspsr & 0x0FFFFFFF) | (imm & 0xF0000000);
                } else {
                    cpsr = (cpsr & 0x0FFFFFFF) | (imm & 0xF0000000);
                }
            } else { // Data Processing (operand2 is immediate)
                    uint32_t op2 = op & 0xFF;
                    //uint32_t shift = (op >> 4) & 0xFF;
                    uint32_t rd = (op >> 12) & 0xF;
                    uint32_t rn = (op >> 16) & 0xF;
                    bool tmp_carry = (cpsr & FLAG_CARRY) == FLAG_CARRY;
                    bool flags = op & (1 << 20);

                    // 32-bit extend operand2
                    if (op2 & 0x80) op2 |= 0xFFFFFF00;

                    // do rotate right
                    int rotate = ((op >> 8) & 0xF) << 1;
                    if (rotate != 0)
                    {
                        uint32_t result = (op2 << (32 - rotate)) | (op2 >> rotate);
                        tmp_carry = (result >> (rotate - 1)) & 1;
                        op2 = result;
                    }

                    // do the actual opcode magic
                    switch ((op >> 21) & 0xF)
                    {
                    case 0b0000: // AND
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) & op2;
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) & op2;
                        }
                        break;
                    }
                    case 0b0001: // EOR
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) ^ op2;
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) ^ op2;
                        }
                        break;
                    }
                    case 0b0010: // SUB
                    {
                        if (flags)
                        {
                            SUBS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) - op2;
                        }
                        break;
                    }
                    case 0b0011: // RSB
                    {
                        if (flags)
                        {
                            RSBS(rd, rn, op2);
                        } else {
                            reg(rd) = op2 + reg(rn);
                        }
                        break;
                    }
                    case 0b0100: // ADD
                    {
                        if (flags)
                        {
                            ADDS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) + op2;
                        }
                        break;
                    }
                    case 0b0101: // ADC
                    {
                        if (flags)
                        {
                            ADCS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) + op2 + CARRY;
                        }
                        break;
                    }
                    case 0b0110: // SBC
                    {
                        if (flags)
                        {
                            SBCS(rd, rn, op2);
                        } else {
                            reg(rd) = reg(rn) - reg(op2) + CARRY - 1;
                        }
                        break;
                    }
                    case 0b0111: // RSC
                    {
                        if (flags)
                        {
                            RSBS(rd, rn, op - CARRY); // this is hacky... maybe write custom code?
                        } else {
                            reg(rd) = op - reg(rn) + CARRY - 1;
                        }
                        break;
                    }
                    case 0b1000: // TST
                    {
                        uint32_t result = reg(rn) & op2;
                        update_sign(result);
                        update_zero(result);
                        break;
                    }
                    case 0b1001: // TEQ
                    {
                        uint32_t result = reg(rn) ^ op2;
                        update_sign(result);
                        update_zero(result);
                        break;
                    }
                    case 0b1010: // CMP
                    {
                        CMP(rn, op2);
                        break;
                    }
                    case 0b1011: // CMN
                    {
                        CMN(rn, op2);
                        break;
                    }
                    case 0b1100: // ORR
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) | op2;
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) | op2;
                        }
                        break;
                    }
                    case 0b1101: // MOV
                    {
                        if (flags) 
                        {
                            update_sign(op2);
                            update_zero(op2);
                            bool_carry(tmp_carry);
                        }
                        reg(rd) = op2;
                        break;
                    }
                    case 0b1110: // BIC
                    {
                        if (flags)
                        {
                            uint32_t result = reg(rn) & ~(op2);
                            update_sign(result);
                            update_zero(result);
                            bool_carry(tmp_carry);
                            reg(rd) = result;
                        } else {
                            reg(rd) = reg(rn) & ~(op2);
                        }
                        break;
                    }
                    case 0b1111: // MVN
                    {
                        if (flags) 
                        {
                            update_sign(~(op2));
                            update_zero(~(op2));
                            bool_carry(tmp_carry);
                        }
                        reg(rd) = ~(op2);
                        break;
                    }
                    }

                    // r15 specific stuff
                    if (rd == 15)
                    {
                        // flush pipeline
                        pipe_state = 0;
                        branched = true;
                        // mode switch if s bit set
                        if (flags)
                        {
                            cpsr = *pspsr;
                            arm7_update_regs();
                        }
                    }
            }
			break;
		// Single Data Transfer
		case 0b010:
		case 0b011:
		{
			uint32_t offset = op & 0xFFF;
			uint32_t rd = (op >> 12) & 0xF;
			uint32_t rn = (op >> 16) & 0xF;
			bool write = op & (1 << 21);
			bool add = op & (1 << 23);
			bool pre = op & (1 << 24);
			uint32_t tmp_base = reg(rn);
            bool load = false;
			
			// Decode shift if neccesary
			if (op & (1 << 25))
			{
				uint32_t rm = op & 0xF;
				uint32_t source;
				
				// Get shift source
				if (op & (1 << 4)) source = reg((op >> 8) & 0xF);
					else source = (op >> 7) & 0x1F;
					
				// Apply shift
				switch ((op >> 5) & 3)
				{
				case 0: // Logical left
					offset = reg(rm) << source;
					break;
				case 1: // Logical right
					offset = reg(rm) >> source;
					break;
				case 2: // Arithmetic right
				{
					int32_t result = (int32_t)(reg(rm)) >> (int32_t)source;
					offset = (uint32_t)result;
					break;
				}
				case 3: // Rotate right
					offset = (reg(rm) << (32 - source)) | (reg(rm) >> source);
				}
			}
			
			if (pre) // is pre indexing enabled?
			{
				if (add) tmp_base += offset;
					else tmp_base -= offset;
			}
			
			// load / store
			if (((op & 0x500000) >> 20) == 0b000) { // str
				arm7_write(tmp_base, reg(rd));
			} else if (((op & 0x500000) >> 20) == 0b001) { // ldr
				reg(rd) = arm7_read(tmp_base);
                load = true;
			} else if (((op & 0x500000) >> 20) == 0b100) { // strb
				arm7_writeb(tmp_base, reg(rd) & 0xFF);
			} else { // ldrb
				reg(rd) = arm7_readb(tmp_base);
                load = true;
			}
			
			if (!pre) // is post indexing enabled?
			{
				if (add) tmp_base += offset;
					else tmp_base -= offset;
			}
			
			// write base back if required
			if (write)
				reg(rn) = tmp_base;

            // flush pipe if neccessary
            if (load && rd == 15)
            {
                pipe_state = 0;
                branched = true;
            }
			
			break;
		}
		// Block Data Transfer
		case 0b100:
		{
            uint32_t reg_list = op & 0xFFFF;
            uint32_t rn = (op >> 16) & 0xF;
            bool load_bit = op & (1 << 20);
            bool write_bit = op & (1 << 21);
            bool psr_bit = op & (1 << 22);
            bool add_bit = op & (1 << 23);
            bool pre_bit = op & (1 << 24);
            bool r15_bit = op & (1 << 15);
            uint32_t tmp_base = reg(rn);
            uint32_t tmp_cpsr = cpsr;

            //NOTICE("BEGIN BLOCK DATA TRANSFER");

            // apply psr bit handling
            if (psr_bit)
            {
                if (load_bit && r15_bit)
                {
                    // copy spsr_<cur> to cpsr and update regs
                    cpsr = *pspsr;
                    arm7_update_regs();
                } else {
                    // switch cpu temporary to user mode
                    cpsr = (cpsr & ~(0x1f)) | 0x10;
                    arm7_update_regs();
                }
            }

            // do actual loads / stores
            if (!load_bit && !pre_bit && !add_bit) // post-decrement store (stmed)
            {
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        arm7_write(tmp_base, reg(i));
                        tmp_base -= 4;
                    }
                }
            } else if (!load_bit && pre_bit && !add_bit) { // pre-decrement store (stmfd)
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        tmp_base -= 4;
                        arm7_write(tmp_base, reg(i));
                    }
                }
            } else if (!load_bit && !pre_bit && add_bit) { // post-increment store (stmea)
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        arm7_write(tmp_base, reg(i));
                        tmp_base += 4;
                    }
                }
            } else if (!load_bit && pre_bit && add_bit) { // pre-increment store (stmfa)
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        tmp_base += 4;
                        arm7_write(tmp_base, reg(i));
                    }
                }
            } else if (load_bit && !pre_bit && !add_bit) { // post-decrement load (ldmfa)
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        reg(i) = arm7_read(tmp_base);
                        tmp_base -= 4;
                    }
                }
            } else if (load_bit && pre_bit && !add_bit) { // pre-decrement load
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        tmp_base -= 4;
                        reg(i) = arm7_read(tmp_base);
                    }
                }
            } else if (load_bit && !pre_bit && add_bit) { // post-increment load
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        reg(i) = arm7_read(tmp_base);
                        tmp_base += 4;
                    }
                }
            } else { // pre-increment load
                for (int i = 15; i >= 0; i--)
                {
                    if (reg_list & (1 << i))
                    {
                        tmp_base += 4;
                        reg(i) = arm7_read(tmp_base);
                    }
                }
            }

            // if cpu mode was temporary changed to user mode
            if (psr_bit && !(load_bit && r15_bit))
            {
                // restore original cpu mode
                cpsr = tmp_cpsr;
                arm7_update_regs();
            }

            // flush pipe if r15 was written
            if (load_bit && r15_bit)
            {
                pipe_state = 0;
                branched = true;
            }

            // write base back if required
            if (write_bit)
                reg(rn) = tmp_base;
            //NOTICE("END BLOCK DATA TRANSFER");
			break;
		}
		// Branch / Branch with link
		case 0b101:
        {
            uint32_t shift = op & 0x00FFFFFF;

            // sign extend
            if (shift & 0x800000)
                shift |= 0xFF000000;

            // link?
            if (op & (1 << 24))
                reg(14) = r15 - 4;

            r15 += shift << 2;

            pipe_state = 0;
            branched = true;

			break;
		}
        // Coprocessor Data Transfer
		case 0b110:
			// This can be ignored at the current state because
			// we don't plan to integrate Z80 Coprocessor emulation.
            WARNING("Coprocessor Data Transfer not implemented"); // Though log it
			break;
		// Coprocessor Data Operation, Coprocessor Register Transfer and Software Interrupt
		case 0b111:
			if (op & (1 << 24)) // Software Interrupt
			{
                reg(14) = r15 - 4; // save return address
				r15 = 8; // let pc point to the fixed supervisor address;
                spsr_svc = cpsr; // save the current status
                
                // update mode
                cpsr = (cpsr & ~(0x1F)) | 0x13;
                arm7_update_regs();

                // fix / flush pipeline
                branched = true;
                pipe_state = 0;
			} else { // Coprocessor Data Operation, Coprocessor Register Transfer
				// This can be ignored at the current state because
				// we don't plan to integrate Z80 Coprocessor emulation.
                WARNING("Coprocessor Data Operation / Register Transfer not implemented"); // Though log it
			}
			break;
		}
    }
        
    arm7_regdump();
    //char test[1337];
    //gets(test);
}

/* Does next processor step */
void arm7_step()
{
    if (THUMB) // Are we in THUMB mode?
    {
        switch (pipe_state)
        {
        case 0:
            op_pipe1 = arm7_readh(r15);
            break;
        case 1:
            op_pipe2 = arm7_readh(r15);
            break;
        case 2:
            arm7_execute_thumb(op_pipe1);
            op_pipe3 = arm7_readh(r15);
            break;
        case 3:
            op_pipe1 = arm7_readh(r15);
            arm7_execute_thumb(op_pipe2);
            break;
        case 4:
            op_pipe2 = arm7_readh(r15);
            arm7_execute_thumb(op_pipe3);
            break;
        }
        if (branched)
        {
            branched = false;
            return;
        }
        r15 += 2;
    } else { // else we are in ARM mode
        switch (pipe_state)
        {
        case 0:
            op_pipe1 = arm7_read(r15);
            break;
        case 1:
            op_pipe2 = arm7_read(r15);
            break;
        case 2:
            arm7_execute(op_pipe1);
            op_pipe3 = arm7_read(r15);
            break;
        case 3:
            op_pipe1 = arm7_read(r15);
            arm7_execute(op_pipe2);
            break;
        case 4:
            op_pipe2 = arm7_read(r15);
            arm7_execute(op_pipe3);
            break;
        }
        if (branched)
        {
            branched = false;
            return;
        }
        r15 += 4;
    }

    pipe_state++;
    if (pipe_state == 5) pipe_state = 2;
}

uint8_t arm7_readb(uint32_t ptr)
{
    return mmu_readbyte(ptr);
}

uint16_t arm7_readh(uint32_t ptr)
{
	return mmu_readhword(ptr);
}

uint32_t arm7_read(uint32_t ptr)
{
	return mmu_readword(ptr);
}

void arm7_writeb(uint32_t ptr, uint8_t value)
{
	mmu_writebyte(ptr, value);
}

void arm7_writeh(uint32_t ptr, uint16_t value)
{
	mmu_writehword(ptr, value);
}

void arm7_write(uint32_t ptr, uint32_t value)
{
	mmu_writeword(ptr, value);
}

