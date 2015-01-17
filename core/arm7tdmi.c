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
    cpsr |= 0b100000;
    arm7_update_regs();

    for (int i = 0; i < 16; i++)
        *gprs[i] = 0;

    pipe_state = 0;

    //reg(0) = 0x08000109;
    //reg(13) = 0x03007F00;
    //r15 = 0x8000108;
    reg(0) = 0x08000101;
    reg(13) = 0x03007F00;
    r15 = 0x8000100;
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

void arm7_execute_thumb(uint32_t opcode)
{
    /* Execution logic goes here */
	uint32_t op = opcode;
    //printf("Opcode: %x\n", op);
	if ((op & 0xF800) == 0x0) { // lsl rd, rs, imm5
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        LSL(rd, rs, imm5);
    } else if ((op & 0xF800) == 0x800) { // lsr rd, rs, imm5
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        LSR(rd, rs, imm5);
    } else if ((op & 0xF800) == 0x1000) { // asr rd, rs, imm5
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        ASR(rd, rs, imm5);
    } else if ((op & 0xFE00) == 0x1800) { // add rd, rs, rn
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t rn = (op >> 6) & 7;
        ADDS(rd, rs, reg(rn));
    } else if ((op & 0xFE00) == 0x1A00) { // sub rd, rs, rn
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t rn = (op >> 6) & 7;
        SUBS(rd, rs, reg(rn));
    } else if ((op & 0xFE00) == 0x1C00) { // add rd, rs, imm3
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t imm3 = (op >> 6) & 7;
        ADDS(rd, rs, imm3);
    } else if ((op & 0xFE00) == 0x1E00) { // sub rd, rs, imm3
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t imm3 = (op >> 6) & 7;
        SUBS(rd, rs, imm3);
    } else if ((op & 0xF800) == 0x2000) { // mov rd, imm8
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        update_sign(false);
        update_zero(imm8);
        reg(rd) = imm8;
    } else if ((op & 0xF800) == 0x2800) { // cmp rd, imm8
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        CMP(rd, imm8);
    } else if ((op & 0xF800) == 0x3000) { // add rd, imm8
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        ADDS(rd, rd, imm8);
    } else if ((op & 0xF800) == 0x3800) { // sub rd, imm8
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        SUBS(rd, rd, imm8);
    } else if ((op & 0xFFC0) == 0x4000) { // and rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        reg(rd) &= reg(rs);
        update_sign(reg(rd));
        update_zero(reg(rd));
    } else if ((op & 0xFFC0) == 0x4040) { // eor rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        reg(rd) ^= reg(rs);
        update_sign(reg(rd));
        update_zero(reg(rd));
    } else if ((op & 0xFFC0) == 0x4080) { // lsl rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        LSL(rd, rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x40C0) { // lsr rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        LSR(rd, rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x4100) { // asr rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        ASR(rd, rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x4140) { // adc rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        ADCS(rd, rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x4180) { // sbc rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        SBCS(rd, rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x41C0) { // ror rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        ROR(rd, rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x4200) { // tst rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        uint32_t result = reg(rd) & reg(rs);
        update_sign(result);
        update_zero(result);
    } else if ((op & 0xFFC0) == 0x4240) { // neg rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        RSBS(rd, rs, 0);
    } else if ((op & 0xFFC0) == 0x4280) { // cmp rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        CMP(rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x42C0) { // cmn rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        CMN(rd, reg(rs));
    } else if ((op & 0xFFC0) == 0x4300) { // orr rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        reg(rd) |= reg(rs);
        update_sign(reg(rd));
        update_zero(reg(rd));
    } else if ((op & 0xFFC0) == 0x4340) { // mul rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        if ( (reg(rd) & 0xFFFF0000) && (reg(rs) & 0xFFFF0000) ) // We can't do all at once since we would loose bits!
        {
            uint64_t hi = ((reg(rd) & 0xFFFF0000) * reg(rd)) & 0xFFFFFFFF;
            uint64_t lo = ((reg(rd) & 0x0000FFFF) * reg(rd)) & 0xFFFFFFFF;
            reg(rd) = (hi + lo) & 0xFFFFFFFF;
        } else {
            reg(rd) *= reg(rs);
        }
        update_sign(reg(rd));
        update_zero(reg(rd));
        bool_carry(false);
    } else if ((op & 0xFFC0) == 0x4380) { // bic rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        reg(rd) &= ~(reg(rs));
        update_sign(reg(rd));
        update_zero(reg(rd));
    } else if ((op & 0xFFC0) == 0x43C0) { // mvn rd, rs
        uint32_t rd = op & 7;
        uint32_t rs = (op >> 3) & 7;
        reg(rd) = ~(reg(rs));
        update_sign(reg(rd));
        update_zero(reg(rd));
    } else if ((op & 0xFFC0) == 0x4440) { // add rd, hs
        uint32_t rd = op & 7;
        uint32_t hs = 8 + ((op >> 3) & 7);
        reg(rd) += reg(hs);
    } else if ((op & 0xFFC0) == 0x4480) { // add hs, rs
        uint32_t hd = 8 + (op & 7);
        uint32_t rs = (op >> 3) & 7;
        reg(hd) += reg(rs);
    } else if ((op & 0xFFC0) == 0x44C0) { // add hd, hs
        uint32_t hd = 8 + (op & 7);
        uint32_t hs = 8 + ((op >> 3) & 7);
        reg(hd) += reg(hs);
    } else if ((op & 0xFFC0) == 0x4540) { // cmp rd, hs
        uint32_t rd = op & 7;
        uint32_t hs = 8 + ((op >> 3) & 7);
        CMP(rd, reg(hs));
    } else if ((op & 0xFFC0) == 0x4580) { // cmp hd, rs
        uint32_t hd = 8 + (op & 7);
        uint32_t rs = (op >> 3) & 7;
        CMP(hd, reg(rs));
    } else if ((op & 0xFFC0) == 0x45C0) { // cmp hd, hs
        uint32_t hd = 8 + (op & 7);
        uint32_t hs = 8 + ((op >> 3) & 7);
        CMP(hd, reg(hs));
    } else if ((op & 0xFFC0) == 0x4640) { // mov rd, hs
        uint32_t rd = op & 7;
        uint32_t hs = 8 + ((op >> 3) & 7);
        reg(rd) = reg(hs);
    } else if ((op & 0xFFC0) == 0x4680) { // mov hd, rs
        uint32_t hd = 8 + (op & 7);
        uint32_t rs = (op >> 3) & 7;
        reg(hd) = reg(rs);
    } else if ((op & 0xFFC0) == 0x46C0) { // mov hd, hs
        uint32_t hd = 8 + (op & 7);
        uint32_t hs = 8 + ((op >> 3) & 7);
        reg(hd) = reg(hs);
    } else if ((op & 0xFFC0) == 0x4700) { // bx rs
        uint32_t rs = (op >> 3) & 7;
        if (!(reg(rs) & 1))
        {
            cpsr &= 0xFFFFFFDF;
            r15 = reg(rs) & 0xFFFFFFFC;
        } else {
            r15 = reg(rs) & 0xFFFFFFFE;
        }
        pipe_state = 0; // Clear the pipeline!!!!111elf
        branched = true;
    } else if ((op & 0xFFC0) == 0x4740) { // bx hs
        uint32_t hs = 8 + ((op >> 3) & 7);
        if (!(reg(hs) & 1))
        {
            cpsr &= 0xFFFFFFDF;
            r15 = reg(hs) & 0xFFFFFFFC;
        } else {
            r15 = reg(hs) & 0xFFFFFFFE;
        }
        branched = true;
        pipe_state = 0; // Clear the pipeline!!!!111elf
    } else if ((op & 0xF800) == 0x4800) { // ldr rd, [r15, imm8]
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        reg(rd) = arm7_read(r15 + (imm8 << 2));
    } else if ((op & 0xFE00) == 0x5000) { // str rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        arm7_write(reg(rb) + reg(ro), reg(rd));
    } else if ((op & 0xFE00) == 0x5400) { // strb rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        arm7_writeb(reg(rb) + reg(ro), reg(rd) & 0xFF);
    } else if ((op & 0xFE00) == 0x5800) { // ldr rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        reg(rd) = arm7_read(reg(rb) + reg(ro));
    } else if ((op & 0xFE00) == 0x5C00) { // ldrb rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        reg(rd) = arm7_readb(reg(rb) + reg(ro));
    } else if ((op & 0xFE00) == 0x5200) { // strh rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        arm7_writeh(reg(rb) + reg(ro), reg(rd));
    } else if ((op & 0xFE00) == 0x5A00) { // ldrh rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        reg(rd) = arm7_readh(reg(rb) + reg(ro));
    } else if ((op & 0xFE00) == 0x5600) { // ldsb rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        reg(rd) = arm7_readb(reg(rb) + reg(ro));
        if (reg(rd) & 0x80)
            reg(rd) |= 0xFFFFFF00;
    } else if ((op & 0xFE00) == 0x5E00) { // ldsh rd, [rb, ro]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t ro = (op >> 6) & 7;
        reg(rd) = arm7_readh(reg(rb) + reg(ro));
        if (reg(rd) & 0x8000)
            reg(rd) |= 0xFFFF0000;
    } else if ((op & 0xF800) == 0x6000) { // str rd, [rb, imm5]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        arm7_write(reg(rb) + (imm5 << 2), reg(rd));
    } else if ((op & 0xF800) == 0x6800) { // ldr rd, [rb, imm5]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        reg(rd) = arm7_read(reg(rb) + (imm5 << 2));
    } else if ((op & 0xF800) == 0x7000) { // strb rd, [rb, imm5]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        arm7_writeb(reg(rb) + imm5, reg(rd));
    } else if ((op & 0xF800) == 0x7800) { // ldrb rd, [rb, imm5]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        reg(rd) = arm7_readb(reg(rb) + imm5);
    } else if ((op & 0xF800) == 0x8000) { // strh rd, [rb, imm5]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        arm7_writeh(reg(rb) + (imm5 << 1), reg(rd));
    } else if ((op & 0xF800) == 0x8800) { // ldrh rd, [rb, imm5]
        uint32_t rd = op & 7;
        uint32_t rb = (op >> 3) & 7;
        uint32_t imm5 = (op >> 6) & 31;
        reg(rd) = arm7_readh(reg(rb) + (imm5 << 1));
    } else if ((op & 0xF800) == 0x9000) { // str rd, [r13, imm8]
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        arm7_write(reg(13) + (imm8 << 2), reg(rd));
    } else if ((op & 0xF800) == 0x9800) { // ldr rd, [r13, imm8]
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        reg(rd) = arm7_read(reg(13) + (imm8 << 2));
    } else if ((op & 0xF800) == 0xA000) { // add rd, r15, imm8
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        reg(rd) = (r15 & 0xFFFFFFFD) + (imm8 << 2);
    } else if ((op & 0xF800) == 0xA800) { // add rd, r13, imm8
        uint32_t imm8 = op & 255;
        uint32_t rd = (op >> 8) & 7;
        reg(rd) = reg(13) + (imm8 << 2);
    } else if ((op & 0xFF80) == 0xB000) { // add r13, imm7
        uint32_t imm7 = op & 127;
        reg(13) += (imm7 << 2);
    } else if ((op & 0xFF80) == 0xB080) { // add r13, -imm7
        uint32_t imm7 = op & 127;
        reg(13) -= (imm7 << 2);
    } else if ((op & 0xFF00) == 0xB400) { // push {rlist}
        uint32_t rlist = op & 255;
        for (int i = 7; i >= 0; i--)
        {
            if (rlist & (1 << i))
            {
                reg(13) -= 4;
                arm7_write(reg(13), reg(i));
            }
        }
    } else if ((op & 0xFF00) == 0xB500) { // push {rlist, lr}
        uint32_t rlist = op & 255;
        reg(13) -= 4;
        arm7_write(reg(13), reg(14));
        for (int i = 7; i >= 0; i--)
        {
            if (rlist & (1 << i))
            {
                reg(13) -= 4;
                arm7_write(reg(13), reg(i));
            }
        }
    } else if ((op & 0xFF00) == 0xBC00) { // pop {rlist}
        uint32_t rlist = op & 255;
        for (int i = 7; i >= 0; i++)
        {
            if (rlist & (1 << i))
            {
                reg(i) = arm7_read(reg(13));
                reg(13) += 4;
            }
        }
    } else if ((op & 0xFF00) == 0xBD00) { // pop {rlist, pc}
        uint32_t rlist = op & 255;
        for (int i = 7; i >= 0; i++)
        {
            if (rlist & (1 << i))
            {
                reg(i) = arm7_read(reg(13));
                reg(13) += 4;
            }
        }
        r15 = arm7_read(reg(13));
        branched = true;
        pipe_state = 0; // Clear the pipeline!!!!111elf
    } else if ((op & 0xF800) == 0xC000) { // stmia rb!, {rlist}
        uint32_t rlist = op & 255;
        uint32_t rb = (op >> 8) & 7;
        for (int i = 7; i >= 0; i--)
        {
            if (rlist & (1 << i))
            {
                arm7_write(reg(rb), reg(i));
                reg(rb) += 4;
            }
        }
    } else if ((op & 0xF800) == 0xC800) { // ldmia rb!, {rlist}
        uint32_t rlist = op & 255;
        uint32_t rb = (op >> 8) & 7;
        for (int i = 7; i >= 0; i--)
        {
            if (rlist & (1 << i))
            {
                reg(i) = arm7_read(reg(rb));
                reg(rb) += 4;
            }
        }
    } else if ((op & 0xFF00) == 0xD000) { // beq label
        uint32_t simm8 = op & 255;
        if (BZERO)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD100) { // bne label
        uint32_t simm8 = op & 255;
        if (!BZERO)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD200) { // bcs label
        uint32_t simm8 = op & 255;
        if (BCARRY)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD300) { // bcc label
        uint32_t simm8 = op & 255;
        if (!CARRY)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD400) { // bmi label
        uint32_t simm8 = op & 255;
        if (BSIGN)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD500) { // bpl label
        uint32_t simm8 = op & 255;
        if (!BSIGN)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD600) { // bvs label
        uint32_t simm8 = op & 255;
        if (BOVERFLOW)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD700) { // bvc label
        uint32_t simm8 = op & 255;
        if (!BOVERFLOW)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD800) { // bhi label
        uint32_t simm8 = op & 255;
        if (BCARRY && !BZERO)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xD900) { // bls label
        uint32_t simm8 = op & 255;
        if (!BCARRY && BZERO)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xDA00) { // bge label
        uint32_t simm8 = op & 255;
        if (BSIGN == BOVERFLOW)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xDB00) { // blt label
        uint32_t simm8 = op & 255;
        if ( (BSIGN && !BOVERFLOW) || (!BSIGN && BOVERFLOW) )
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xDC00) { // bgt label
        uint32_t simm8 = op & 255;
        if (!BZERO && BSIGN == BOVERFLOW )
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xDD00) { // ble label
        uint32_t simm8 = op & 255;
        if (BZERO || BSIGN != BOVERFLOW)
        {
            if (simm8 & 0x80) simm8 |= 0xFFFFFF00;
            r15 += (simm8 << 1);
            branched = true;
            pipe_state = 0;
        }
    } else if ((op & 0xFF00) == 0xDF00) { // swi
        reg(14) = r15 - 2;
        spsr_svc = cpsr;
        r15 = 0x8;
        cpsr &= 0xFFFFFFDF;
        cpsr &= 0xFFFFFFE0;
        cpsr |= 0x13;
        arm7_update_regs();
        branched = true;
        pipe_state = 0;
    } else if ((op & 0xF800) == 0xE000) { // b label
        uint32_t imm11 = op & 2047;
        r15 += imm11 << 1;
        branched = true;
        pipe_state = 0;
    } else if ((op & 0xF800) == 0xF000) { // bl imm11
        uint32_t imm11 = op & 2047;
        reg(14) = r15 + (imm11 << 12);
    } else if ((op & 0xF800) == 0xF800) { // bh imm11
        uint32_t imm11 = op & 2047;
        uint32_t temp = r15 - 2;
        r15 = reg(14) + (imm11 << 1);
        reg(14) = temp | 1;
        branched = true;
        pipe_state = 0;
    } else {
        printf("Unknown instruction at PC=%x", r15 - 4);
	}

    arm7_regdump();
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

    if (cond_given)
    {
        if ((op & 0x0FFFFFF0) == 0x012FFF10) { // Branch and Exchange (BX)
        	uint32_t rn = op & 0xF;
        	if (reg(rn) & 1) // if mode switch to thumb
        	{
        		cpsr |= 0b100000;
        		r15 = reg(rn) & ~(1);
        	} else { // stay in arm mode
        		r15 = reg(rn) & ~(3);
        	}
        	// check if pipe is really flushed?
        	pipe_state = 0;
        	branched = true;
        } else if ((op & 0x0E000000) == 0x0A000000) { // Branch and Branch with Link
        	uint32_t offset = op & 0xFFFFFF;
        	if (offset & 0x800000) // check if sign extend is needed
        	{
        		offset |= 0xFF000000;
        	}
        	if (op & 0x01000000) { // with link
        		reg(14) = r15 - 4;
        	}
        	r15 += offset << 2;
        	pipe_state = 0;
        	branched = true;
        } 
    }
        
    arm7_regdump();
    char test[1337];
    gets(test);
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

