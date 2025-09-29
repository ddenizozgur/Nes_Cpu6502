#if !defined(_NES_CPU6502_H)
#define _NES_CPU6502_H

#include "Canvas.h"

#define STKP_OFFSET 0x100

#define INTR_VECTOR_NMI_L   0xfffa
#define INTR_VECTOR_NMI_H   0xfffb

#define INTR_VECTOR_RST_L   0xfffc
#define INTR_VECTOR_RST_H   0xfffd

#define INTR_VECTOR_IRQ_L   0xfffe
#define INTR_VECTOR_IRQ_H   0xffff

typedef enum {
    FLAG_N = 0x01,
    FLAG_V = 0x02,
    FLAG_UNUSED = 0x04,
    FLAG_B = 0x08,
    FLAG_D = 0x10,  // NOTE: !!! unused for NES !!!
    FLAG_I = 0x20,
    FLAG_Z = 0x40,
    FLAG_C = 0x80,
} FLAG_;

typedef struct {
    uint8_t  acc, x, y, stkp, stat;
    uint16_t pc;

    uint64_t clk_count;
} Nes_Cpu6502;

Nes_Cpu6502 nes_cpu6502_init(uint16_t rst_addr) {
    return (Nes_Cpu6502) {
        .stkp = 0xff,
        .stat = FLAG_UNUSED,
        .pc = rst_addr,
    };
}

INLINE void nes_cpu6502_set_flag(Nes_Cpu6502 *cpu, FLAG_ flag, bool bit) {
    if (bit) cpu->stat |= flag;
    else     cpu->stat &= ~flag;
}

INLINE bool nes_cpu6502_get_flag(Nes_Cpu6502 *cpu, FLAG_ flag) {
    return cpu->stat & flag;
}

void memory_init(uint8_t *memory, uint16_t rst_addr, uint16_t irq_addr, uint16_t nmi_addr) {
    memory[INTR_VECTOR_RST_L] = rst_addr & 0xff;
    memory[INTR_VECTOR_RST_H] = (rst_addr >> 8) & 0xff;

    memory[INTR_VECTOR_IRQ_L] = irq_addr & 0xff;
    memory[INTR_VECTOR_IRQ_H] = (irq_addr >> 8) & 0xff;

    memory[INTR_VECTOR_NMI_L] = nmi_addr & 0xff;
    memory[INTR_VECTOR_NMI_H] = (nmi_addr >> 8) & 0xff;
}

INLINE uint8_t memory_read_byte(uint8_t *memory, uint16_t addr) {
    if (addr < 0x2000) {
// $0000 - $00ff :: zero page
// $0100 - $01ff :: stack
// $0200 - $07ff :: ram
// $0800 - $1fff :: mirrors ($0000 - $07ff)
        return memory[addr & 0x7ff];
    }
    if (addr < 0x4000) {
// $2000 - $2007 :: io regs
// $2008 - $3fff :: mirrors ($2000 - $2007)
        uint16_t reg = 0x2000 + (addr & 0x7);
        return 0;   // TODO: ppu regs
    }
    if (addr < 0x4020) {
// $4000 - $4020 :: io regs
        return 0;   // TODO: io regs
    }
    if (addr < 0x8000)
        return 0;   // TODO: ???

    return memory[addr];
}
INLINE void memory_write_byte(uint8_t *memory, uint16_t addr, uint8_t byte) {
    if (addr < 0x2000) {
        memory[addr & 0x7ff] = byte;
        return;
    }
    if (addr < 0x4000) {
        uint16_t reg = 0x2000 + (addr & 0x7);
        return;     // TODO: ppu regs
    }
    if (addr < 0x4020)
        return;     // TODO: io
    if (addr < 0x8000)
        return;     // TODO: ???

    memory[addr] = byte;
}

INLINE void nes_cpu6502_stack_push(Nes_Cpu6502 *cpu, uint8_t *memory, uint8_t byte) {
    memory_write_byte(memory, STKP_OFFSET + cpu->stkp--, byte);
}
INLINE uint8_t nes_cpu6502_stack_pop(Nes_Cpu6502 *cpu, uint8_t *memory) {
    return memory_read_byte(memory, STKP_OFFSET + ++cpu->stkp);
}

//
// INTERRUPTS
//

void nes_cpu6502_intr_rst(Nes_Cpu6502 *cpu, uint8_t *memory) {
    uint8_t l = memory_read_byte(memory, INTR_VECTOR_RST_L);
    uint8_t h = memory_read_byte(memory, INTR_VECTOR_RST_H);
    cpu->pc = (h << 8) | l;

    cpu->acc = cpu->x = cpu->y = cpu->clk_count = 0;
    cpu->stkp = 0xff;
    cpu->stat = FLAG_UNUSED;
}

void nes_cpu6502_intr_irq(Nes_Cpu6502 *cpu, uint8_t *memory) {
    if (!nes_cpu6502_get_flag(cpu, FLAG_I))
        return;

    nes_cpu6502_stack_push(cpu, memory, (cpu->pc >> 8) & 0xff);
    nes_cpu6502_stack_push(cpu, memory, cpu->pc & 0xff);

    nes_cpu6502_set_flag(cpu, FLAG_B, 0);
    nes_cpu6502_stack_push(cpu, memory, cpu->stat);

    uint8_t l = memory_read_byte(memory, INTR_VECTOR_IRQ_L);
    uint8_t h = memory_read_byte(memory, INTR_VECTOR_IRQ_H);
    cpu->pc = (h << 8) | l;
}

void nes_cpu6502_intr_nmi(Nes_Cpu6502 *cpu, uint8_t *memory) {
    nes_cpu6502_stack_push(cpu, memory, (cpu->pc >> 8) & 0xff);
    nes_cpu6502_stack_push(cpu, memory, cpu->pc & 0xff);

    nes_cpu6502_set_flag(cpu, FLAG_B, 0);
    nes_cpu6502_stack_push(cpu, memory, cpu->stat);

    uint8_t l = memory_read_byte(memory, INTR_VECTOR_NMI_L);
    uint8_t h = memory_read_byte(memory, INTR_VECTOR_NMI_H);
    cpu->pc = (h << 8) | l;
}

//
// ADDRESSING MODES
//

uint16_t nes_cpu6502_get_addr_zp0(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: zero-page addressing
    return memory_read_byte(memory, cpu->pc++);
}

uint16_t nes_cpu6502_get_addr_zpx(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: x-indexed zero-page addressing
    uint16_t temp = memory_read_byte(memory, cpu->pc++);
    return (temp + cpu->x) & 0xff;
}

uint16_t nes_cpu6502_get_addr_zpy(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: y-indexed zero-page addressing
    uint16_t temp = memory_read_byte(memory, cpu->pc++);
    return (temp + cpu->y) & 0xff;
}

uint16_t nes_cpu6502_get_addr_abs(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: absolute addressing
    uint8_t l = memory_read_byte(memory, cpu->pc++);
    uint8_t h = memory_read_byte(memory, cpu->pc++);
    return (h << 8) | l;
}

uint16_t nes_cpu6502_get_addr_absx(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: x-indexed absolute addressing
    uint8_t l = memory_read_byte(memory, cpu->pc++);
    uint8_t h = memory_read_byte(memory, cpu->pc++);
    uint16_t addr = (h << 8) | l;
    return addr + cpu->x;
}

uint16_t nes_cpu6502_get_addr_absy(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: y-indexed absolute addressing
    uint8_t l = memory_read_byte(memory, cpu->pc++);
    uint8_t h = memory_read_byte(memory, cpu->pc++);
    uint16_t addr = (h << 8) | l;
    return addr + cpu->y;
}

uint16_t nes_cpu6502_get_addr_ind(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: indirect addressing
    uint8_t temp_l = memory_read_byte(memory, cpu->pc++);
    uint8_t temp_h = memory_read_byte(memory, cpu->pc++);
    uint16_t temp_addr = (temp_h << 8) | temp_l;

    uint8_t l = memory_read_byte(memory, temp_addr);
    uint16_t page_bug = (temp_addr & 0xff00) | ((temp_addr + 1) & 0xff);
    uint8_t h = memory_read_byte(memory, page_bug);
    return (h << 8) | l;
}

uint16_t nes_cpu6502_get_addr_indx(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: x-indexed indirect addressing
    uint16_t temp_addr = memory_read_byte(memory, cpu->pc++);

    uint8_t l = memory_read_byte(memory, (temp_addr + cpu->x) & 0xff);
    uint8_t h = memory_read_byte(memory, (temp_addr + 1 + cpu->x) & 0xff);
    return (h << 8) | l;
}

uint16_t nes_cpu6502_get_addr_indy(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: y-indexed indirect addressing
    uint16_t temp_addr = memory_read_byte(memory, cpu->pc++);

    uint8_t l = memory_read_byte(memory, temp_addr);
    uint8_t h = memory_read_byte(memory, (temp_addr + 1) & 0xff);
    uint16_t addr = (h << 8) | l;
    return addr + cpu->y;
}

uint16_t nes_cpu6502_get_addr_rel(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: relative addressing !!! implemented in instructions !!!
    return memory_read_byte(memory, cpu->pc++);
}

uint16_t nes_cpu6502_get_addr_impl(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: implied addressing !!! instructions such not require memory access !!!
    return 0;
}

uint16_t nes_cpu6502_get_addr_acc(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: accumulator addressing !!! instructions such not require memory access !!!
    return 0;
}

uint16_t nes_cpu6502_get_addr_imm(Nes_Cpu6502 *cpu, uint8_t *memory) {
// NOTE: immediate addressing
    return cpu->pc++;
}

//
// INSTRUCTIONS
//

void nes_cpu6502_do_instr_lda(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: load accumulator from memory
    uint8_t fetched = memory_read_byte(memory, addr);

    nes_cpu6502_set_flag(cpu, FLAG_N, fetched & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, fetched == 0);
    cpu->acc = fetched;
}

void nes_cpu6502_do_instr_ldx(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: load x-index from memory
    uint8_t fetched = memory_read_byte(memory, addr);

    nes_cpu6502_set_flag(cpu, FLAG_N, fetched & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, fetched == 0);
    cpu->x = fetched;
}

void nes_cpu6502_do_instr_ldy(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: load y-index from memory
    uint8_t fetched = memory_read_byte(memory, addr);

    nes_cpu6502_set_flag(cpu, FLAG_N, fetched & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, fetched == 0);
    cpu->y = fetched;
}

void nes_cpu6502_do_instr_sta(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: store accumulator in memory
    memory_write_byte(memory, addr, cpu->acc);
}

void nes_cpu6502_do_instr_stx(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: store x-index in memory
    memory_write_byte(memory, addr, cpu->x);
}

void nes_cpu6502_do_instr_sty(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: store y-index in memory
    memory_write_byte(memory, addr, cpu->y);
}

void nes_cpu6502_do_instr_tax(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: transfer accumulator to x-index
    uint8_t temp = cpu->acc;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->x = temp;
}

void nes_cpu6502_do_instr_tay(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: transfer accumulator to y-index
    uint8_t temp = cpu->acc;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->y = temp;
}

void nes_cpu6502_do_instr_tsx(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: transfer stack pointer to x-index
    uint8_t temp = cpu->stkp;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->x = temp;
}

void nes_cpu6502_do_instr_txa(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: transfer x-index to accumulator
    uint8_t temp = cpu->x;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_txs(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: transfer x-index to stack pointer
    cpu->stkp = cpu->x;
}

void nes_cpu6502_do_instr_tya(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: transfer y-index to accumulator
    uint8_t temp = cpu->y;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_pha(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: push accumulator on stack
    nes_cpu6502_stack_push(cpu, memory, cpu->acc);
}

void nes_cpu6502_do_instr_php(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: push processor status on stack
    nes_cpu6502_stack_push(cpu, memory, cpu->stat);
}

void nes_cpu6502_do_instr_pla(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: pop accumulator from stack
    uint8_t temp = nes_cpu6502_stack_pop(cpu, memory);

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_plp(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: pop processor status from stack
    uint8_t temp = nes_cpu6502_stack_pop(cpu, memory);

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->stat = temp;
}

void nes_cpu6502_do_instr_asl_acc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: arithmetic shift left (accumulator)
    uint8_t temp = cpu->acc << 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->acc & 0x80);
    cpu->acc = temp;
}
void nes_cpu6502_do_instr_asl_mem(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: arithmetic shift left (memory)
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched << 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, fetched & 0x80);
    memory_write_byte(memory, addr, temp);
}

void nes_cpu6502_do_instr_lsr_acc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: logical shift right (accumulator)
    uint8_t temp = cpu->acc >> 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, 0);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->acc & 0x1);
    cpu->acc = temp;
}
void nes_cpu6502_do_instr_lsr_mem(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: logical shift right (memory)
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched >> 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, 0);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, fetched & 0x1);
    memory_write_byte(memory, addr, temp);
}

void nes_cpu6502_do_instr_rol_acc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: rotate left (accumulator)
    uint8_t temp = cpu->acc << 1;
    temp |= nes_cpu6502_get_flag(cpu, FLAG_C) & 0x1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->acc & 0x80);
    cpu->acc = temp;
}
void nes_cpu6502_do_instr_rol_mem(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: rotate left (memory)
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched << 1;
    temp |= nes_cpu6502_get_flag(cpu, FLAG_C) & 0x1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, fetched & 0x80);
    memory_write_byte(memory, addr, temp);
}

void nes_cpu6502_do_instr_ror_acc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: rotate right (accumulator)
    uint8_t temp = cpu->acc >> 1;
    temp |= (nes_cpu6502_get_flag(cpu, FLAG_C) & 0x1) << 7;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->acc & 0x1);
    cpu->acc = temp;
}
void nes_cpu6502_do_instr_ror_mem(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: rotate right (memory)
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched >> 1;
    temp |= (nes_cpu6502_get_flag(cpu, FLAG_C) & 0x1) << 7;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, fetched & 0x1);
    memory_write_byte(memory, addr, temp);
}

void nes_cpu6502_do_instr_and(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: and memory with accumulator
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched & cpu->acc;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_bit(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: bit test memory with accumulator
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched & cpu->acc;

    nes_cpu6502_set_flag(cpu, FLAG_N, fetched & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_V, fetched & 0x40);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
}

void nes_cpu6502_do_instr_eor(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: xor memory with accumulator
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched ^ cpu->acc;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_ora(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: or memory with accumulator
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched | cpu->acc;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_adc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: add memory to accumulator with carry
    uint8_t fetched = memory_read_byte(memory, addr);
    uint16_t temp = (uint16_t)cpu->acc + fetched + nes_cpu6502_get_flag(cpu, FLAG_C);

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);

    bool sign_l = cpu->acc & 0x80;
    bool sign_r = fetched & 0x80;
    bool overflow = (sign_l == sign_r) && (sign_l != (temp & 0x80));
    nes_cpu6502_set_flag(cpu, FLAG_V, overflow);

    nes_cpu6502_set_flag(cpu, FLAG_Z, (temp & 0xff) == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, temp > 0xff);
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_cmp(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: compare memory and accumulator
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = cpu->acc - fetched;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->acc >= fetched);
}

void nes_cpu6502_do_instr_cpx(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: compare x-index to memory
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = cpu->x - fetched;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->x >= fetched);
}

void nes_cpu6502_do_instr_cpy(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: compare y-index to memory
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = cpu->y - fetched;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, cpu->y >= fetched);
}

void nes_cpu6502_do_instr_sbc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: substract memory from accumulator with borrow
    uint8_t fetched = memory_read_byte(memory, addr);
    uint16_t temp = (uint16_t)cpu->acc - fetched - !nes_cpu6502_get_flag(cpu, FLAG_C);

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);

    bool sign_l = cpu->acc & 0x80;
    bool sign_r = fetched & 0x80;                                       //
    bool overflow = (sign_l != sign_r) && (sign_l != (temp & 0x80));    // TODO: !!! check overflow logic !!!
    nes_cpu6502_set_flag(cpu, FLAG_V, overflow);                        //

    nes_cpu6502_set_flag(cpu, FLAG_Z, (temp & 0xff) == 0);
    nes_cpu6502_set_flag(cpu, FLAG_C, !(temp & 0x100));
    cpu->acc = temp;
}

void nes_cpu6502_do_instr_dec(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: decrement memory by one
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched - 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    memory_write_byte(memory, addr, temp);
}

void nes_cpu6502_do_instr_dex(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: decrement x-index by one
    uint8_t temp = cpu->x - 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->x = temp;
}

void nes_cpu6502_do_instr_dey(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: decrement y-index by one
    uint8_t temp = cpu->y - 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->y = temp;
}

void nes_cpu6502_do_instr_inc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: increment memory by one
    uint8_t fetched = memory_read_byte(memory, addr);
    uint8_t temp = fetched + 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    memory_write_byte(memory, addr, temp);
}

void nes_cpu6502_do_instr_inx(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: increment x-index by one
    uint8_t temp = cpu->x + 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->x = temp;
}

void nes_cpu6502_do_instr_iny(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: increment y-index by one
    uint8_t temp = cpu->y + 1;

    nes_cpu6502_set_flag(cpu, FLAG_N, temp & 0x80);
    nes_cpu6502_set_flag(cpu, FLAG_Z, temp == 0);
    cpu->y = temp;
}

void nes_cpu6502_do_instr_brk(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: break interrupt
    nes_cpu6502_set_flag(cpu, FLAG_I, 1);

    uint16_t push_pc = cpu->pc + 2;
    nes_cpu6502_stack_push(cpu, memory, (push_pc >> 8) & 0xff);
    nes_cpu6502_stack_push(cpu, memory, push_pc & 0xff);

    nes_cpu6502_set_flag(cpu, FLAG_B, 1);
    nes_cpu6502_stack_push(cpu, memory, cpu->stat);
    nes_cpu6502_set_flag(cpu, FLAG_B, 0);

    uint8_t l = memory_read_byte(memory, INTR_VECTOR_IRQ_L);
    uint8_t h = memory_read_byte(memory, INTR_VECTOR_IRQ_H);
    cpu->pc = (h << 8) | l;
}

void nes_cpu6502_do_instr_jmp(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: jump indirect
    cpu->pc = addr;
}

void nes_cpu6502_do_instr_jsr(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: jump to subroutine
    uint16_t push_pc = cpu->pc + 2;
    nes_cpu6502_stack_push(cpu, memory, (push_pc >> 8) & 0xff);
    nes_cpu6502_stack_push(cpu, memory, push_pc & 0xff);

    cpu->pc = addr;
}

void nes_cpu6502_do_instr_rti(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: return from interrupt
    cpu->stat = nes_cpu6502_stack_pop(cpu, memory);

    uint8_t l = nes_cpu6502_stack_pop(cpu, memory);
    uint8_t h = nes_cpu6502_stack_pop(cpu, memory);
    cpu->pc = (h << 8) | l;
}

void nes_cpu6502_do_instr_rts(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// TODO: !!! check logic !!!
// NOTE: return from subroutine
    uint8_t l = nes_cpu6502_stack_pop(cpu, memory);
    uint8_t h = nes_cpu6502_stack_pop(cpu, memory);
    cpu->pc = (h << 8) | l;
}

void nes_cpu6502_do_instr_bcc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on carry clear
    if (!nes_cpu6502_get_flag(cpu, FLAG_C))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_bcs(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on carry set
    if (nes_cpu6502_get_flag(cpu, FLAG_C))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_beq(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on result zero
    if (nes_cpu6502_get_flag(cpu, FLAG_Z))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_bmi(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on result negative
    if (nes_cpu6502_get_flag(cpu, FLAG_N))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_bne(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on result not zero
    if (!nes_cpu6502_get_flag(cpu, FLAG_Z))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_bpl(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on result positive
    if (!nes_cpu6502_get_flag(cpu, FLAG_N))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_bvc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on overflow clear
    if (!nes_cpu6502_get_flag(cpu, FLAG_V))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_bvs(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: branch on overflow set
    if (nes_cpu6502_get_flag(cpu, FLAG_V))
        cpu->pc += (char)addr;
}

void nes_cpu6502_do_instr_clc(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: clear carry flag
    nes_cpu6502_set_flag(cpu, FLAG_C, 0);
}

void nes_cpu6502_do_instr_cld(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: clear decimal flag !!! unused for nes (maybe) !!!
    nes_cpu6502_set_flag(cpu, FLAG_D, 0);
}

void nes_cpu6502_do_instr_cli(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: clear interrupt disable
    nes_cpu6502_set_flag(cpu, FLAG_I, 0);
}

void nes_cpu6502_do_instr_clv(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: clear overflow flag
    nes_cpu6502_set_flag(cpu, FLAG_V, 0);
}

void nes_cpu6502_do_instr_sec(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: set carry flag
    nes_cpu6502_set_flag(cpu, FLAG_C, 1);
}

void nes_cpu6502_do_instr_sed(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: set decimal flag !!! unused for nes (maybe) !!!
    nes_cpu6502_set_flag(cpu, FLAG_D, 1);
}

void nes_cpu6502_do_instr_sei(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: set interrupt disable
    nes_cpu6502_set_flag(cpu, FLAG_I, 1);
}

void nes_cpu6502_do_instr_nop(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: no operation
}

void nes_cpu6502_do_instr_xxx(Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
// NOTE: undocumented intructions
}

typedef struct {
    const char *operation_tag;
    uint16_t (*addr_mode) (Nes_Cpu6502 *, uint8_t *);
    void (*operation) (Nes_Cpu6502 *, uint8_t *, uint16_t);
    uint32_t cycles;
} Nes_Cpu6502_Instr;

const Nes_Cpu6502_Instr instr_lookup_table[] = {
    { "brk", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_brk, 7 },
    { "ora", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_ora, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 3 },
    { "ora", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_ora, 3 },
    { "asl", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_asl_mem, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "php", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_php, 3 },
    { "ora", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_ora, 2 },
    { "asl", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_asl_acc, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "ora", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_ora, 4 },
    { "asl", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_asl_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "bpl", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bpl, 2 },
    { "ora", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_ora, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "ora", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_ora, 4 },
    { "asl", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_asl_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "clc", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_clc, 2 },
    { "ora", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_ora, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "ora", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_ora, 4 },
    { "asl", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_asl_mem, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "jsr", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_jsr, 6 },
    { "and", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_and, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "bit", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_bit, 3 },
    { "and", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_and, 3 },
    { "rol", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_rol_mem, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "plp", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_plp, 4 },
    { "and", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_and, 2 },
    { "rol", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_rol_acc, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "bit", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_bit, 4 },
    { "and", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_and, 4 },
    { "rol", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_rol_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "bmi", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bmi, 2 },
    { "and", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_and, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "and", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_and, 4 },
    { "rol", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_rol_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "sec", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_sec, 2 },
    { "and", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_and, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "and", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_and, 4 },
    { "rol", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_rol_mem, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "rti", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_rti, 6 },
    { "eor", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_eor, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 3 },
    { "eor", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_eor, 3 },
    { "lsr", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_lsr_mem, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "pha", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_pha, 3 },
    { "eor", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_eor, 2 },
    { "lsr", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_lsr_acc, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "jmp", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_jmp, 3 },
    { "eor", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_eor, 4 },
    { "lsr", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_lsr_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "bvc", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bvc, 2 },
    { "eor", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_eor, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "eor", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_eor, 4 },
    { "lsr", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_lsr_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "cli", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_cli, 2 },
    { "eor", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_eor, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "eor", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_eor, 4 },
    { "lsr", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_lsr_mem, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "rts", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_rts, 6 },
    { "adc", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_adc, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 3 },
    { "adc", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_adc, 3 },
    { "ror", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_ror_mem, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "pla", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_pla, 4 },
    { "adc", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_adc, 2 },
    { "ror", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_ror_acc, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "jmp", nes_cpu6502_get_addr_ind , nes_cpu6502_do_instr_jmp, 5 },
    { "adc", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_adc, 4 },
    { "ror", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_ror_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "bvs", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bvs, 2 },
    { "adc", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_adc, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "adc", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_adc, 4 },
    { "ror", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_ror_mem, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "sei", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_sei, 2 },
    { "adc", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_adc, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "adc", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_adc, 4 },
    { "ror", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_ror_mem, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "sta", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_sta, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "sty", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_sty, 3 },
    { "sta", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_sta, 3 },
    { "stx", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_stx, 3 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 3 },
    { "dey", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_dey, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "txa", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_txa, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "sty", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_sty, 4 },
    { "sta", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_sta, 4 },
    { "stx", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_stx, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 4 },
    { "bcc", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bcc, 2 },
    { "sta", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_sta, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "sty", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_sty, 4 },
    { "sta", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_sta, 4 },
    { "stx", nes_cpu6502_get_addr_zpy , nes_cpu6502_do_instr_stx, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 4 },
    { "tya", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_tya, 2 },
    { "sta", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_sta, 5 },
    { "txs", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_txs, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 5 },
    { "sta", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_sta, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "ldy", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_ldy, 2 },
    { "lda", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_lda, 6 },
    { "ldx", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_ldx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "ldy", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_ldy, 3 },
    { "lda", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_lda, 3 },
    { "ldx", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_ldx, 3 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 3 },
    { "tay", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_tay, 2 },
    { "lda", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_lda, 2 },
    { "tax", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_tax, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "ldy", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_ldy, 4 },
    { "lda", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_lda, 4 },
    { "ldx", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_ldx, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 4 },
    { "bcs", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bcs, 2 },
    { "lda", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_lda, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "ldy", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_ldy, 4 },
    { "lda", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_lda, 4 },
    { "ldx", nes_cpu6502_get_addr_zpy , nes_cpu6502_do_instr_ldx, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 4 },
    { "clv", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_clv, 2 },
    { "lda", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_lda, 4 },
    { "tsx", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_tsx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 4 },
    { "ldy", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_ldy, 4 },
    { "lda", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_lda, 4 },
    { "ldx", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_ldx, 4 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 4 },
    { "cpy", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_cpy, 2 },
    { "cmp", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_cmp, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "cpy", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_cpy, 3 },
    { "cmp", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_cmp, 3 },
    { "dec", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_dec, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "iny", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_iny, 2 },
    { "cmp", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_cmp, 2 },
    { "dex", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_dex, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "cpy", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_cpy, 4 },
    { "cmp", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_cmp, 4 },
    { "dec", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_dec, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "bne", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_bne, 2 },
    { "cmp", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_cmp, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "cmp", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_cmp, 4 },
    { "dec", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_dec, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "cld", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_cld, 2 },
    { "cmp", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_cmp, 4 },
    { "nop", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "cmp", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_cmp, 4 },
    { "dec", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_dec, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "cpx", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_cpx, 2 },
    { "sbc", nes_cpu6502_get_addr_indx, nes_cpu6502_do_instr_sbc, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "cpx", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_cpx, 3 },
    { "sbc", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_sbc, 3 },
    { "inc", nes_cpu6502_get_addr_zp0 , nes_cpu6502_do_instr_inc, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 5 },
    { "inx", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_inx, 2 },
    { "sbc", nes_cpu6502_get_addr_imm , nes_cpu6502_do_instr_sbc, 2 },
    { "nop", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_sbc, 2 },
    { "cpx", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_cpx, 4 },
    { "sbc", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_sbc, 4 },
    { "inc", nes_cpu6502_get_addr_abs , nes_cpu6502_do_instr_inc, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "beq", nes_cpu6502_get_addr_rel , nes_cpu6502_do_instr_beq, 2 },
    { "sbc", nes_cpu6502_get_addr_indy, nes_cpu6502_do_instr_sbc, 5 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 8 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "sbc", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_sbc, 4 },
    { "inc", nes_cpu6502_get_addr_zpx , nes_cpu6502_do_instr_inc, 6 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 6 },
    { "sed", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_sed, 2 },
    { "sbc", nes_cpu6502_get_addr_absy, nes_cpu6502_do_instr_sbc, 4 },
    { "nop", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 2 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_nop, 4 },
    { "sbc", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_sbc, 4 },
    { "inc", nes_cpu6502_get_addr_absx, nes_cpu6502_do_instr_inc, 7 },
    { "???", nes_cpu6502_get_addr_impl, nes_cpu6502_do_instr_xxx, 7 },
};

void nes_cpu6502_step(Nes_Cpu6502 *cpu, uint8_t *memory) {
    uint8_t  opcode = memory_read_byte(memory, cpu->pc++);

    uint32_t cycles = instr_lookup_table[opcode].cycles;
    cpu->clk_count += cycles;

    uint16_t addr = instr_lookup_table[opcode].addr_mode(cpu, memory);
    instr_lookup_table[opcode].operation(cpu, memory, addr);
}

//
// DEBUG UTILITIES
//

void draw_nes_cpu6502_regs(Canvas *canvas, int x, int y, Nes_Cpu6502 *cpu) {
    int x0 = x;

    canvas->color = CANVAS_WHITE;
    canvas_printf(canvas, x, y, " acc: %02x", cpu->acc);
    y += CANVAS_FONT_HEIGHT;
    canvas_printf(canvas, x, y, "   x: %02x", cpu->x);
    y += CANVAS_FONT_HEIGHT;
    canvas_printf(canvas, x, y, "   y: %02x", cpu->y);
    y += CANVAS_FONT_HEIGHT;

    canvas->color = CANVAS_LTPURPLE;
    canvas_printf(canvas, x, y, "stkp: %02x", cpu->stkp);
    y += CANVAS_FONT_HEIGHT;

    {
        canvas->color = CANVAS_WHITE;
        canvas_printf(canvas, x, y, "stat:");
        x += 6 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_N) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "N");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_V) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "V");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_UNUSED) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "-");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_B) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "B");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_D) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "D");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_I) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "I");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_Z) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "Z");
        x += 2 * CANVAS_FONT_WIDTH;

        canvas->color = nes_cpu6502_get_flag(cpu, FLAG_C) ? CANVAS_LTGREEN : CANVAS_LTRED;
        canvas_printf(canvas, x, y, "C");
        x += 2 * CANVAS_FONT_WIDTH;
    }

    x = x0;
    y += CANVAS_FONT_HEIGHT;
    canvas->color = CANVAS_YELLOW;
    canvas_printf(canvas, x, y, "  pc: %04x", cpu->pc);
}

void draw_nes_cpu6502_memory(Canvas *canvas, int x, int y, int cols, int rows, Nes_Cpu6502 *cpu, uint8_t *memory, uint16_t addr) {
    int x0 = x;
    for (int j = 0; j < rows; j++) {
        canvas->color = CANVAS_WHITE;
        canvas_printf(canvas, x, y, "%04x:", addr);
        x += 6 * CANVAS_FONT_WIDTH;

        for (int i = 0; i < cols; i++) {
            uint8_t data = memory[addr];
            ABGR8888 color = data == 0 ? CANVAS_LTGRAY : CANVAS_WHITE;

            if (cpu->pc == addr) {
                canvas->color = CANVAS_YELLOW;
                canvas_fill_rect(canvas, x, y, 2 * CANVAS_FONT_WIDTH, CANVAS_FONT_HEIGHT);

                canvas->color = CANVAS_BLACK;
                canvas_printf(canvas, x, y, "%02x", data);
                x += 3 * CANVAS_FONT_WIDTH;
            } else if (STKP_OFFSET + cpu->stkp == addr) {
                canvas->color = CANVAS_LTPURPLE;
                canvas_fill_rect(canvas, x, y, 2 * CANVAS_FONT_WIDTH, CANVAS_FONT_HEIGHT);

                canvas->color = CANVAS_BLACK;
                canvas_printf(canvas, x, y, "%02x", data);
                x += 3 * CANVAS_FONT_WIDTH;
            } else {
                canvas->color = color;
                canvas_printf(canvas, x, y, "%02x", data);
                x += 3 * CANVAS_FONT_WIDTH;
            }

            addr++;
        }

        x = x0;
        y += CANVAS_FONT_HEIGHT;
    }
}

void draw_nes_cpu6502_disasm(Canvas *canvas, int x, int y, int lines, Nes_Cpu6502 *cpu, uint8_t *memory) {
    uint16_t current_pc = cpu->pc;

    for (uint16_t i = 0; i < lines; i++) {
        uint8_t opcode = memory_read_byte(memory, current_pc++);

        const char *tag = instr_lookup_table[opcode].operation_tag;
        canvas->color = i == 0 ? CANVAS_WHITE : CANVAS_LTGRAY;
        canvas_printf(canvas, x, y + i * CANVAS_FONT_HEIGHT, "%s", tag);

        uint16_t (*addr_mode) (Nes_Cpu6502 *, uint8_t *) = instr_lookup_table[opcode].addr_mode;
        if (addr_mode == nes_cpu6502_get_addr_zp0) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "$%02x\t/zp0", l);
        } else if (addr_mode == nes_cpu6502_get_addr_zpx) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "$%02x, x\t/zpx", l);
        } else if (addr_mode == nes_cpu6502_get_addr_zpy) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "$%02x, y\t/zpy", l);
        } else if (addr_mode == nes_cpu6502_get_addr_abs) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            uint8_t h = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "$%04x\t/abs", (h << 8) | l);
        } else if (addr_mode == nes_cpu6502_get_addr_absx) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            uint8_t h = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "$%04x, x\t/absx", (h << 8) | l);
        } else if (addr_mode == nes_cpu6502_get_addr_absy) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            uint8_t h = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "$%04x, y\t/absx", (h << 8) | l);
        } else if (addr_mode == nes_cpu6502_get_addr_ind) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            uint8_t h = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "($%04x)\t/ind", (h << 8) | l);
        } else if (addr_mode == nes_cpu6502_get_addr_indx) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "($%02x, x)\t/indx", l);
        } else if (addr_mode == nes_cpu6502_get_addr_indy) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "($%02x), y\t/indy", l);
        } else if (addr_mode == nes_cpu6502_get_addr_rel) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "[$%02x]\t/rel", l);
        } else if (addr_mode == nes_cpu6502_get_addr_impl) {
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "\t/impl");
        } else if (addr_mode == nes_cpu6502_get_addr_acc) {
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "\t/acc");
        } else if (addr_mode == nes_cpu6502_get_addr_imm) {
            uint8_t l = memory_read_byte(memory, current_pc++);
            canvas_printf(canvas, x + 4 * CANVAS_FONT_WIDTH, y + i * CANVAS_FONT_HEIGHT, "#$%02x\t/imm", l);
        } else {
            ASSERT(false, "@unreachable");
        }
    }
}

#endif  // _NES_CPU6502_H