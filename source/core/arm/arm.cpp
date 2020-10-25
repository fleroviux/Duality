/*
 * Copyright (C) 2020 fleroviux
 */

#include "arm.hpp"

namespace fauxDS::core::arm {

void ARM::Reset() {
  constexpr u32 nop = 0xE320F000;

  state.Reset();
  SwitchMode(MODE_SYS);
  opcode[0] = nop;
  opcode[1] = nop;
  state.r15 = ExceptionBase();

  for (auto coprocessor : coprocessors)
    if (coprocessor != nullptr)
      coprocessor->Reset();
}

void ARM::Run(int instructions) {
  while (instructions-- > 0) {
    auto instruction = opcode[0];

    if (state.cpsr.f.thumb) {
      state.r15 &= ~1;

      opcode[0] = opcode[1];
      opcode[1] = ReadHalfCode(state.r15);
      (this->*s_opcode_lut_16[instruction >> 5])(instruction);
    } else {
      state.r15 &= ~3;

      opcode[0] = opcode[1];
      opcode[1] = ReadWordCode(state.r15);
      auto condition = static_cast<Condition>(instruction >> 28);
      if (CheckCondition(condition)) {
        int hash = ((instruction >> 16) & 0xFF0) |
                   ((instruction >>  4) & 0x00F);
        if (condition == COND_NV) {
          hash |= 4096;
        }
        (this->*s_opcode_lut_32[hash])(instruction);
      } else {
        state.r15 += 4;
      }
    }
  }
}


void ARM::AttachCoprocessor(uint id, Coprocessor* coprocessor) {
  ASSERT(id <= 15, "Coprocessor ID must be lower or equal to 15");
  coprocessors[id] = coprocessor;
}

void ARM::SignalIRQ() {
  if (state.cpsr.f.mask_irq) {
    return;
  }

  if (state.cpsr.f.thumb) {
    /* Store return address in r14<irq>. */
    state.bank[BANK_IRQ][BANK_R14] = state.r15;

    /* Save program status and switch to IRQ mode. */
    state.spsr[BANK_IRQ].v = state.cpsr.v;
    SwitchMode(MODE_IRQ);
    state.cpsr.f.thumb = 0;
    state.cpsr.f.mask_irq = 1;
  } else {
    /* Store return address in r14<irq>. */
    state.bank[BANK_IRQ][BANK_R14] = state.r15 - 4;

    /* Save program status and switch to IRQ mode. */
    state.spsr[BANK_IRQ].v = state.cpsr.v;
    SwitchMode(MODE_IRQ);
    state.cpsr.f.mask_irq = 1;
  }

  /* Jump to exception vector. */
  state.r15 = ExceptionBase() + 0x18;
  ReloadPipeline32();
}

void ARM::ReloadPipeline32() {
  opcode[0] = ReadWordCode(state.r15);
  opcode[1] = ReadWordCode(state.r15 + 4);
  state.r15 += 8;
}

void ARM::ReloadPipeline16() {
  opcode[0] = ReadHalfCode(state.r15);
  opcode[1] = ReadHalfCode(state.r15 + 2);
  state.r15 += 4;
}

void ARM::BuildConditionTable() {
  for (int flags = 0; flags < 16; flags++) {
    bool n = flags & 8;
    bool z = flags & 4;
    bool c = flags & 2;
    bool v = flags & 1;

    condition_table[COND_EQ][flags] = z;
    condition_table[COND_NE][flags] = !z;
    condition_table[COND_CS][flags] =  c;
    condition_table[COND_CC][flags] = !c;
    condition_table[COND_MI][flags] =  n;
    condition_table[COND_PL][flags] = !n;
    condition_table[COND_VS][flags] =  v;
    condition_table[COND_VC][flags] = !v;
    condition_table[COND_HI][flags] =  c && !z;
    condition_table[COND_LS][flags] = !c ||  z;
    condition_table[COND_GE][flags] = n == v;
    condition_table[COND_LT][flags] = n != v;
    condition_table[COND_GT][flags] = !(z || (n != v));
    condition_table[COND_LE][flags] =  (z || (n != v));
    condition_table[COND_AL][flags] = true;
    condition_table[COND_NV][flags] = true;
  }
}

bool ARM::CheckCondition(Condition condition) {
  if (condition == COND_AL) {
    return true;
  }
  return condition_table[condition][state.cpsr.v >> 28];
}

auto ARM::GetRegisterBankByMode(Mode mode) -> Bank {
  switch (mode) {
    case MODE_USR:
    case MODE_SYS:
      return BANK_NONE;
    case MODE_FIQ:
      return BANK_FIQ;
    case MODE_IRQ:
      return BANK_IRQ;
    case MODE_SVC:
      return BANK_SVC;
    case MODE_ABT:
      return BANK_ABT;
    case MODE_UND:
      return BANK_UND;
  }

  UNREACHABLE;
}

void ARM::SwitchMode(Mode new_mode) {
  auto old_bank = GetRegisterBankByMode(state.cpsr.f.mode);
  auto new_bank = GetRegisterBankByMode(new_mode);

  state.cpsr.f.mode = new_mode;
  p_spsr = &state.spsr[new_bank];

  if (old_bank == new_bank) {
    return;
  }

  if (old_bank == BANK_FIQ || new_bank == BANK_FIQ) {
    for (int i = 0; i < 7; i++){
      state.bank[old_bank][i] = state.reg[8 + i];
    }

    for (int i = 0; i < 7; i++) {
      state.reg[8 + i] = state.bank[new_bank][i];
    }
  } else {
    state.bank[old_bank][5] = state.r13;
    state.bank[old_bank][6] = state.r14;

    state.r13 = state.bank[new_bank][5];
    state.r14 = state.bank[new_bank][6];
  }
}

} // namespace fauxDS::core::arm