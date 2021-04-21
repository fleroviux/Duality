/*
 * Copyright (C) 2020 fleroviux
 */

#include <stdexcept>
#include <xbyak/xbyak.h>

#include "arm.hpp"
#include "tablegen/decoder.hpp"

namespace Duality::Core::arm {

void ARM::Reset() {
  constexpr u32 nop = 0xE320F000;

  state.Reset();
  SwitchMode(state.cpsr.f.mode);
  opcode[0] = nop;
  opcode[1] = nop;
  state.r15 = ExceptionBase();
  wait_for_irq = false;
  IRQLine() = false;

  for (auto coprocessor : coprocessors)
    if (coprocessor != nullptr)
      coprocessor->Reset();

  block_cache = {};
}

void ARM::Run(int instructions) {
  using namespace Xbyak::util;

  if (IsWaitingForIRQ() && !IRQLine()) {
    return;
  }

  instructions += instruction_overshoot;

  while (instructions-- > 0) {
    if (IRQLine()) SignalIRQ();

    if (state.cpsr.f.thumb) {
      state.r15 &= ~1;

      u32 key = state.r15 | 1;

      if (auto match = block_cache.find(key); match != block_cache.end()) {
        match->second.fn();
        instructions -= match->second.instructions;
      } else {
        auto basic_block = BasicBlock{};
        auto code = new Xbyak::CodeGenerator{}; // leak goes brrr
        auto ip = state.r15 - 4;

        code->sub(rsp, 0x28);

        // NOTE: using 'instructions' number of instructions seem to cause a weird bug
        // in at least ARMWrestler and Pokémon Diamond.
        for (int i = 0; i < 32; i++) {
          auto opcode = ReadHalfCode(ip);
          auto opcode_type = GetThumbInstructionType(opcode);

          code->mov(rax, u64((const void*)s_opcode_lut_16[opcode >> 5]));
          code->mov(rcx, u64(this));
          code->mov(edx, opcode);
          code->call(rax);

          basic_block.instructions++;

          // Any thumb instruction that can potentially branch will break the basic block for now.
          if (opcode_type == ThumbInstrType::HighRegisterOps ||
              opcode_type == ThumbInstrType::PushPop ||
              opcode_type == ThumbInstrType::SoftwareInterrupt ||
              opcode_type == ThumbInstrType::ConditionalBranch ||
              opcode_type == ThumbInstrType::UnconditionalBranch ||
              opcode_type == ThumbInstrType::LongBranchLinkExchangeSuffix ||
              opcode_type == ThumbInstrType::LongBranchLinkPrefix ||
              opcode_type == ThumbInstrType::LongBranchLinkSuffix) {
            break;
          }

          ip += 2;
        }

        code->add(rsp, 0x28);
        code->ret();
        basic_block.fn = code->getCode<void (*)()>();
        block_cache[key] = basic_block;

        basic_block.fn();
        instructions -= basic_block.instructions;
      }
    } else {
      state.r15 &= ~3;

      u32 key = state.r15;

      if (auto match = block_cache.find(key); match != block_cache.end()) {
        auto const& basic_block = match->second;
        // TODO: with this approach we might have to stop the basic block
        // if CPSR changes during the block and the block is conditional.
        if (CheckCondition(basic_block.condition)) {
          basic_block.fn();
        } else {
          state.r15 += basic_block.instructions * sizeof(u32);
        }
        instructions -= basic_block.instructions;
      } else {
        auto basic_block = BasicBlock{};
        auto code = new Xbyak::CodeGenerator{}; // leak goes brrr
        auto ip = state.r15 - 8;

        code->sub(rsp, 0x28);

        for (int i = 0; i < 1; i++) {
          auto opcode = ReadWordCode(ip);
          auto condition = static_cast<Condition>(opcode >> 28);
          auto opcode_type = GetThumbInstructionType(opcode);

          int hash = ((opcode >> 16) & 0xFF0) |
                     ((opcode >>  4) & 0x00F);

          if (condition == COND_NV) {
            condition = COND_AL;
            hash |= 4096;
          }

          if (i == 0) {
            basic_block.condition = condition;
          } else if (condition != basic_block.condition) {
            break;
          }

          code->mov(rax, u64((const void*)s_opcode_lut_32[hash]));
          code->mov(rcx, u64(this));
          code->mov(edx, opcode);
          code->call(rax);

          basic_block.instructions++;

          // TODO: detect branches and abort the basic block.
          // Also abort the branch if CPSR flags change and the block conditional :methharold:

          ip += 4;
        }

        code->add(rsp, 0x28);
        code->ret();
        basic_block.fn = code->getCode<void (*)()>();
        block_cache[key] = basic_block;

        if (CheckCondition(basic_block.condition)) {
          basic_block.fn();
        } else {
          state.r15 += basic_block.instructions * sizeof(u32);
        }

        instructions -= basic_block.instructions;
      }

      // auto instruction = opcode[0];

      // state.r15 &= ~3;

      // opcode[0] = opcode[1];
      // opcode[1] = ReadWordCode(state.r15);
      // auto condition = static_cast<Condition>(instruction >> 28);

      // if (CheckCondition(condition)) {
      //   int hash = ((instruction >> 16) & 0xFF0) |
      //              ((instruction >>  4) & 0x00F);
      //   if (condition == COND_NV) {
      //     hash |= 4096;
      //   }
      //   (this->*s_opcode_lut_32[hash])(instruction);

      //   if (IsWaitingForIRQ()) return;
      // } else {
      //   state.r15 += 4;
      // }
    }

    instruction_overshoot = instructions;
  }
}

void ARM::InvalidateCodeCache() {
  block_cache = {};
}

void ARM::AttachCoprocessor(uint id, Coprocessor* coprocessor) {
  if (id >= 16) { 
    throw std::runtime_error{"Coprocessor ID must be lower or equal to 15"};
  }
  
  coprocessors[id] = coprocessor;
}

void ARM::SignalIRQ() {
  wait_for_irq = false;

  if (state.cpsr.f.mask_irq) {
    return;
  }

  // Save current program status register.
  state.spsr[BANK_IRQ].v = state.cpsr.v;

  // Enter IRQ mode and disable IRQs.
  SwitchMode(MODE_IRQ);
  state.cpsr.f.mask_irq = 1;

  // Save current program counter and disable Thumb.
  if (state.cpsr.f.thumb) {
    state.cpsr.f.thumb = 0;
    state.r14 = state.r15;
  } else {
    state.r14 = state.r15 - 4;
  }
  
  // Jump to IRQ exception vector.
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

} // namespace Duality::Core::arm
