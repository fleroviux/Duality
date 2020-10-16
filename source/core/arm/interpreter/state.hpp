/*
 * Copyright (C) 2020 fleroviux
 *
 * Licensed under GPLv3 or any later version.
 * Refer to the included LICENSE file.
 */

#pragma once

#include <cstdint>

namespace fauxDS::core::arm {

enum Mode : unsigned int {
  MODE_USR = 0x10,
  MODE_FIQ = 0x11,
  MODE_IRQ = 0x12,
  MODE_SVC = 0x13,
  MODE_ABT = 0x17,
  MODE_UND = 0x1B,
  MODE_SYS = 0x1F
};
  
enum Bank {
  BANK_NONE = 0,
  BANK_FIQ  = 1,
  BANK_SVC  = 2,
  BANK_ABT  = 3,
  BANK_IRQ  = 4,
  BANK_UND  = 5
};

enum Condition {
  COND_EQ = 0,
  COND_NE = 1,
  COND_CS = 2,
  COND_CC = 3,
  COND_MI = 4,
  COND_PL = 5,
  COND_VS = 6,
  COND_VC = 7,
  COND_HI = 8,
  COND_LS = 9,
  COND_GE = 10,
  COND_LT = 11,
  COND_GT = 12,
  COND_LE = 13,
  COND_AL = 14,
  COND_NV = 15
};

enum BankedRegister {
  BANK_R8  = 0,
  BANK_R9  = 1,
  BANK_R10 = 2,
  BANK_R11 = 3,
  BANK_R12 = 4,
  BANK_R13 = 5,
  BANK_R14 = 6
};

struct State {
  static constexpr int kBankCount = 6;

  union StatusRegister {
    struct {
      Mode mode : 5;
      unsigned int thumb : 1;
      unsigned int mask_fiq : 1;
      unsigned int mask_irq : 1;
      unsigned int reserved : 19;
      unsigned int q : 1;
      unsigned int v : 1;
      unsigned int c : 1;
      unsigned int z : 1;
      unsigned int n : 1;
    } f;
    std::uint32_t v;
  };
  
  // General Purpose Registers
  union {
    struct {
      std::uint32_t r0;
      std::uint32_t r1;
      std::uint32_t r2;
      std::uint32_t r3;
      std::uint32_t r4;
      std::uint32_t r5;
      std::uint32_t r6;
      std::uint32_t r7;
      std::uint32_t r8;
      std::uint32_t r9;
      std::uint32_t r10;
      std::uint32_t r11;
      std::uint32_t r12;
      std::uint32_t r13;
      std::uint32_t r14;
      std::uint32_t r15;
    };
    std::uint32_t reg[16];
  };
  
  // Banked Registers
  std::uint32_t bank[kBankCount][7];

  // Program Status Registers
  StatusRegister cpsr;
  StatusRegister spsr[kBankCount];
  
  State() { Reset(); }
  
  void Reset() {
    for (int i = 0; i < 16; i++) {
      reg[i] = 0;
    }

    for (int b = 0; b < kBankCount; b++) {
      for (int r = 0; r < 7; r++) {
        bank[b][r] = 0;
      }
      spsr[b].v = 0;
    }

    cpsr.v = MODE_SYS;
  }
};

} // namespace fauxDS::core::arm
