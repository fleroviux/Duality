/*
 * Copyright (C) 2020 fleroviux
 *
 * Licensed under GPLv3 or any later version.
 * Refer to the included LICENSE file.
 */

void SetZeroAndSignFlag(std::uint32_t value) {
  state.cpsr.f.n = value >> 31;
  state.cpsr.f.z = (value == 0);
}

std::uint32_t ADD(std::uint32_t op1, std::uint32_t op2, bool set_flags) {
  if (set_flags) {
    std::uint64_t result64 = (std::uint64_t)op1 + (std::uint64_t)op2;
    std::uint32_t result32 = (std::uint32_t)result64;

    SetZeroAndSignFlag(result32);
    state.cpsr.f.c = result64 >> 32;
    state.cpsr.f.v = (~(op1 ^ op2) & (op2 ^ result32)) >> 31;
    return result32;
  } else {
    return op1 + op2;
  }
}

std::uint32_t ADC(std::uint32_t op1, std::uint32_t op2, bool set_flags) {
  if (set_flags) {
    std::uint64_t result64 = (std::uint64_t)op1 + (std::uint64_t)op2 + (std::uint64_t)state.cpsr.f.c;
    std::uint32_t result32 = (std::uint32_t)result64;

    SetZeroAndSignFlag(result32);
    state.cpsr.f.c = result64 >> 32;
    state.cpsr.f.v = (~(op1 ^ op2) & (op2 ^ result32)) >> 31;
    return result32;
  } else {
    return op1 + op2 + state.cpsr.f.c;
  }
}

std::uint32_t SUB(std::uint32_t op1, std::uint32_t op2, bool set_flags) {
  std::uint32_t result = op1 - op2;

  if (set_flags) {
    SetZeroAndSignFlag(result);
    state.cpsr.f.c = op1 >= op2;
    state.cpsr.f.v = ((op1 ^ op2) & (op1 ^ result)) >> 31;
  }

  return result;
}

std::uint32_t SBC(std::uint32_t op1, std::uint32_t op2, bool set_flags) {
  std::uint32_t op3 = (state.cpsr.f.c) ^ 1;
  std::uint32_t result = op1 - op2 - op3;

  if (set_flags) {
    SetZeroAndSignFlag(result);
    state.cpsr.f.c = (std::uint64_t)op1 >= (std::uint64_t)op2 + (std::uint64_t)op3;
    state.cpsr.f.v = ((op1 ^ op2) & (op1 ^ result)) >> 31;
  }

  return result;
}

std::uint32_t QADD(std::uint32_t op1, std::uint32_t op2, bool saturate = true) {
  std::uint32_t result = op1 + op2;

  if ((~(op1 ^ op2) & (op2 ^ result)) >> 31) {
    state.cpsr.f.q = 1;
    if (saturate) {
      return 0x80000000 - (result >> 31);
    }
  }

  return result;
}

std::uint32_t QSUB(std::uint32_t op1, std::uint32_t op2) {
  return QADD(op1, ~(op2 - 1), true);
}

void DoShift(int opcode, std::uint32_t& operand, std::uint32_t amount, int& carry, bool immediate) {
  /* TODO: how does ARMv6K/ARM11 MPCore handle shift amounts > 255? */
  amount &= 0xFF;

  switch (opcode) {
    case 0: LSL(operand, amount, carry); break;
    case 1: LSR(operand, amount, carry, immediate); break;
    case 2: ASR(operand, amount, carry, immediate); break;
    case 3: ROR(operand, amount, carry, immediate); break;
  }
}

void LSL(std::uint32_t& operand, std::uint32_t amount, int& carry) {
  if (amount == 0) return;

#if (defined(__i386__) || defined(__x86_64__))
  if (amount >= 32) {
    if (amount > 32) {
      carry = 0;
    } else {
      carry = operand & 1;
    }
    operand = 0;
    return;
  }
#endif
  carry = (operand << (amount - 1)) >> 31;
  operand <<= amount;
}

void LSR(std::uint32_t& operand, std::uint32_t amount, int& carry, bool immediate) {
  if (amount == 0) {
    // LSR #0 equals to LSR #32
    if (immediate) {
      amount = 32;
    } else {
      return;
    }
  }

#if (defined(__i386__) || defined(__x86_64__))
  if (amount >= 32) {
    if (amount > 32) {
      carry = 0;
    } else {
      carry = operand >> 31;
    }
    operand = 0;
    return;
  }
#endif
  carry = (operand >> (amount - 1)) & 1;
  operand >>= amount;
}

void ASR(std::uint32_t& operand, std::uint32_t amount, int& carry, bool immediate) {
  if (amount == 0) {
    // ASR #0 equals to ASR #32
    if (immediate) {
      amount = 32;
    } else {
      return;
    }
  }

  int msb = operand >> 31;

#if (defined(__i386__) || defined(__x86_64__))
  if (amount >= 32) {
    carry = msb;
    operand = 0xFFFFFFFF * msb;
    return;
  }
#endif

  carry = (operand >> (amount - 1)) & 1;
  operand = (operand >> amount) | ((0xFFFFFFFF * msb) << (32 - amount));
}

void ROR(std::uint32_t& operand, std::uint32_t amount, int& carry, bool immediate) {
  // ROR #0 equals to RRX #1
  if (amount != 0 || !immediate) {
    if (amount == 0) return;

    amount %= 32;
    operand = (operand >> amount) | (operand << (32 - amount));
    carry = operand >> 31;
  } else {
    auto lsb = operand & 1;
    operand = (operand >> 1) | (carry << 31);
    carry = lsb;
  }
}
