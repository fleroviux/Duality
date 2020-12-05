/*
 * Copyright (C) 2020 fleroviux
 */

#pragma once

#include <common/integer.hpp>

namespace fauxDS::core {

/// Hardware accelerated 64-bit division and square root engine.
struct MathEngine {
  enum class DivisionMode {
    S32_S32 = 0,
    S64_S32 = 1,
    S64_S64 = 2,
    Reserved = 3
  };
  
  MathEngine() { Reset(); }

  void Reset();

  struct DIVCNT {
    DIVCNT(MathEngine& math_engine) : math_engine(math_engine) {}

    auto ReadByte (uint offset) -> u8;
    void WriteByte(uint offset, u8 value);

  private:
    friend struct fauxDS::core::MathEngine;

    DivisionMode mode = DivisionMode::S32_S32;
    bool error_divide_by_zero = false;

    MathEngine& math_engine;
  } divcnt { *this };

  struct DIV {
    DIV(MathEngine& math_engine) : math_engine(math_engine) {}

    auto ReadByte (uint offset) -> u8;
    void WriteByte(uint offset, u8 value);
  private:
    friend struct fauxDS::core::MathEngine;

    u64 value = 0;

    MathEngine& math_engine;
  };

  DIV div_numer { *this };
  DIV div_denom { *this };
  DIV div_result { *this };
  DIV div_remain { *this };

private:
  void UpdateDivision();
};

}; // namespace fauxDS::core