/*
 * Copyright (C) 2020 fleroviux
 */

#pragma once

#include <common/integer.hpp>

namespace fauxDS::core::arm {

/** Base class that memory systems must implement
  * in order to be connected to an ARM core.
  * Provides an uniform interface for ARM cores to
  * access memory.
  */
struct MemoryBase {
  enum class Bus : int {
    Code,
    Data
  };

  virtual void SetDTCM(u32 base, u32 limit) {}
  virtual void SetITCM(u32 base, u32 limit) {}

  virtual auto ReadByte(u32 address, Bus bus) ->  u8 = 0;
  virtual auto ReadHalf(u32 address, Bus bus) -> u16 = 0;
  virtual auto ReadWord(u32 address, Bus bus) -> u32 = 0;
  
  virtual void WriteByte(u32 address, u8  value) = 0;
  virtual void WriteHalf(u32 address, u16 value) = 0;
  virtual void WriteWord(u32 address, u32 value) = 0;
};

} // namespace fauxDS::core::arm
