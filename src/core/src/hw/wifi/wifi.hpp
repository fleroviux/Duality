/*
 * Copyright (C) 2020 fleroviux
 */

#pragma once

#include <util/integer.hpp>

namespace Duality::Core {

struct WIFI {
  WIFI() {
    Reset();
  }

  void Reset();
  auto ReadByteIO(u32 address) -> u8;
  void WriteByteIO(u32 address, u8 value);

private:
  /* Wifi RAM and stubbed IO registers.
   * TODO: figure out which registers precisely need to be stubbed
   * and only stub those.
   */
  u8 mmio[0x42F8];

  // Baseband chip internal registers.
  u8 bb_regs[0x69];
};

} // namespace Duality::Core
