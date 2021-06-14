/*
 * Copyright (C) 2020 fleroviux
 */

#pragma once

#include <util/integer.hpp>
#include <fstream>

#include "arm/memory.hpp"
#include "interconnect.hpp"

namespace Duality::Core {

struct ARM7MemoryBus final : arm::MemoryBase {
  ARM7MemoryBus(Interconnect* interconnect);

  bool& IsHalted() { return halted; }

  auto ReadByte(u32 address, Bus bus) ->  u8 override;
  auto ReadHalf(u32 address, Bus bus) -> u16 override;
  auto ReadWord(u32 address, Bus bus) -> u32 override;
  auto ReadQuad(u32 address, Bus bus) -> u64 override;

  void WriteByte(u32 address,  u8 value, Bus bus) override;
  void WriteHalf(u32 address, u16 value, Bus bus) override;
  void WriteWord(u32 address, u32 value, Bus bus) override;
  void WriteQuad(u32 address, u64 value, Bus bus) override;

  std::uint32_t r15 = 0;

private:
  void UpdateMemoryMap(u32 address_lo, u64 address_hi);

  template<typename T>
  auto Read(u32 address) -> T;

  template<typename T>
  void Write(u32 address, T value);

  auto ReadByteIO(u32 address) ->  u8;
  auto ReadHalfIO(u32 address) -> u16;
  auto ReadWordIO(u32 address) -> u32;
  
  void WriteByteIO(u32 address,  u8 value);
  void WriteHalfIO(u32 address, u16 value);
  void WriteWordIO(u32 address, u32 value);

  /// ARM7 internal memory
  u8 bios[0x4000] {0};
  u8 iwram[0x10000];
  u8 derp[0x1000000];

  /// ARM7 and ARM9 shared memory
  u8* ewram;
  Interconnect::SWRAM::Alloc const& swram;

  /// HW components and I/O registers
  APU& apu;
  Cartridge& cart;
  IPC& ipc;
  IRQ& irq7;
  SPI& spi;
  Timer& timer;
  DMA7& dma;
  VideoUnit& video_unit;
  VRAM& vram;
  Interconnect::WRAMCNT& wramcnt;
  Interconnect::KeyInput& keyinput;
  Interconnect::ExtKeyInput& extkeyinput;
  bool halted;
};

} // namespace Duality::Core
