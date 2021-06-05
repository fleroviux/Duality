/*
 * Copyright (C) 2021 fleroviux
 */

#pragma once

// TODO: make this a CMake option of lunatic.
#define LUNATIC_NO_CUSTOM_INT_TYPES

#include <lunatic/cpu.hpp>

#include "arm/arm.hpp"
#include "interconnect.hpp"
#include "bus.hpp"

namespace Duality::Core {

struct ARM7 {
  ARM7(Interconnect& interconnect);

  void Reset(u32 entrypoint);
  auto Bus() -> ARM7MemoryBus& { return bus; }
  bool IsHalted() { return bus.IsHalted(); }
  void Run(uint cycles);

private:
  /// No-operation stub for the CP14 coprocessor
  struct CP14 : arm::Coprocessor {
    void Reset() override {}

    auto Read(
      int opcode1,
      int cn,
      int cm,
      int opcode2
    ) -> u32 override { return 0; }
    
    void Write(
      int opcode1,
      int cn,
      int cm,
      int opcode2,
      u32 value
    ) override {}
  } cp14;

  struct Memory final : lunatic::Memory {
    Memory(ARM7MemoryBus* memory) : memory(memory) {
      // this will go bad on exit. ahahahaha.
      pagetable = std::unique_ptr<std::array<u8*, 1048576>>{memory->pagetable.get()};
    }

    auto ReadByte(u32 address, Bus bus) -> u8 {
      return memory->ReadByte(address, static_cast<arm::MemoryBase::Bus>(bus));
    }

    auto ReadHalf(u32 address, Bus bus) -> u16 {
      return memory->ReadHalf(address, static_cast<arm::MemoryBase::Bus>(bus));
    }

    auto ReadWord(u32 address, Bus bus) -> u32 {
      return memory->ReadWord(address, static_cast<arm::MemoryBase::Bus>(bus));
    }

    void WriteByte(u32 address, u8 value, Bus bus) {
      memory->WriteByte(address, value, static_cast<arm::MemoryBase::Bus>(bus));
    }


    void WriteHalf(u32 address, u16 value, Bus bus) {
      memory->WriteHalf(address, value, static_cast<arm::MemoryBase::Bus>(bus));
    }


    void WriteWord(u32 address, u32 value, Bus bus) {
      memory->WriteWord(address, value, static_cast<arm::MemoryBase::Bus>(bus));
    }

    ARM7MemoryBus* memory;
  } jit_memory {&bus};

  ARM7MemoryBus bus;
  arm::ARM core;
  IRQ& irq;
  std::unique_ptr<lunatic::CPU> cpu;
};

} // namespace Duality::Core
