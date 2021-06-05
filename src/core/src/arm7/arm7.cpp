/*
 * Copyright (C) 2021 fleroviux
 */

#include "arm7.hpp"

namespace Duality::Core {

ARM7::ARM7(Interconnect& interconnect)
    : bus(&interconnect)
    , core(arm::ARM::Architecture::ARMv4T, &bus)
    , irq(interconnect.irq7) {
  core.AttachCoprocessor(14, &cp14);
  irq.SetCore(core);
  interconnect.dma7.SetMemory(&bus);
  interconnect.apu.SetMemory(&bus);

  cpu = lunatic::CreateCPU(lunatic::CPU::Descriptor{
    .model = lunatic::CPU::Descriptor::Model::ARM7,
    .block_size = 32,
    .memory = jit_memory
  });

  Reset(0);
}

void ARM7::Reset(u32 entrypoint) {
  // TODO: reset the bus and all associated devices.
  core.ExceptionBase(0);
  core.Reset();
  core.SwitchMode(arm::MODE_SYS);
  core.GetState().r13 = 0x0380FD80;
  core.GetState().bank[arm::BANK_IRQ][arm::BANK_R13] = 0x0380FF80;
  core.GetState().bank[arm::BANK_SVC][arm::BANK_R13] = 0x0380FFC0;
  core.SetPC(entrypoint);

  cpu->GetGPR(lunatic::GPR::SP, lunatic::Mode::System) = 0x0380FD80;
  cpu->GetGPR(lunatic::GPR::SP, lunatic::Mode::IRQ) = 0x0380FF80;
  cpu->GetGPR(lunatic::GPR::SP, lunatic::Mode::Supervisor) = 0x0380FFC0;
  cpu->GetGPR(lunatic::GPR::PC) = entrypoint + sizeof(u32) * 2;
  cpu->GetCPSR().f.mode = lunatic::Mode::System;
}

void ARM7::Run(uint cycles) {
  if (!bus.IsHalted() || irq.HasPendingIRQ()) {
    bus.IsHalted() = false;
    //core.Run(cycles);
    // FIXME
    cpu->IRQLine() = core.IRQLine();
    cpu->Run(cycles);
  }
}

} // namespace Duality::Core
