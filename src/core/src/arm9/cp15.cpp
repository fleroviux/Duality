/*
 * Copyright (C) 2020 fleroviux
 */

#include <functional>

#include "cp15.hpp"

namespace Duality::Core {

CP15::CP15(arm::ARM* core, ARM9MemoryBus* bus)
    : core(core)
    , bus(bus) {
  for (int i = 0; i < kLUTSize; i++) {
    handler_rd[i] = &CP15::DefaultRead;
    handler_wr[i] = &CP15::DefaultWrite;
  }

  RegisterHandler(0, 0, 0, &CP15::ReadMainID);
  RegisterHandler(0, 0, 1, &CP15::ReadCacheType);
  RegisterHandler(1, 0, 0, &CP15::ReadControlRegister);
  RegisterHandler(9, 1, 0, &CP15::ReadDTCMConfig);
  RegisterHandler(9, 1, 1, &CP15::ReadITCMConfig);

  RegisterHandler(1, 0, 0, &CP15::WriteControlRegister);
  RegisterHandler(7, 0, 4, &CP15::WriteWaitForIRQ);
  RegisterHandler(7, 8, 2, &CP15::WriteWaitForIRQ);
  RegisterHandler(9, 1, 0, &CP15::WriteDTCMConfig);
  RegisterHandler(9, 1, 1, &CP15::WriteITCMConfig);
  RegisterHandler(7, 5, 0, &CP15::InvalidateCodeCache);
  RegisterHandler(7, 5, 1, &CP15::InvalidateCodeCacheLineVA);
  RegisterHandler(7, 5, 2, &CP15::InvalidateCodeCacheLineSI);
}

void CP15::Reset() {
  // TODO: these values are for direct boot.
  // they would need to be different if we boot into the firmware.

  // Reset control register (enable DTCM and ITCM, exception base = 0xFFFF0000)
  Write(0, 1, 0, 0, 0x0005707D);

  // Reset DTCM and ITCM configuration
  Write(0, 9, 1, 0, 0x0300000A);
  Write(0, 9, 1, 1, 0x00000020);
}

void CP15::RegisterHandler(int cn, int cm, int opcode, ReadHandler handler) {
  handler_rd[Index(cn, cm, opcode)] = handler;
}

void CP15::RegisterHandler(int cn, int cm, int opcode, WriteHandler handler) {
  handler_wr[Index(cn, cm, opcode)] = handler;
}

auto CP15::Read(int opcode1, int cn, int cm, int opcode2) -> u32 {
  if (opcode1 == 0) {
    return std::invoke(handler_rd[Index(cn, cm, opcode2)], this, cn, cm, opcode2);
  }

  return 0;
}

void CP15::Write(int opcode1, int cn, int cm, int opcode2, u32 value) {
  if (opcode1 == 0) {
    std::invoke(handler_wr[Index(cn, cm, opcode2)], this, cn, cm, opcode2, value);
  }
}

auto CP15::DefaultRead(int cn, int cm, int opcode) -> u32 {
  LOG_WARN("CP15: unknown read c{0} c{1} #{2}", cn, cm, opcode);
  return 0;
}

void CP15::DefaultWrite(int cn, int cm, int opcode, u32 value) {
  LOG_WARN("CP15: unknown write c{0} c{1} #{2} = 0x{3:08X}", cn, cm, opcode, value);
}

auto CP15::ReadMainID(int cn, int cm, int opcode) -> u32 {
  return 0x41059461;
}

auto CP15::ReadCacheType(int cn, int cm, int opcode) -> u32 {
  return 0x0F0D2112;
}

auto CP15::ReadControlRegister(int cn, int cm, int opcode) -> u32 {
  return reg_control;
}

void CP15::WriteControlRegister(int cn, int cm, int opcode, u32 value) {
  // On NDS ARM9:
  // - bits 0, 2, 7 and 12 - 19 are read- and writeable
  // - bits 3 - 6 always are set
  reg_control = (value & 0x000FF085) | 0x78;

  core->ExceptionBase((value & 0x2000) == 0 ? 0x00000000 : 0xFFFF0000);
  
  dtcm_config.enable = value & 0x10000;
  dtcm_config.enable_read = dtcm_config.enable && (value & 0x20000) == 0;
  bus->SetDTCM(dtcm_config);
  
  itcm_config.enable = value & 0x40000;
  itcm_config.enable_read = itcm_config.enable && (value & 0x80000) == 0;
  bus->SetITCM(itcm_config);

  ASSERT((value & 0x80) == 0, "CP15: enabled unsupported big-endian mode!");
  ASSERT((value & 0x8000) == 0, "CP15: enabled unsupported Pre-ARMv5 mode!");
}

void CP15::WriteWaitForIRQ(int cn, int cm, int opcode, u32 value) {
  core->WaitForIRQ();
}

auto CP15::ReadDTCMConfig(int cn, int cm, int opcode) -> u32 {
  return reg_dtcm;
}

auto CP15::ReadITCMConfig(int cn, int cm, int opcode) -> u32 {
  return reg_itcm;
}

void CP15::WriteDTCMConfig(int cn, int cm, int opcode, u32 value) {
  auto size = (value >> 1) & 0x1F;
  if (size < 3 || size > 23) {
    LOG_ERROR("CP15: DTCM virtual size must be between 4 KiB (3) and 4 GiB (23)");
  }

  auto base = value & 0xFFFFF000;
  reg_dtcm = value;
  dtcm_config.base = base;
  dtcm_config.limit = base + (512 << size) - 1;
  if (dtcm_config.limit < dtcm_config.base) {
    LOG_ERROR("CP15: DTCM limit is lower than base address!");
  }
  bus->SetDTCM(dtcm_config);

  LOG_INFO("CP15: DTCM mapped @ 0x{0:08X} - 0x{1:08X}", dtcm_config.base, dtcm_config.limit);
}

void CP15::WriteITCMConfig(int cn, int cm, int opcode, u32 value) {  
  auto size = (value >> 1) & 0x1F;
  if (size < 3 || size > 23) {
    LOG_ERROR("CP15: ITCM virtual size must be between 4 KiB (3) and 4 GiB (23)");
  }

  auto base = value & 0xFFFFF000;
  if (base != 0) {
    value &= 0xFFF;
    LOG_ERROR("CP15: ITCM base address cannot be changed on the Nintendo DS!");
  }
  reg_itcm = value;
  itcm_config.base = 0;
  itcm_config.limit = (512 << size) - 1;
  bus->SetITCM(itcm_config);

  LOG_INFO("CP15: ITCM mapped @ 0x{0:08X} - 0x{1:08X}", itcm_config.base, itcm_config.limit);
}

void CP15::InvalidateCodeCache(int cn, int cm, int opcode, u32 value) {
  core->InvalidateCodeCache();
}

void CP15::InvalidateCodeCacheLineVA(int cn, int cm, int opcode, u32 value) {
  // TODO: only invalidate the relevant address range.
  core->InvalidateCodeCache();
}

void CP15::InvalidateCodeCacheLineSI(int cn, int cm, int opcode, u32 value) {
  // TODO: only invalidate the relevant address range.
  core->InvalidateCodeCache();
}

} // namespace Duality::Core
