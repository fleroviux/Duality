/*
 * Copyright (C) 2020 fleroviux
 */

#include <util/log.hpp>

#include "bus.hpp"

namespace Duality::Core {

enum Registers {
  // PPU engine A
  REG_DISPSTAT = 0x0400'0004,
  REG_VCOUNT = 0x0400'0006,

  // DMA
  REG_DMA0SAD = 0x0400'00B0,
  REG_DMA0DAD = 0x0400'00B4,
  REG_DMA0CNT_L = 0x0400'00B8,
  REG_DMA0CNT_H = 0x0400'00BA,
  REG_DMA1SAD = 0x0400'00BC,
  REG_DMA1DAD = 0x0400'00C0,
  REG_DMA1CNT_L = 0x0400'00C4,
  REG_DMA1CNT_H = 0x0400'00C6,
  REG_DMA2SAD = 0x0400'00C8,
  REG_DMA2DAD = 0x0400'00CC,
  REG_DMA2CNT_L = 0x0400'00D0,
  REG_DMA2CNT_H = 0x0400'00D2,
  REG_DMA3SAD = 0x0400'00D4,
  REG_DMA3DAD = 0x0400'00D8,
  REG_DMA3CNT_L = 0x0400'00DC,
  REG_DMA3CNT_H = 0x0400'00DE,

  // Timers
  REG_TM0CNT_L = 0x0400'0100,
  REG_TM0CNT_H = 0x0400'0102,
  REG_TM1CNT_L = 0x0400'0104,
  REG_TM1CNT_H = 0x0400'0106,
  REG_TM2CNT_L = 0x0400'0108,
  REG_TM2CNT_H = 0x0400'010A,
  REG_TM3CNT_L = 0x0400'010C,
  REG_TM3CNT_H = 0x0400'010E,

  // Input
  REG_KEYINPUT = 0x0400'0130,
  REG_EXTKEYINPUT = 0x0400'0136,

  // IPC
  REG_IPCSYNC = 0x0400'0180,
  REG_IPCFIFOCNT = 0x0400'0184,
  REG_IPCFIFOSEND = 0x0400'0188,
  REG_IPCFIFORECV = 0x0410'0000,

  // Cartridge interface
  REG_AUXSPICNT = 0x0400'01A0,
  REG_AUXSPIDATA = 0x0400'01A2,
  REG_ROMCTRL = 0x0400'01A4,
  REG_CARDCMD = 0x0400'01A8,
  REG_CARDDATA = 0x0410'0010,

  // SPI
  REG_SPICNT = 0x0400'01C0,
  REG_SPIDATA = 0x0400'01C2,

  // IRQ
  REG_IME = 0x0400'0208,
  REG_IE  = 0x0400'0210,
  REG_IF  = 0x0400'0214,

  REG_VRAMSTAT = 0x0400'0240,
  REG_WRAMSTAT = 0x0400'0241,

  REG_POSTFLG = 0x0400'0300,
  REG_HALTCNT = 0x0400'0301,

  // Sound
  REG_SOUNDCHAN_LO = 0x0400'0400,
  REG_SOUNDCHAN_HI = 0x0400'04FF
};

static u8 bb_regs[0x69] {0};

auto ARM7MemoryBus::ReadByteIO(u32 address) -> u8 {
  switch (address) {
    // PPU engine A
    case REG_DISPSTAT|0:
      return video_unit.dispstat7.ReadByte(0);
    case REG_DISPSTAT|1:
      return video_unit.dispstat7.ReadByte(1);
    case REG_VCOUNT|0:
      return video_unit.vcount.ReadByte(0);
    case REG_VCOUNT|1:
      return video_unit.vcount.ReadByte(1);

    // DMA
    case REG_DMA0SAD|0:
      return dma.Read(0, 0);
    case REG_DMA0SAD|1:
      return dma.Read(0, 1);
    case REG_DMA0SAD|2:
      return dma.Read(0, 2);
    case REG_DMA0SAD|3:
      return dma.Read(0, 3);
    case REG_DMA0DAD|0:
      return dma.Read(0, 4);
    case REG_DMA0DAD|1:
      return dma.Read(0, 5);
    case REG_DMA0DAD|2:
      return dma.Read(0, 6);
    case REG_DMA0DAD|3:
      return dma.Read(0, 7);
    case REG_DMA0CNT_L|0:
      return dma.Read(0, 8);
    case REG_DMA0CNT_L|1:
      return dma.Read(0, 9);
    case REG_DMA0CNT_H|0:
      return dma.Read(0, 10);
    case REG_DMA0CNT_H|1:
      return dma.Read(0, 11);
    case REG_DMA1SAD|0:
      return dma.Read(1, 0);
    case REG_DMA1SAD|1:
      return dma.Read(1, 1);
    case REG_DMA1SAD|2:
      return dma.Read(1, 2);
    case REG_DMA1SAD|3:
      return dma.Read(1, 3);
    case REG_DMA1DAD|0:
      return dma.Read(1, 4);
    case REG_DMA1DAD|1:
      return dma.Read(1, 5);
    case REG_DMA1DAD|2:
      return dma.Read(1, 6);
    case REG_DMA1DAD|3:
      return dma.Read(1, 7);
    case REG_DMA1CNT_L|0:
      return dma.Read(1, 8);
    case REG_DMA1CNT_L|1:
      return dma.Read(1, 9);
    case REG_DMA1CNT_H|0:
      return dma.Read(1, 10);
    case REG_DMA1CNT_H|1:
      return dma.Read(1, 11);
    case REG_DMA2SAD|0:
      return dma.Read(2, 0);
    case REG_DMA2SAD|1:
      return dma.Read(2, 1);
    case REG_DMA2SAD|2:
      return dma.Read(2, 2);
    case REG_DMA2SAD|3:
      return dma.Read(2, 3);
    case REG_DMA2DAD|0:
      return dma.Read(2, 4);
    case REG_DMA2DAD|1:
      return dma.Read(2, 5);
    case REG_DMA2DAD|2:
      return dma.Read(2, 6);
    case REG_DMA2DAD|3:
      return dma.Read(2, 7);
    case REG_DMA2CNT_L|0:
      return dma.Read(2, 8);
    case REG_DMA2CNT_L|1:
      return dma.Read(2, 9);
    case REG_DMA2CNT_H|0:
      return dma.Read(2, 10);
    case REG_DMA2CNT_H|1:
      return dma.Read(2, 11);
    case REG_DMA3SAD|0:
      return dma.Read(3, 0);
    case REG_DMA3SAD|1:
      return dma.Read(3, 1);
    case REG_DMA3SAD|2:
      return dma.Read(3, 2);
    case REG_DMA3SAD|3:
      return dma.Read(3, 3);
    case REG_DMA3DAD|0:
      return dma.Read(3, 4);
    case REG_DMA3DAD|1:
      return dma.Read(3, 5);
    case REG_DMA3DAD|2:
      return dma.Read(3, 6);
    case REG_DMA3DAD|3:
      return dma.Read(3, 7);
    case REG_DMA3CNT_L|0:
      return dma.Read(3, 8);
    case REG_DMA3CNT_L|1:
      return dma.Read(3, 9);
    case REG_DMA3CNT_H|0:
      return dma.Read(3, 10);
    case REG_DMA3CNT_H|1:
      return dma.Read(3, 11);

    // Timers
    case REG_TM0CNT_L|0:
      return timer.Read(0, 0);
    case REG_TM0CNT_L|1:
      return timer.Read(0, 1);
    case REG_TM0CNT_H|0:
      return timer.Read(0, 2);
    case REG_TM0CNT_H|1:
      return timer.Read(0, 3);
    case REG_TM1CNT_L|0:
      return timer.Read(1, 0);
    case REG_TM1CNT_L|1:
      return timer.Read(1, 1);
    case REG_TM1CNT_H|0:
      return timer.Read(1, 2);
    case REG_TM1CNT_H|1:
      return timer.Read(1, 3);
    case REG_TM2CNT_L|0:
      return timer.Read(2, 0);
    case REG_TM2CNT_L|1:
      return timer.Read(2, 1);
    case REG_TM2CNT_H|0:
      return timer.Read(2, 2);
    case REG_TM2CNT_H|1:
      return timer.Read(2, 3);
    case REG_TM3CNT_L|0:
      return timer.Read(3, 0);
    case REG_TM3CNT_L|1:
      return timer.Read(3, 1);
    case REG_TM3CNT_H|0:
      return timer.Read(3, 2);
    case REG_TM3CNT_H|1:
      return timer.Read(3, 3);

    // Input
    case REG_KEYINPUT|0:
      return keyinput.ReadByte(0);
    case REG_KEYINPUT|1:
      return keyinput.ReadByte(1);
    case REG_EXTKEYINPUT:
      return extkeyinput.ReadByte();

    // IPC
    case REG_IPCSYNC|0:
      return ipc.ipcsync.ReadByte(IPC::Client::ARM7, 0);
    case REG_IPCSYNC|1:
      return ipc.ipcsync.ReadByte(IPC::Client::ARM7, 1);
    case REG_IPCFIFOCNT|0:
      return ipc.ipcfifocnt.ReadByte(IPC::Client::ARM7, 0);
    case REG_IPCFIFOCNT|1:
      return ipc.ipcfifocnt.ReadByte(IPC::Client::ARM7, 1);
    case REG_IPCFIFORECV|0:
      return ipc.ipcfiforecv.ReadByte(IPC::Client::ARM7, 0);
    case REG_IPCFIFORECV|1:
      return ipc.ipcfiforecv.ReadByte(IPC::Client::ARM7, 1);
    case REG_IPCFIFORECV|2:
      return ipc.ipcfiforecv.ReadByte(IPC::Client::ARM7, 2);
    case REG_IPCFIFORECV|3:
      return ipc.ipcfiforecv.ReadByte(IPC::Client::ARM7, 3);

    // Cartridge interface
    case REG_AUXSPICNT|0:
      return cart.auxspicnt.ReadByte(0);
    case REG_AUXSPICNT|1:
      return cart.auxspicnt.ReadByte(1);
    case REG_AUXSPIDATA|0:
      return cart.ReadSPI();
    case REG_AUXSPIDATA|1:
      return 0;
    case REG_ROMCTRL|0:
      return cart.romctrl.ReadByte(0);
    case REG_ROMCTRL|1:
      return cart.romctrl.ReadByte(1);
    case REG_ROMCTRL|2:
      return cart.romctrl.ReadByte(2);
    case REG_ROMCTRL|3:
      return cart.romctrl.ReadByte(3);
    case REG_CARDCMD|0:
      return cart.cardcmd.ReadByte(0);
    case REG_CARDCMD|1:
      return cart.cardcmd.ReadByte(1);
    case REG_CARDCMD|2:
      return cart.cardcmd.ReadByte(2);
    case REG_CARDCMD|3:
      return cart.cardcmd.ReadByte(3);
    case REG_CARDCMD|4:
      return cart.cardcmd.ReadByte(4);
    case REG_CARDCMD|5:
      return cart.cardcmd.ReadByte(5);
    case REG_CARDCMD|6:
      return cart.cardcmd.ReadByte(6);
    case REG_CARDCMD|7:
      return cart.cardcmd.ReadByte(7);
    case REG_CARDDATA|0:
    case REG_CARDDATA|1:
    case REG_CARDDATA|2:
    case REG_CARDDATA|3:
      ASSERT(false, "ARM7: unhandled byte read from REG_CARDDATA");
      return 0;

    // SPI
    case REG_SPICNT|0:
      return spi.spicnt.ReadByte(0);
    case REG_SPICNT|1:
      return spi.spicnt.ReadByte(1);
    case REG_SPIDATA|0:
      return spi.spidata.ReadByte();
    case REG_SPIDATA|1:
      // not functional/used but accessed anyways.
      return 0;

    // IRQ
    case REG_IME|0:
      return irq7.ime.ReadByte(0);
    case REG_IME|1:
      return irq7.ime.ReadByte(1);
    case REG_IME|2:
      return irq7.ime.ReadByte(2);
    case REG_IME|3:
      return irq7.ime.ReadByte(3);
    case REG_IE|0:
      return irq7.ie.ReadByte(0);
    case REG_IE|1:
      return irq7.ie.ReadByte(1);
    case REG_IE|2:
      return irq7.ie.ReadByte(2);
    case REG_IE|3:
      return irq7.ie.ReadByte(3);
    case REG_IF|0:
      return irq7._if.ReadByte(0);
    case REG_IF|1:
      return irq7._if.ReadByte(1);
    case REG_IF|2:
      return irq7._if.ReadByte(2);
    case REG_IF|3:
      return irq7._if.ReadByte(3);

    case REG_VRAMSTAT:
      return vram.vramstat.ReadByte();
    case REG_WRAMSTAT:
      return wramcnt.ReadByte();

    case REG_POSTFLG:
     return 1;

    case REG_SOUNDCHAN_LO ... REG_SOUNDCHAN_HI:
      return apu.Read((address >> 4) & 15, address & 15);

    default:
      //LOG_WARN("ARM7: MMIO: unhandled read from 0x{0:08X} r15=0x{1:08X}", address, r15);
      
      return derp[address & 0xFFFFFF];
  }

  return 0;
}

auto ARM7MemoryBus::ReadHalfIO(u32 address) -> u16 {
  switch (address) {
    case REG_IPCFIFORECV|0:
      return ipc.ipcfiforecv.ReadHalf(IPC::Client::ARM7, 0);
    case REG_IPCFIFORECV|2:
      return ipc.ipcfiforecv.ReadHalf(IPC::Client::ARM7, 2);
  }

  return (ReadByteIO(address | 0) << 0) |
         (ReadByteIO(address | 1) << 8);
}

auto ARM7MemoryBus::ReadWordIO(u32 address) -> u32 {
  switch (address) {
    case REG_IPCFIFORECV:
      return ipc.ipcfiforecv.ReadWord(IPC::Client::ARM7);
    case REG_CARDDATA:
      return cart.ReadROM();
  }

  return (ReadByteIO(address | 0) <<  0) |
         (ReadByteIO(address | 1) <<  8) |
         (ReadByteIO(address | 2) << 16) |
         (ReadByteIO(address | 3) << 24);
}

void ARM7MemoryBus::WriteByteIO(u32 address,  u8 value) {
  switch (address) {
    // PPU engine A
    case REG_DISPSTAT|0:
      video_unit.dispstat7.WriteByte(0, value);
      break;
    case REG_DISPSTAT|1:
      video_unit.dispstat7.WriteByte(1, value);
      break;

    // DMA
    case REG_DMA0SAD|0:
      dma.Write(0, 0, value);
      break;
    case REG_DMA0SAD|1:
      dma.Write(0, 1, value);
      break;
    case REG_DMA0SAD|2:
      dma.Write(0, 2, value);
      break;
    case REG_DMA0SAD|3:
      dma.Write(0, 3, value);
      break;
    case REG_DMA0DAD|0:
      dma.Write(0, 4, value);
      break;
    case REG_DMA0DAD|1:
      dma.Write(0, 5, value);
      break;
    case REG_DMA0DAD|2:
      dma.Write(0, 6, value);
      break;
    case REG_DMA0DAD|3:
      dma.Write(0, 7, value);
      break;
    case REG_DMA0CNT_L|0:
      dma.Write(0, 8, value);
      break;
    case REG_DMA0CNT_L|1:
      dma.Write(0, 9, value);
      break;
    case REG_DMA0CNT_H|0:
      dma.Write(0, 10, value);
      break;
    case REG_DMA0CNT_H|1:
      dma.Write(0, 11, value);
      break;
    case REG_DMA1SAD|0:
      dma.Write(1, 0, value);
      break;
    case REG_DMA1SAD|1:
      dma.Write(1, 1, value);
      break;
    case REG_DMA1SAD|2:
      dma.Write(1, 2, value);
      break;
    case REG_DMA1SAD|3:
      dma.Write(1, 3, value);
      break;
    case REG_DMA1DAD|0:
      dma.Write(1, 4, value);
      break;
    case REG_DMA1DAD|1:
      dma.Write(1, 5, value);
      break;
    case REG_DMA1DAD|2:
      dma.Write(1, 6, value);
      break;
    case REG_DMA1DAD|3:
      dma.Write(1, 7, value);
      break;
    case REG_DMA1CNT_L|0:
      dma.Write(1, 8, value);
      break;
    case REG_DMA1CNT_L|1:
      dma.Write(1, 9, value);
      break;
    case REG_DMA1CNT_H|0:
      dma.Write(1, 10, value);
      break;
    case REG_DMA1CNT_H|1:
      dma.Write(1, 11, value);
      break;
    case REG_DMA2SAD|0:
      dma.Write(2, 0, value);
      break;
    case REG_DMA2SAD|1:
      dma.Write(2, 1, value);
      break;
    case REG_DMA2SAD|2:
      dma.Write(2, 2, value);
      break;
    case REG_DMA2SAD|3:
      dma.Write(2, 3, value);
      break;
    case REG_DMA2DAD|0:
      dma.Write(2, 4, value);
      break;
    case REG_DMA2DAD|1:
      dma.Write(2, 5, value);
      break;
    case REG_DMA2DAD|2:
      dma.Write(2, 6, value);
      break;
    case REG_DMA2DAD|3:
      dma.Write(2, 7, value);
      break;
    case REG_DMA2CNT_L|0:
      dma.Write(2, 8, value);
      break;
    case REG_DMA2CNT_L|1:
      dma.Write(2, 9, value);
      break;
    case REG_DMA2CNT_H|0:
      dma.Write(2, 10, value);
      break;
    case REG_DMA2CNT_H|1:
      dma.Write(2, 11, value);
      break;
    case REG_DMA3SAD|0:
      dma.Write(3, 0, value);
      break;
    case REG_DMA3SAD|1:
      dma.Write(3, 1, value);
      break;
    case REG_DMA3SAD|2:
      dma.Write(3, 2, value);
      break;
    case REG_DMA3SAD|3:
      dma.Write(3, 3, value);
      break;
    case REG_DMA3DAD|0:
      dma.Write(3, 4, value);
      break;
    case REG_DMA3DAD|1:
      dma.Write(3, 5, value);
      break;
    case REG_DMA3DAD|2:
      dma.Write(3, 6, value);
      break;
    case REG_DMA3DAD|3:
      dma.Write(3, 7, value);
      break;
    case REG_DMA3CNT_L|0:
      dma.Write(3, 8, value);
      break;
    case REG_DMA3CNT_L|1:
      dma.Write(3, 9, value);
      break;
    case REG_DMA3CNT_H|0:
      dma.Write(3, 10, value);
      break;
    case REG_DMA3CNT_H|1:
      dma.Write(3, 11, value);
      break;

    // Timers
    case REG_TM0CNT_L|0:
      timer.Write(0, 0, value);
      break;
    case REG_TM0CNT_L|1:
      timer.Write(0, 1, value);
      break;
    case REG_TM0CNT_H|0:
      timer.Write(0, 2, value);
      break;
    case REG_TM0CNT_H|1:
      timer.Write(0, 3, value);
      break;
    case REG_TM1CNT_L|0:
      timer.Write(1, 0, value);
      break;
    case REG_TM1CNT_L|1:
      timer.Write(1, 1, value);
      break;
    case REG_TM1CNT_H|0:
      timer.Write(1, 2, value);
      break;
    case REG_TM1CNT_H|1:
      timer.Write(1, 3, value);
      break;
    case REG_TM2CNT_L|0:
      timer.Write(2, 0, value);
      break;
    case REG_TM2CNT_L|1:
      timer.Write(2, 1, value);
      break;
    case REG_TM2CNT_H|0:
      timer.Write(2, 2, value);
      break;
    case REG_TM2CNT_H|1:
      timer.Write(2, 3, value);
      break;
    case REG_TM3CNT_L|0:
      timer.Write(3, 0, value);
      break;
    case REG_TM3CNT_L|1:
      timer.Write(3, 1, value);
      break;
    case REG_TM3CNT_H|0:
      timer.Write(3, 2, value);
      break;
    case REG_TM3CNT_H|1:
      timer.Write(3, 3, value);
      break;

    // IPC
    case REG_IPCSYNC|0:
      ipc.ipcsync.WriteByte(IPC::Client::ARM7, 0, value);
      break;
    case REG_IPCSYNC|1:
      ipc.ipcsync.WriteByte(IPC::Client::ARM7, 1, value);
      break;
    case REG_IPCFIFOCNT|0:
      ipc.ipcfifocnt.WriteByte(IPC::Client::ARM7, 0, value);
      break;
    case REG_IPCFIFOCNT|1:
      ipc.ipcfifocnt.WriteByte(IPC::Client::ARM7, 1, value);
      break;
    case REG_IPCFIFOSEND|0:
    case REG_IPCFIFOSEND|1:
    case REG_IPCFIFOSEND|2:
    case REG_IPCFIFOSEND|3:
      ipc.ipcfifosend.WriteByte(IPC::Client::ARM7, value);
      break;

    // Cartridge interface
    case REG_AUXSPICNT|0:
      cart.auxspicnt.WriteByte(0, value);
      break;
    case REG_AUXSPICNT|1:
      cart.auxspicnt.WriteByte(1, value);
      break;
    case REG_AUXSPIDATA|0:
      cart.WriteSPI(value);
      break;
    case REG_AUXSPIDATA|1:
      break;
    case REG_ROMCTRL|0:
      cart.romctrl.WriteByte(0, value);
      break;
    case REG_ROMCTRL|1:
      cart.romctrl.WriteByte(1, value);
      break;
    case REG_ROMCTRL|2:
      cart.romctrl.WriteByte(2, value);
      break;
    case REG_ROMCTRL|3:
      cart.romctrl.WriteByte(3, value);
      break;
    case REG_CARDCMD|0:
      cart.cardcmd.WriteByte(0, value);
      break;
    case REG_CARDCMD|1:
      cart.cardcmd.WriteByte(1, value);
      break;
    case REG_CARDCMD|2:
      cart.cardcmd.WriteByte(2, value);
      break;
    case REG_CARDCMD|3:
      cart.cardcmd.WriteByte(3, value);
      break;
    case REG_CARDCMD|4:
      cart.cardcmd.WriteByte(4, value);
      break;
    case REG_CARDCMD|5:
      cart.cardcmd.WriteByte(5, value);
      break;
    case REG_CARDCMD|6:
      cart.cardcmd.WriteByte(6, value);
      break;
    case REG_CARDCMD|7:
      cart.cardcmd.WriteByte(7, value);
      break;

    // SPI
    case REG_SPICNT|0:
      spi.spicnt.WriteByte(0, value);
      break;
    case REG_SPICNT|1:
      spi.spicnt.WriteByte(1, value);
      break;
    case REG_SPIDATA|0:
      spi.spidata.WriteByte(value);
      break;
    case REG_SPIDATA|1:
      // not functional/used but accessed anyways.
      break;

    // IRQ
    case REG_IME|0:
      irq7.ime.WriteByte(0, value);
      break;
    case REG_IME|1:
      irq7.ime.WriteByte(1, value);
      break;
    case REG_IME|2:
      irq7.ime.WriteByte(2, value);
      break;
    case REG_IME|3:
      irq7.ime.WriteByte(3, value);
      break;
    case REG_IE|0:
      irq7.ie.WriteByte(0, value);
      break;
    case REG_IE|1:
      irq7.ie.WriteByte(1, value);
      break;
    case REG_IE|2:
      irq7.ie.WriteByte(2, value);
      break;
    case REG_IE|3:
      irq7.ie.WriteByte(3, value);
      break;
    case REG_IF|0:
      irq7._if.WriteByte(0, value);
      break;
    case REG_IF|1:
      irq7._if.WriteByte(1, value);
      break;
    case REG_IF|2:
      irq7._if.WriteByte(2, value);
      break;
    case REG_IF|3:
      irq7._if.WriteByte(3, value);
      break;

    case REG_HALTCNT: {
      auto mode = value >> 6;
      if (mode == 2) {
        IsHalted() = true;
      } else if (mode != 0) {
        LOG_ERROR("ARM7: MMIO: unhandled HALTCNT mode #{0}", mode);
      }
      break;
    }

    case REG_SOUNDCHAN_LO ... REG_SOUNDCHAN_HI:
      return apu.Write((address >> 4) & 15, address & 15, value);

    default:
      
      // stub BB read/write
      if (address == 0x04808159) {
        auto direction = value >> 4;
        auto index = derp[0x808158] & 0xFF;
        
        if (direction == 5) {
          bb_regs[index] = derp[0x80815A];
        }

        if (direction == 6) {
          derp[0x80815C] = bb_regs[index];
        }
      }

      derp[address & 0xFFFFFF] = value;
      //LOG_WARN("ARM7: MMIO: unhandled write to 0x{0:08X} = 0x{1:02X}", address, value);
  }
}

void ARM7MemoryBus::WriteHalfIO(u32 address, u16 value) {
  switch (address) {
    case REG_IPCFIFOSEND|0:
    case REG_IPCFIFOSEND|2:
      ipc.ipcfifosend.WriteHalf(IPC::Client::ARM7, value);
      break;
    default:
      WriteByteIO(address | 0, value & 0xFF);
      WriteByteIO(address | 1, value >> 8);
      break;
  }
}

void ARM7MemoryBus::WriteWordIO(u32 address, u32 value) {
  switch (address) {
    case REG_IPCFIFOSEND:
      ipc.ipcfifosend.WriteWord(IPC::Client::ARM7, value);
      break;
    default:
      WriteByteIO(address | 0, u8(value >>  0));
      WriteByteIO(address | 1, u8(value >>  8));
      WriteByteIO(address | 2, u8(value >> 16));
      WriteByteIO(address | 3, u8(value >> 24));
      break;
  }
}

} // namespace Duality::Core
