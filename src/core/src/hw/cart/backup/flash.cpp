/*
 * Copyright (C) 2021 fleroviux
 */

#include <util/log.hpp>

#include "flash.hpp"

// TODO: figure out what state the FLASH chip is in after a command completed.
// Also what happens if you submit a command but do not assert /CS?

// TODO: generate a manufacturer and device id based on e.g. the size.

namespace Duality::Core {

FLASH::FLASH(std::string const& save_path, Size size_hint)
    : save_path(save_path)
    , size_hint(size_hint) {
  Reset();
}

void FLASH::Reset() {
  static const std::vector<size_t> kBackupSize { 
    0x40000, 0x80000, 0x100000 };

  auto size = kBackupSize[static_cast<int>(size_hint)];

  file = BackupFile::OpenOrCreate(save_path, kBackupSize, size);
  mask = size - 1U;
  Deselect();

  write_enable_latch = false;
  deep_power_down = false;
}

void FLASH::Deselect() {
  if (current_cmd == Command::PageWrite ||
      current_cmd == Command::PageProgram ||
      current_cmd == Command::PageErase ||
      current_cmd == Command::SectorErase) {
    write_enable_latch = false;
  }

  state = State::ReceiveCommand;
}

auto FLASH::Transfer(u8 data) -> u8 {
  switch (state) {
    case State::ReceiveCommand: {
      ParseCommand(static_cast<Command>(data));
      break;
    }
    case State::ReadJEDEC: {
      if (address < 3) {
        return jedec_id[address++];
      }
      break;
    }
    case State::ReadStatus: {
      return write_enable_latch ? 2 : 0;
    }
    case State::SendAddress0: {
      address  = data << 16;
      state = State::SendAddress1;
      break;
    }
    case State::SendAddress1: {
      address |= data << 8;
      state = State::SendAddress2;
      break;
    }
    case State::SendAddress2: {
      address |= data;

      switch (current_cmd) {
        case Command::ReadData:
          state = State::ReadData;
          break;
        case Command::ReadDataFast:
          state = State::DummyByte;
          break;
        case Command::PageWrite:
          state = State::PageWrite;
          break;
        case Command::PageProgram:
          state = State::PageProgram;
          break;
        case Command::PageErase:
          state = State::PageErase;
          break;
        case Command::SectorErase:
          state = State::SectorErase;
          break;
        default:
          ASSERT(false, "FLASH: no state for command 0x{0:02X} after 24-bit address.", current_cmd);
      }
      break;
    }
    case State::DummyByte: {
      state = State::ReadData;
      break;
    }
    case State::ReadData: {
      return file->Read(address++ & mask);
    }
    case State::PageWrite: {
      file->Write(address, data);
      address = (address & ~0xFF) | ((address + 1) & 0xFF);
      break;
    }
    case State::PageProgram: {
      // TODO: confirm that page program actually is a bitwise-AND operation.
      // melonDS seems to set the data to zeroes, but is that correct?
      file->Write(address, file->Read(address) & data);
      address = (address & ~0xFF) | ((address + 1) & 0xFF);
      break;
    }
    case State::PageErase: {
      address &= ~0xFF;
      for (uint i = 0; i < 256; i++) {
        file->Write(address + i, 0xFF);
      }
      break;
    }
    case State::SectorErase: {
      address &= ~0xFFFF;
      for (uint i = 0; i < 0x10000; i++) {
        file->Write(address + i, 0xFF);
      }
      break;
    }
    default: {
      UNREACHABLE;
    }
  }

  return 0xFF;
}

void FLASH::ParseCommand(Command cmd) {
  if (deep_power_down) {
    if (cmd == Command::ReleaseDeepPowerDown) {
      deep_power_down = false;
    }
    return;
  }

  current_cmd = cmd;

  switch (cmd) {
    case Command::WriteEnable: {
      write_enable_latch = true;
      break;
    }
    case Command::WriteDisable: {
      write_enable_latch = false;
      break;
    }
    case Command::ReadJEDEC: {
      address = 0;
      state = State::ReadJEDEC;
      break;
    }
    case Command::ReadStatus: {
      state = State::ReadStatus;
      break;
    }
    case Command::ReadData:
    case Command::ReadDataFast: {
      state = State::SendAddress0;
      break;
    }
    case Command::PageWrite:
    case Command::PageProgram:
    case Command::PageErase:
    case Command::SectorErase: {
      if (write_enable_latch) {
        state = State::SendAddress0;
      }
      break;
    }
    case Command::DeepPowerDown: {
      deep_power_down = true;
      break;
    }
  }
}

} // namespace Duality::Core