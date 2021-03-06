/*
 * Copyright (C) 2020 fleroviux
 */

#pragma once

#include <functional>
#include <util/integer.hpp>
#include <util/log.hpp>
#include <core/device/video_device.hpp>

#include "gpu/gpu.hpp"
#include "ppu/ppu.hpp"
#include "vram.hpp"
#include "hw/dma/dma7.hpp"
#include "hw/dma/dma9.hpp"
#include "hw/irq/irq.hpp"
#include "scheduler.hpp"

namespace Duality::Core {

/// Graphics subsystem which contains two 2D PPUs (A and B) and a 3D GPU.
struct VideoUnit {
  enum class Screen { Top, Bottom };

  VideoUnit(Scheduler& scheduler, IRQ& irq7, IRQ& irq9, DMA7& dma7, DMA9& dma9);

  void Reset();
  void SetVideoDevice(VideoDevice& device);
  auto GetOutput(Screen screen) -> u32 const*;

  /// Graphics status and IRQ control.
  struct DisplayStatus {
    auto ReadByte (uint offset) -> u8;
    void WriteByte(uint offset, u8 value);

  private:
    friend struct Duality::Core::VideoUnit;

    struct {
      bool flag = false;
      bool enable_irq = false;
    } vblank = {}, hblank = {}, vcount = {};

    u16 vcount_setting = 0;

    std::function<void(void)> write_cb;
  } dispstat7, dispstat9;

  /// Currently rendered scanline.
  /// TODO: "VCOUNT register is write-able, allowing to synchronize linked DS consoles."
  struct VCOUNT {
    auto ReadByte(uint offset) -> u8;

  private:
    friend struct Duality::Core::VideoUnit;
    u16 value = 0xFFFF;
  } vcount;

  /// Graphics power control register
  /// TODO: right now all bits except for "display_swap" will be ignored.
  struct PowerControl {
    auto ReadByte (uint offset) -> u8;
    void WriteByte(uint offset, u8 value);

  private:
    friend struct Duality::Core::VideoUnit;

    bool enable_lcds = false;
    bool enable_ppu_a = false;
    bool enable_ppu_b = false;
    bool enable_gpu_geometry = false;
    bool enable_gpu_render = false;
    bool display_swap = false;
  } powcnt1;

  u8 pram[0x800];
  u8 oam[0x800];
  VRAM vram;
  GPU gpu;
  PPU ppu_a;
  PPU ppu_b;

private:
  void CheckVerticalCounterIRQ(DisplayStatus& dispstat, IRQ& irq);
  void OnHdrawBegin(int late);
  void OnHblankBegin(int late);

  Scheduler& scheduler;
  IRQ& irq7;
  IRQ& irq9;
  DMA7& dma7;
  DMA9& dma9;
  VideoDevice* video_device = nullptr;
};

} // namespace Duality::Core
