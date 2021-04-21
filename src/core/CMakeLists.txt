cmake_minimum_required(VERSION 3.2)
project(Duality-Core CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
  src/arm/arm.cpp
  src/arm/tablegen/tablegen.cpp
  src/arm7/arm7.cpp
  src/arm7/bus.cpp
  src/arm7/bus_mmio.cpp
  src/arm9/arm9.cpp
  src/arm9/bus.cpp
  src/arm9/bus_mmio.cpp
  src/arm9/cp15.cpp
  src/hw/apu/apu.cpp
  src/hw/cart/backup/eeprom.cpp
  src/hw/cart/backup/eeprom512b.cpp
  src/hw/cart/backup/flash.cpp
  src/hw/cart/cart.cpp
  src/hw/dma/dma7.cpp
  src/hw/dma/dma9.cpp
  src/hw/ipc/ipc.cpp
  src/hw/irq/irq.cpp
  src/hw/math/math_engine.cpp
  src/hw/spi/tsc/tsc.cpp
  src/hw/spi/spi.cpp
  src/hw/timer/timer.cpp
  src/hw/video_unit/gpu/commands.cpp
  src/hw/video_unit/gpu/gpu.cpp
  src/hw/video_unit/gpu/renderer.cpp
  src/hw/video_unit/ppu/render/affine.cpp
  src/hw/video_unit/ppu/render/oam.cpp
  src/hw/video_unit/ppu/render/text.cpp
  src/hw/video_unit/ppu/render/window.cpp
  src/hw/video_unit/ppu/composer.cpp
  src/hw/video_unit/ppu/ppu.cpp
  src/hw/video_unit/ppu/registers.cpp
  src/hw/video_unit/video_unit.cpp
  src/hw/video_unit/vram.cpp
  src/core_impl.cpp
  src/scheduler.cpp)

set(HEADERS
  src/arm/handlers/arithmetic.inl
  src/arm/handlers/handler16.inl
  src/arm/handlers/handler32.inl
  src/arm/handlers/memory.inl
  src/arm/tablegen/decoder.hpp
  src/arm/tablegen/gen_arm.hpp
  src/arm/tablegen/gen_thumb.hpp
  src/arm/arm.hpp
  src/arm/coprocessor.hpp
  src/arm/state.hpp
  src/arm/memory.hpp
  src/arm7/arm7.hpp
  src/arm7/bus.hpp
  src/arm9/arm9.hpp
  src/arm9/bus.hpp
  src/arm9/cp15.hpp
  src/hw/apu/apu.hpp
  src/hw/cart/backup/eeprom.hpp
  src/hw/cart/backup/eeprom512b.hpp
  src/hw/cart/backup/flash.hpp
  src/hw/cart/cart.hpp
  src/hw/dma/dma7.hpp
  src/hw/dma/dma9.hpp
  src/hw/ipc/ipc.hpp
  src/hw/irq/irq.hpp
  src/hw/math/math_engine.hpp
  src/hw/spi/tsc/tsc.hpp
  src/hw/spi/spi.hpp
  src/hw/spi/spi_device.hpp
  src/hw/timer/timer.cpp
  src/hw/video_unit/gpu/fixed_point.hpp
  src/hw/video_unit/gpu/gpu.hpp
  src/hw/video_unit/gpu/matrix_stack.hpp
  src/hw/video_unit/ppu/ppu.hpp
  src/hw/video_unit/ppu/registers.hpp
  src/hw/video_unit/video_unit.hpp
  src/hw/video_unit/vram.hpp
  src/hw/video_unit/vram_region.hpp
  src/interconnect.hpp
  src/scheduler.hpp)

set(HEADERS_PUBLIC
  include/core/device/audio_device.hpp
  include/core/device/video_device.hpp
  include/core/core.hpp)

add_library(duality-core STATIC ${SOURCES} ${HEADERS} ${HEADERS_PUBLIC})
# Why are we linking fmt into the core again?
target_link_libraries(duality-core duality-util xbyak::xbyak fmt)
target_include_directories(duality-core PRIVATE src)
target_include_directories(duality-core PUBLIC include)

