/*
 * Copyright (C) 2020 fleroviux
 */

#pragma once

#include <util/integer.hpp>
#include <util/meta.hpp>
#include <util/fifo.hpp>
#include <memory>
#include <vector>

#include "hw/dma/dma9.hpp"
#include "hw/irq/irq.hpp"
#include "hw/video_unit/vram.hpp"
#include "scheduler.hpp"
#include "color.hpp"
#include "matrix_stack.hpp"

namespace Duality::Core {

/// 3D graphics processing unit (GPU)
struct GPU {
  GPU(Scheduler& scheduler, IRQ& irq9, DMA9& dma9, VRAM const& vram);
  
  void Reset();
  void WriteGXFIFO(u32 value);
  void WriteCommandPort(uint port, u32 value);

  template<typename T>
  auto ReadClipMatrix(u32 offset) -> T {
    static_assert(common::is_one_of_v<T, u8, u16, u32>, "T must be u8, u16 or u32");
    if (offset >= 64) {
      UNREACHABLE;
    }
    auto row = (offset >> 2) & 3;
    auto col =  offset >> 4;
    return static_cast<T>(clip_matrix[col][row].raw() >> ((offset & 3) * 8));
  }

  void Render();
  auto GetOutput() -> Color4 const* { return &draw_buffer[0]; }

  struct DISP3DCNT {
    auto ReadByte (uint offset) -> u8;
    void WriteByte(uint offset, u8 value);
    
  private:
    friend struct Duality::Core::GPU;
    
    enum class Shading {
      Toon = 0,
      Highlight = 1
    };
    
    enum class FogMode {
      ColorAndAlpha = 0,
      Alpha = 1
    };
  
    bool enable_textures = false;
    Shading shading_mode = Shading::Toon;
    bool enable_alpha_test = false;
    bool enable_alpha_blend = false;
    bool enable_antialias = false;
    bool enable_edge_marking = false;
    FogMode fog_mode = FogMode::ColorAndAlpha;
    bool enable_fog = false;
    int fog_depth_shift = 0;
    bool rdlines_underflow = false;
    bool polyvert_ram_overflow = false;
    bool enable_rear_bitmap = false;
  } disp3dcnt;
  
  struct GXSTAT {
    // TODO: implement (box)test and matrix stack bits.
    GXSTAT(GPU& gpu) : gpu(gpu) {}
        
    auto ReadByte (uint offset) -> u8;
    void WriteByte(uint offset, u8 value);
    
  private:
    friend struct Duality::Core::GPU;
    
    enum class IRQMode {
      Never = 0,
      LessThanHalfFull = 1,
      Empty = 2,
      Reserved = 3
    };
    
    bool gx_busy = false;
    IRQMode cmd_fifo_irq = IRQMode::Never;

    GPU& gpu;
  } gxstat { *this };

private:
  enum class MatrixMode {
    Projection = 0,
    Modelview = 1,
    Simultaneous = 2,
    Texture = 3
  };
  
  struct CmdArgPack {
    u8 command = 0;
    u32 argument = 0;
  };

  struct PolygonParams {
    bool enable_light[4] { false };
  
    enum class Mode {
      Modulation,
      Decal,
      Shaded,
      Shadow
    } mode = Mode::Modulation;

    bool render_back_side = false;
    bool render_front_side = false;
    bool enable_translucent_depth_write = false; 
    bool render_far_plane_polys = false;
    bool render_1dot_depth_tested = false;

    enum class DepthTest {
      Less,
      Equal
    } depth_test = DepthTest::Less;

    bool enable_fog = false;
    int alpha = 0;
    int polygon_id = 0;
  };

  struct TextureParams {
    u32 address = 0;
    bool repeat[2] { false };
    bool flip[2] { false };
    int size[2] { 0 };

    enum class Format {
      None,
      A3I5,
      Palette2BPP,
      Palette4BPP,
      Palette8BPP,
      Compressed4x4,
      A5I3,
      Direct
    } format = Format::None;

    bool color0_transparent = false;

    enum class Transform {
      None,
      TexCoord,
      Normal,
      Position
    } transform = Transform::TexCoord;

    u16 palette_base = 0;
  };

  struct Vertex {
    Vector4<Fixed20x12> position;
    Color4 color;
    Vector2<Fixed12x4> uv;
    // ...
  };

  struct Polygon {
    int count;
    int indices[10];
    // TODO: unify polygon and texture parameters?
    PolygonParams params;
    TextureParams texture_params;
  };

  void Enqueue(CmdArgPack pack);
  auto Dequeue() -> CmdArgPack;
  void ProcessCommands();
  void CheckGXFIFO_IRQ();
  void UpdateClipMatrix();

  auto DequeueMatrix4x4() -> Matrix4<Fixed20x12> {
    Matrix4<Fixed20x12> mat;
    for (int col = 0; col < 4; col++) {
      for (int row = 0; row < 4; row++) {
        mat[col][row] = Dequeue().argument;
      }
    }
    return mat;
  }
  
  auto DequeueMatrix4x3() -> Matrix4<Fixed20x12> {
    Matrix4<Fixed20x12> mat;
    for (int col = 0; col < 4; col++) {
      for (int row = 0; row < 3; row++) {
        mat[col][row] = Dequeue().argument;
      }
    }
    mat[3][3] = Fixed20x12::from_int(1);
    return mat;
  }

  auto DequeueMatrix3x3() -> Matrix4<Fixed20x12> {
    Matrix4<Fixed20x12> mat;
    for (int col = 0; col < 3; col++) {
      for (int row = 0; row < 3; row++) {
        mat[col][row] = Dequeue().argument;
      }
    }
    mat[3][3] = Fixed20x12::from_int(1);
    return mat;
  }

  void AddVertex(Vector4<Fixed20x12> const& position);
  auto ClipPolygon(std::vector<Vertex> const& vertices, bool quadstrip) -> std::vector<Vertex>;
  auto SampleTexture(TextureParams const& params, Vector2<Fixed12x4> const& uv) -> Color4;

  /// Matrix commands
  void CMD_SetMatrixMode();
  void CMD_PushMatrix();
  void CMD_PopMatrix();
  void CMD_StoreMatrix();
  void CMD_RestoreMatrix();
  void CMD_LoadIdentity();
  void CMD_LoadMatrix4x4();
  void CMD_LoadMatrix4x3();
  void CMD_MatrixMultiply4x4();
  void CMD_MatrixMultiply4x3();
  void CMD_MatrixMultiply3x3();
  void CMD_MatrixScale();
  void CMD_MatrixTranslate();

  /// Vertex submission commands
  void CMD_SetColor();
  void CMD_SetNormal();
  void CMD_SetUV();
  void CMD_SubmitVertex_16();
  void CMD_SubmitVertex_10();
  void CMD_SubmitVertex_XY();
  void CMD_SubmitVertex_XZ();
  void CMD_SubmitVertex_YZ();
  void CMD_SubmitVertex_Offset();
  void CMD_SetPolygonAttributes();
  void CMD_SetTextureParameters();
  void CMD_SetPaletteBase();

  /// Lighting commands
  void CMD_SetMaterialColor0();
  void CMD_SetMaterialColor1();
  void CMD_SetLightVector();
  void CMD_SetLightColor();

  /// Vertex list commands
  void CMD_BeginVertexList();
  void CMD_EndVertexList();

  void CMD_SwapBuffers();

  bool in_vertex_list;
  bool is_quad;
  bool is_strip;
  bool is_first;

  struct VertexRAM {
    int count = 0;
    Vertex data[6144];
  } vertex[2];

  struct PolygonRAM {
    int count = 0;
    Polygon data[2048];
  } polygon[2];

  /// ID of the buffer the geometry engine currently writes into.
  int gx_buffer_id = 0;

  /// Temporary vertex buffer for the primitive being submitted at the moment.
  /// TODO: figure out a name that doesn't completely suck.
  std::vector<Vertex> vertices;

  /// Untransformed vertex from the previous vertex submission command.
  Vector4<Fixed20x12> position_old;

  /// Current vertex color
  Color4 vertex_color;

  /// Current vertex UV
  Vector2<Fixed12x4> vertex_uv;
  Vector2<Fixed12x4> vertex_uv_source;

  /// Current polygon parameters
  PolygonParams poly_params;

  /// Current texture parameters
  TextureParams texture_params;

  struct Light {
    Vector3<Fixed20x12> direction;
    Vector3<Fixed20x12> halfway;
    Color4 color;
  } lights[4];

  struct Material {
    Color4 diffuse;
    Color4 ambient;
    Color4 specular;
    Color4 emissive;

    bool enable_shinyness_table = false;
  } material;

  /// GPU texture and texture palette data
  Region<4, 131072> const& vram_texture { 3 };
  Region<8> const& vram_palette { 7 };

  Scheduler& scheduler;
  IRQ& irq9;
  DMA9& dma9;
  common::FIFO<CmdArgPack, 256> gxfifo;
  common::FIFO<CmdArgPack, 4> gxpipe;
  
  Color4 draw_buffer[256 * 192];
  u32 depth_buffer[256 * 192];

  /// Packed command processing
  u32 packed_cmds;
  int packed_args_left;
  
  /// Matrix stacks
  MatrixMode matrix_mode;
  MatrixStack< 1> projection;
  MatrixStack<31> modelview;
  MatrixStack<31> direction;
  MatrixStack< 1> texture;
  Matrix4<Fixed20x12> clip_matrix;

  Scheduler::Event* cmd_event = nullptr;
};

} // namespace Duality::Core
