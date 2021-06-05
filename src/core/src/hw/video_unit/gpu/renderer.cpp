/*
 * Copyright (C) 2021 fleroviux
 */

#include "gpu.hpp"

namespace Duality::Core {

auto GPU::SampleTexture(TextureParams const& params, Vector2<Fixed12x4> const& uv) -> Color4 {
  const int size[2] {
    8 << params.size[0],
    8 << params.size[1]
  };

  int coord[2] { uv.x().integer(), uv.y().integer() };

  for (int i = 0; i < 2; i++) {
    if (coord[i] < 0 || coord[i] >= size[i]) {
      int mask = size[i] - 1;
      if (params.repeat[i]) {
        bool odd = (coord[i] >> (3 + params.size[i])) & 1;
        coord[i] &= mask;
        if (params.flip[i] && odd) {
          coord[i] ^= mask;
        }
      } else {
        coord[i] = std::clamp(coord[i], 0, mask);
      }
    }
  }

  auto offset = coord[1] * size[0] + coord[0];
  auto palette_addr = params.palette_base << 4;

  switch (params.format) {
    case TextureParams::Format::None: {
      return Color4{};
    }
    case TextureParams::Format::A3I5: {
      u8  value = vram_texture.Read<u8>(params.address + offset);
      int index = value & 0x1F;
      int alpha = value >> 5;

      auto rgb555 = vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF;
      auto rgb6666 = Color4::from_rgb555(rgb555);
      
      if (params.color0_transparent && index == 0) {
        rgb6666.a() = 0;
      } else {
        rgb6666.a() = (alpha << 3) | alpha; // 3-bit alpha to 6-bit alpha  
      }

      return rgb6666;
    }
    case TextureParams::Format::Palette2BPP: {
      auto index = (vram_texture.Read<u8>(params.address + (offset >> 2)) >> (2 * (offset & 3))) & 3;

      if (params.color0_transparent && index == 0) {
        return Color4{0, 0, 0, 0};
      }
      
      return Color4::from_rgb555(vram_palette.Read<u16>((palette_addr >> 1) + index * sizeof(u16)) & 0x7FFF);
    }
    case TextureParams::Format::Palette4BPP: {
      auto index = (vram_texture.Read<u8>(params.address + (offset >> 1)) >> (4 * (offset & 1))) & 15;
      
      if (params.color0_transparent && index == 0) {
        return Color4{0, 0, 0, 0};
      }
      
      return Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF);
    }
    case TextureParams::Format::Palette8BPP: {
      auto index = vram_texture.Read<u8>(params.address + offset);

      if (params.color0_transparent && index == 0) {
        return Color4{0, 0, 0, 0};
      }

      return Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF);
    }
    case TextureParams::Format::Compressed4x4: {
      auto row_x = coord[0] >> 2;
      auto row_y = coord[1] >> 2;
      auto tile_x = coord[0] & 3;
      auto tile_y = coord[1] & 3;
      auto row_size = size[0] >> 2;

      auto data_address = params.address + (row_y * row_size + row_x) * sizeof(u32) + tile_y;

      auto data_slot_index  = data_address >> 18;
      auto data_slot_offset = data_address & 0x1FFFF;
      auto info_address = 0x20000 + (data_slot_offset >> 1) + (data_slot_index * 0x10000);

      auto data = vram_texture.Read<u8>(data_address); 
      auto info = vram_texture.Read<u16>(info_address);

      auto index = (data >> (tile_x * 2)) & 3;
      auto palette_offset = info & 0x3FFF;
      auto mode = info >> 14;

      palette_addr += palette_offset << 2;

      switch (mode) {
        case 0: {
          if (index == 3) {
            return Color4{0, 0, 0, 0};
          }
          return Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF);
        }
        case 1: {
          if (index == 2) {
            auto color_0 = Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + 0) & 0x7FFF);
            auto color_1 = Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + 2) & 0x7FFF);

            for (uint i = 0; i < 3; i++) {
              color_0[i] = Fixed6{s8((color_0[i].raw() >> 1) + (color_1[i].raw() >> 1))};
            }

            return color_0;
          }
          if (index == 3) {
            return Color4{0, 0, 0, 0};
          }
          return Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF);
        }
        case 2: {
          return Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF);
        }
        default: {
          if (index == 2 || index == 3) {
            int coeff_0 = index == 2 ? 5 : 3;
            int coeff_1 = index == 2 ? 3 : 5;

            auto color_0 = Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + 0) & 0x7FFF);
            auto color_1 = Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + 2) & 0x7FFF);

            for (uint i = 0; i < 3; i++) {
              color_0[i] = Fixed6{s8(((color_0[i].raw() * coeff_0) + (color_1[i].raw() * coeff_1)) >> 3)};
            }

            return color_0;
          }
          return Color4::from_rgb555(vram_palette.Read<u16>(palette_addr + index * sizeof(u16)) & 0x7FFF);
        }
      }
    }
    case TextureParams::Format::A5I3: {
      u8  value = vram_texture.Read<u8>(params.address + offset);
      int index = value & 7;
      int alpha = value >> 3;

      auto rgb555 = vram_palette.Read<u16>((params.palette_base << 4) + index * sizeof(u16)) & 0x7FFF;
      auto rgb6666 = Color4::from_rgb555(rgb555);
      
      if (params.color0_transparent && index == 0) {
        rgb6666.a() = 0;
      } else {
        rgb6666.a() = (alpha << 1) | (alpha >> 4); // 5-bit alpha to 6-bit alpha  
      }

      return rgb6666;
    }
    case TextureParams::Format::Direct: {
      auto color = vram_texture.Read<u16>(params.address + offset * sizeof(u16));

      if (color & 0x8000) {
        return Color4{0, 0, 0, 0};
      }

      return Color4::from_rgb555(color);
    }
  };

  return Color4{};
}

void GPU::Render() {
  return;
  
  for (uint i = 0; i < 256 * 192; i++) {
    draw_buffer[i] = Color4{0, 0, 0, 0};
    depth_buffer[i] = 0xFFFFFF;
  }

  for (int i = 0; i < polygon[gx_buffer_id ^ 1].count; i++) {
    Polygon const& poly = polygon[gx_buffer_id ^ 1].data[i];

    struct Point {
      s32 x;
      s32 y;
      u32 depth;
      Vertex const* vertex;
    } points[poly.count];

    int start = 0;
    s32 y_min = 256;
    s32 y_max = 0;

    for (int j = 0; j < poly.count; j++) {
      auto const& vert = vertex[gx_buffer_id ^ 1].data[poly.indices[j]];
      auto& point = points[j];

      // TODO: use the provided viewport configuration.
      // TODO: support w-Buffering mode.
      point.x = ( vert.position.x() / vert.position.w() * Fixed20x12::from_int(128)).integer() + 128;
      point.y = (-vert.position.y() / vert.position.w() * Fixed20x12::from_int( 96)).integer() +  96;
      point.depth = (((s64(vert.position.z().raw()) << 14) / vert.position.w().raw()) + 0x3FFF) << 9;
      point.vertex = &vert;

      // Pick the first vertex with the lowest y-Coordinate as the start node.
      // Also update the minimum y-Coordinate.
      if (point.y < y_min) {
        y_min = point.y;
        start = j;
      }

      // Update the maximum y-Coordinate
      if (point.y > y_max) {
        y_max = point.y;
      }
    }

    int s[2];
    int e[2];

    // first edge (CW)
    s[0] = start;
    e[0] = start == (poly.count - 1) ? 0 : (start + 1);

    // second edge (CCW)
    s[1] = start;
    e[1] = start == 0 ? (poly.count - 1) : (start - 1);

    for (s32 y = y_min; y <= y_max; y++) {
      struct Span {
        s32 x[2];
        s32 w[2];
        u32 depth[2];
        Vector2<Fixed12x4> uv[2];
        Color4 color[2];
      } span;

      int a = 0; // left edge index
      int b = 1; // right edge index

      if (points[e[0]].y <= y) {
        s[0] = e[0];
        if (++e[0] == poly.count)
          e[0] = 0;
      }

      if (points[e[1]].y <= y) {
        s[1] = e[1];
        if (--e[1] == -1)
          e[1] = poly.count - 1;
      }

      auto lerp = [](s32 a, s32 b, s32 t, s32 t_max, s32 w_a = 1 << 12, s32 w_b = 1 << 12) {
        // // If both w-Coordinates are same then division-by-zero should cancel out.
        // if (w_a == w_b) {
        //   w_a = 1;
        //   w_b = 1;
        // }

        // // If only one w-Coordinate is zero, then the other will be biased to zero.
        // if (w_a == 0) return a;
        // if (w_b == 0) return b;

        if (w_a != 0 && w_b != 0) {
          auto x = (s64(t_max - t) << 24) / w_a;
          auto y = (s64(t) << 24) / w_b;
          auto max = x + y;

          if (max != 0) {
            return s32((a * x + b * y) / max);
          }
        }

        return a;
      };

      for (int j = 0; j < 2; j++) {
        auto t = y - points[s[j]].y;
        auto t_max = points[e[j]].y - points[s[j]].y;
        auto w0 = points[s[j]].vertex->position.w().raw();
        auto w1 = points[e[j]].vertex->position.w().raw();

        // TODO: interpolation of w0 and w1 can be optimized.
        span.x[j] = lerp(points[s[j]].x, points[e[j]].x, t, t_max);
        span.w[j] = lerp(w0, w1, t, t_max, w0, w1);
        span.depth[j] = lerp(points[s[j]].depth, points[e[j]].depth, t, t_max);

        for (int k = 0; k < 2; k++) {
          span.uv[j][k] = Fixed12x4{s16(lerp(
            points[s[j]].vertex->uv[k].raw(),
            points[e[j]].vertex->uv[k].raw(), t, t_max, w0, w1))};
        }

        for (int k = 0; k < 3; k++) {
          span.color[j][k] = Fixed6{s8(lerp(
            points[s[j]].vertex->color[k].raw(),
            points[e[j]].vertex->color[k].raw(), t, t_max, w0, w1))};
        }
      }

      if (span.x[0] > span.x[1]) {
        a ^= 1;
        b ^= 1;
      }

      if (y >= 0 && y <= 191) {
        for (s32 x = span.x[a]; x <= span.x[b]; x++) {
          if (x >= 0 && x <= 255) {
            auto t = x - span.x[a];
            auto t_max = span.x[b] - span.x[a];
            auto index = y * 256 + x;

            auto depth_old = depth_buffer[index];
            auto depth_new = lerp(span.depth[a], span.depth[b], t, t_max);

            if (poly.params.depth_test == PolygonParams::DepthTest::Less) {
              if (depth_new >= depth_old)
                continue;
            } else {
              if (std::abs(s32(depth_new) - s32(depth_old)) > 0x200)
                continue;
            }

            Vector2<Fixed12x4> uv;
            Color4 vertex_color;

            for (int j = 0; j < 2; j++) {
              uv[j] = Fixed12x4{s16(lerp(span.uv[a][j].raw(), span.uv[b][j].raw(), t, t_max, span.w[a], span.w[b]))};
            }

            for (int j = 0; j < 3; j++) {
              vertex_color[j] = Fixed6{s8(lerp(
                span.color[a][j].raw(),
                span.color[b][j].raw(), t, t_max, span.w[a], span.w[b]))};
            }

            bool do_blend = disp3dcnt.enable_alpha_blend && draw_buffer[index].a() != 0;

            auto color = vertex_color;

            // TODO: alpha=0 means wireframe mode. handle that.
            color.a() = (poly.params.alpha << 1) | (poly.params.alpha >> 4); 

            if (disp3dcnt.enable_textures) {
              auto texel = SampleTexture(poly.texture_params, uv);
              // TODO: perform alpha test
              if (texel.a() == 0) {
                continue;
              }
              color *= texel;
            }

            if (do_blend) {
              auto a0 = color.a();
              auto a1 = Fixed6{63} - a0;
              for (uint j = 0; j < 3; j++)
                color[j] = color[j] * a0 + draw_buffer[index][j] * a1;
              color.a() = std::max(color.a(), draw_buffer[index].a());
            }

            draw_buffer[index] = color;
            // TODO: figure out what the actual logic is supposed to be.
            //if (color.a() == 63 || poly.params.enable_translucent_depth_write) {
              depth_buffer[index] = depth_new;
            //}
          }
        }
      }
    }
  }
}

} // namespace Duality::Core
