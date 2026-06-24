#include "gsplat_rviz_plugin/splat_loaders/compact_decoder.hpp"

#include <cmath>
#include <cstring>

namespace gsplat_rviz_plugin
{
namespace
{

constexpr size_t kCompactStride = 32;
constexpr float kShC0 = 0.28209479177387814f;

uint16_t readLeU16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) |
         static_cast<uint16_t>(p[1] << 8);
}

float readLeF32(const uint8_t * p)
{
  uint32_t bits = static_cast<uint32_t>(p[0]) |
                  (static_cast<uint32_t>(p[1]) << 8) |
                  (static_cast<uint32_t>(p[2]) << 16) |
                  (static_cast<uint32_t>(p[3]) << 24);
  float v;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

float halfToFloat(uint16_t h)
{
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000u) << 16;
  uint32_t exp = (h >> 10) & 0x1Fu;
  uint32_t mant = h & 0x03FFu;

  uint32_t bits = 0;
  if (exp == 0) {
    if (mant == 0) {
      bits = sign;
    } else {
      exp = 1;
      while ((mant & 0x0400u) == 0) {
        mant <<= 1;
        --exp;
      }
      mant &= 0x03FFu;
      bits = sign | ((exp + 112u) << 23) | (mant << 13);
    }
  } else if (exp == 31) {
    bits = sign | 0x7F800000u | (mant << 13);
  } else {
    bits = sign | ((exp + 112u) << 23) | (mant << 13);
  }

  float v;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

void rotationMatrix(
  const geometry_msgs::msg::Quaternion & q_msg,
  float r[3][3])
{
  float x = static_cast<float>(q_msg.x);
  float y = static_cast<float>(q_msg.y);
  float z = static_cast<float>(q_msg.z);
  float w = static_cast<float>(q_msg.w);
  const float norm = std::sqrt(x*x + y*y + z*z + w*w);
  if (norm > 1e-6f) {
    x /= norm; y /= norm; z /= norm; w /= norm;
  } else {
    x = y = z = 0.0f; w = 1.0f;
  }

  r[0][0] = 1.0f - 2.0f * (y*y + z*z);
  r[0][1] = 2.0f * (x*y - w*z);
  r[0][2] = 2.0f * (x*z + w*y);
  r[1][0] = 2.0f * (x*y + w*z);
  r[1][1] = 1.0f - 2.0f * (x*x + z*z);
  r[1][2] = 2.0f * (y*z - w*x);
  r[2][0] = 2.0f * (x*z - w*y);
  r[2][1] = 2.0f * (y*z + w*x);
  r[2][2] = 1.0f - 2.0f * (x*x + y*y);
}

}  // namespace

std::vector<SplatGPU> decodeCompactDcFp16CovRgbaV1(
  const std::vector<uint8_t> & bytes,
  std::string & error_msg)
{
  error_msg.clear();
  if ((bytes.size() % kCompactStride) != 0) {
    error_msg = "compact_dc_fp16_cov_rgba_v1 payload size is not a multiple of 32 bytes.";
    return {};
  }

  std::vector<SplatGPU> splats;
  splats.reserve(bytes.size() / kCompactStride);

  for (size_t offset = 0; offset < bytes.size(); offset += kCompactStride) {
    const uint8_t * p = bytes.data() + offset;
    SplatGPU g{};

    g.center[0] = readLeF32(p + 0);
    g.center[1] = readLeF32(p + 4);
    g.center[2] = readLeF32(p + 8);

    g.covA[0] = halfToFloat(readLeU16(p + 12));
    g.covA[1] = halfToFloat(readLeU16(p + 14));
    g.covA[2] = halfToFloat(readLeU16(p + 16));
    g.covB[0] = halfToFloat(readLeU16(p + 18));
    g.covB[1] = halfToFloat(readLeU16(p + 20));
    g.covB[2] = halfToFloat(readLeU16(p + 22));

    g.sh[0][0] = (static_cast<float>(p[24]) / 255.0f - 0.5f) / kShC0;
    g.sh[0][1] = (static_cast<float>(p[25]) / 255.0f - 0.5f) / kShC0;
    g.sh[0][2] = (static_cast<float>(p[26]) / 255.0f - 0.5f) / kShC0;
    g.sh[0][3] = 0.0f;
    g.alpha = static_cast<float>(p[27]) / 255.0f;

    splats.push_back(g);
  }

  return splats;
}

void applyMapFromSourceTransform(
  std::vector<SplatGPU> & splats,
  const geometry_msgs::msg::Transform & transform)
{
  float r[3][3];
  rotationMatrix(transform.rotation, r);
  const float t[3] = {
    static_cast<float>(transform.translation.x),
    static_cast<float>(transform.translation.y),
    static_cast<float>(transform.translation.z)};

  for (auto & g : splats) {
    const float c[3] = {g.center[0], g.center[1], g.center[2]};
    for (int i = 0; i < 3; ++i) {
      g.center[i] = r[i][0] * c[0] + r[i][1] * c[1] + r[i][2] * c[2] + t[i];
    }

    const float s[3][3] = {
      {g.covA[0], g.covA[1], g.covA[2]},
      {g.covA[1], g.covB[0], g.covB[1]},
      {g.covA[2], g.covB[1], g.covB[2]}};
    float rs[3][3] = {};
    float out[3][3] = {};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          rs[i][j] += r[i][k] * s[k][j];
        }
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          out[i][j] += rs[i][k] * r[j][k];
        }
      }
    }

    g.covA[0] = out[0][0];
    g.covA[1] = out[0][1];
    g.covA[2] = out[0][2];
    g.covB[0] = out[1][1];
    g.covB[1] = out[1][2];
    g.covB[2] = out[2][2];
  }
}

}  // namespace gsplat_rviz_plugin
