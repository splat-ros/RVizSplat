#include "gsplat_rviz_plugin/splat_loaders/ply_loader.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <istream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace gsplat_rviz_plugin
{
namespace
{

struct PropEntry
{
  std::string type;
  size_t byte_offset;
  size_t value_index;
};

static size_t plyTypeBytes(const std::string & t)
{
  if (t == "float"  || t == "float32" || t == "int"  || t == "int32" ||
      t == "uint"   || t == "uint32") return 4;
  if (t == "double" || t == "float64" || t == "int64" || t == "uint64") return 8;
  if (t == "short"  || t == "int16"   || t == "ushort"|| t == "uint16") return 2;
  if (t == "char"   || t == "int8"    || t == "uchar" || t == "uint8") return 1;
  return 4;
}

static float readBytesAsFloat(const char * ptr, const std::string & type)
{
  if (type == "float" || type == "float32") {
    float v; std::memcpy(&v, ptr, 4); return v;
  }
  if (type == "double" || type == "float64") {
    double v; std::memcpy(&v, ptr, 8); return static_cast<float>(v);
  }
  if (type == "uchar"  || type == "uint8")  { uint8_t  v; std::memcpy(&v, ptr, 1); return static_cast<float>(v); }
  if (type == "char"   || type == "int8")   { int8_t   v; std::memcpy(&v, ptr, 1); return static_cast<float>(v); }
  if (type == "ushort" || type == "uint16") { uint16_t v; std::memcpy(&v, ptr, 2); return static_cast<float>(v); }
  if (type == "short"  || type == "int16")  { int16_t  v; std::memcpy(&v, ptr, 2); return static_cast<float>(v); }
  if (type == "int"    || type == "int32")  { int32_t  v; std::memcpy(&v, ptr, 4); return static_cast<float>(v); }
  if (type == "uint"   || type == "uint32") { uint32_t v; std::memcpy(&v, ptr, 4); return static_cast<float>(v); }
  float v; std::memcpy(&v, ptr, 4); return v;
}

static float binGet(
  const std::string & name,
  const std::unordered_map<std::string, PropEntry> & props,
  const char * vptr,
  float def = 0.0f)
{
  auto it = props.find(name);
  return (it == props.end()) ? def : readBytesAsFloat(vptr + it->second.byte_offset, it->second.type);
}

static float ascGet(
  const std::string & name,
  const std::unordered_map<std::string, PropEntry> & props,
  const std::vector<float> & vals,
  float def = 0.0f)
{
  auto it = props.find(name);
  return (it == props.end() || it->second.value_index >= vals.size()) ? def : vals[it->second.value_index];
}

static inline float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
static inline float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

static SplatGPU buildSplat(
  float x, float y, float z,
  float fdc0, float fdc1, float fdc2,
  float raw_opacity,
  float ls0, float ls1, float ls2,
  float qw, float qx, float qy, float qz)
{
  SplatGPU g{};

  g.center[0] = x; g.center[1] = y; g.center[2] = z;
  g.alpha = sigmoid(raw_opacity);

  const float sx = std::exp(ls0);
  const float sy = std::exp(ls1);
  const float sz = std::exp(ls2);

  float qnorm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (qnorm > 1e-6f) { qw /= qnorm; qx /= qnorm; qy /= qnorm; qz /= qnorm; }

  const float R[3][3] = {
    {1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)},
    {    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)},
    {    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)}
  };
  const float s2[3] = {sx*sx, sy*sy, sz*sz};

  // Σ = R·diag(s²)·Rᵀ  →  {v11,v12,v13,v22,v23,v33}
  float cov[6] = {};
  const int idx[3][3] = {{0,1,2},{-1,3,4},{-1,-1,5}};
  for (int r = 0; r < 3; ++r)
    for (int c = r; c < 3; ++c) {
      float val = 0.0f;
      for (int k = 0; k < 3; ++k) val += R[r][k] * s2[k] * R[c][k];
      cov[idx[r][c]] = val;
    }

  g.covA[0] = cov[0]; g.covA[1] = cov[1]; g.covA[2] = cov[2];
  g.covB[0] = cov[3]; g.covB[1] = cov[4]; g.covB[2] = cov[5];

  // DC SH term — higher-order filled by caller
  g.sh[0][0] = fdc0; g.sh[0][1] = fdc1; g.sh[0][2] = fdc2; g.sh[0][3] = 0.0f;

  (void)clamp01(0.0f);  // silence unused-function warning

  return g;
}

}  // anonymous namespace

static std::vector<SplatGPU> loadPlyStream(
  std::istream & file, std::string & error_msg, int & sh_degree)
{
  error_msg.clear();
  sh_degree = 0;

  std::string line;
  std::getline(file, line);
  if (line.rfind("ply", 0) != 0) { error_msg = "Not a PLY file"; return {}; }

  enum class Format { Unknown, BinaryLE, ASCII } format = Format::Unknown;
  int vertex_count = 0;
  std::vector<std::pair<std::string, std::string>> prop_list;
  bool in_vertex = false;

  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line == "end_header") break;
    std::istringstream iss(line);
    std::string tok; iss >> tok;
    if (tok == "format") {
      std::string fmt; iss >> fmt;
      if (fmt == "binary_little_endian") format = Format::BinaryLE;
      else if (fmt == "ascii")           format = Format::ASCII;
      else { error_msg = "Unsupported PLY format: " + fmt; return {}; }
    } else if (tok == "element") {
      std::string elem; iss >> elem;
      in_vertex = (elem == "vertex");
      if (in_vertex) iss >> vertex_count;
    } else if (tok == "property" && in_vertex) {
      std::string type, name; iss >> type;
      if (type == "list") continue;
      iss >> name;
      prop_list.push_back({name, type});
    }
  }

  if (format == Format::Unknown) { error_msg = "PLY format not specified"; return {}; }
  if (vertex_count <= 0)         { error_msg = "PLY vertex count is 0 or missing"; return {}; }

  std::unordered_map<std::string, PropEntry> props;
  size_t byte_off = 0;
  for (size_t i = 0; i < prop_list.size(); ++i) {
    const auto & [n, t] = prop_list[i];
    props[n] = PropEntry{t, byte_off, i};
    byte_off += plyTypeBytes(t);
  }
  const size_t stride = byte_off;

  if (!props.count("x") || !props.count("y") || !props.count("z")) {
    error_msg = "PLY missing x/y/z properties"; return {};
  }

  int num_rest_total = 0;
  for (const auto & [n, t] : prop_list)
    if (n.rfind("f_rest_", 0) == 0) ++num_rest_total;
  const int nrc  = num_rest_total / 3;           // rest per channel
  const int tpc  = 1 + nrc;                      // total per channel
  sh_degree = (tpc >= 16) ? 3 : (tpc >= 9) ? 2 : (tpc >= 4) ? 1 : 0;

  std::vector<SplatGPU> splats;
  splats.reserve(static_cast<size_t>(vertex_count));

  auto fillSH = [&](SplatGPU & g, auto get) {
    // Higher-order SH (PLY is channel-major → reorder to coefficient-major)
    for (int ci = 0; ci < nrc && (ci + 1) < 16; ++ci) {
      g.sh[ci+1][0] = get("f_rest_" + std::to_string(ci));
      g.sh[ci+1][1] = get("f_rest_" + std::to_string(ci + nrc));
      g.sh[ci+1][2] = get("f_rest_" + std::to_string(ci + 2*nrc));
      g.sh[ci+1][3] = 0.0f;
    }
  };

  if (format == Format::BinaryLE) {
    const size_t total = stride * static_cast<size_t>(vertex_count);
    std::vector<char> data(total);
    if (!file.read(data.data(), static_cast<std::streamsize>(total))) {
      error_msg = "Unexpected end of file"; return {};
    }
    for (int i = 0; i < vertex_count; ++i) {
      const char * vp = data.data() + i * stride;
      auto get = [&](const std::string & n, float d = 0.0f) {
        return binGet(n, props, vp, d);
      };
      SplatGPU g = buildSplat(
        get("x"), get("y"), get("z"),
        get("f_dc_0"), get("f_dc_1"), get("f_dc_2"),
        get("opacity", 10.0f),
        get("scale_0"), get("scale_1"), get("scale_2"),
        get("rot_0", 1.0f), get("rot_1"), get("rot_2"), get("rot_3"));
      fillSH(g, get);
      splats.push_back(g);
    }
  } else {
    for (int i = 0; i < vertex_count; ++i) {
      if (!std::getline(file, line)) {
        error_msg = "Unexpected end of ASCII data at vertex " + std::to_string(i);
        return {};
      }
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::istringstream vss(line);
      std::vector<float> vals;
      float v; while (vss >> v) vals.push_back(v);
      if (vals.size() < prop_list.size()) {
        error_msg = "Too few values at ASCII vertex " + std::to_string(i); return {};
      }
      auto get = [&](const std::string & n, float d = 0.0f) {
        return ascGet(n, props, vals, d);
      };
      SplatGPU g = buildSplat(
        get("x"), get("y"), get("z"),
        get("f_dc_0"), get("f_dc_1"), get("f_dc_2"),
        get("opacity", 10.0f),
        get("scale_0"), get("scale_1"), get("scale_2"),
        get("rot_0", 1.0f), get("rot_1"), get("rot_2"), get("rot_3"));
      fillSH(g, get);
      splats.push_back(g);
    }
  }

  return splats;
}

std::vector<SplatGPU> loadPly(
  const std::string & path, std::string & error_msg, int & sh_degree)
{
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    error_msg = "Cannot open file: " + path;
    sh_degree = 0;
    return {};
  }
  return loadPlyStream(file, error_msg, sh_degree);
}

std::vector<SplatGPU> loadPlyBytes(
  const std::vector<uint8_t> & bytes, std::string & error_msg, int & sh_degree)
{
  if (bytes.empty()) {
    return loadPlyBytes(std::string(), error_msg, sh_degree);
  }
  const std::string payload(reinterpret_cast<const char *>(bytes.data()), bytes.size());
  return loadPlyBytes(payload, error_msg, sh_degree);
}

std::vector<SplatGPU> loadPlyBytes(
  const std::string & bytes, std::string & error_msg, int & sh_degree)
{
  std::istringstream stream(bytes, std::ios::binary);
  return loadPlyStream(stream, error_msg, sh_degree);
}

}  // namespace gsplat_rviz_plugin
