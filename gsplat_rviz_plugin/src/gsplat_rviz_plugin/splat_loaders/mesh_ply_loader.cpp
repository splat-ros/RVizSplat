#include "gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.hpp"

#include <cstring>
#include <fstream>
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
  size_t      byte_offset;
  size_t      value_index;
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
  if (type == "float"  || type == "float32") { float    v; std::memcpy(&v, ptr, 4); return v; }
  if (type == "double" || type == "float64") { double   v; std::memcpy(&v, ptr, 8); return static_cast<float>(v); }
  if (type == "uchar"  || type == "uint8")   { uint8_t  v; std::memcpy(&v, ptr, 1); return static_cast<float>(v); }
  if (type == "char"   || type == "int8")    { int8_t   v; std::memcpy(&v, ptr, 1); return static_cast<float>(v); }
  if (type == "ushort" || type == "uint16")  { uint16_t v; std::memcpy(&v, ptr, 2); return static_cast<float>(v); }
  if (type == "short"  || type == "int16")   { int16_t  v; std::memcpy(&v, ptr, 2); return static_cast<float>(v); }
  if (type == "int"    || type == "int32")   { int32_t  v; std::memcpy(&v, ptr, 4); return static_cast<float>(v); }
  if (type == "uint"   || type == "uint32")  { uint32_t v; std::memcpy(&v, ptr, 4); return static_cast<float>(v); }
  float v; std::memcpy(&v, ptr, 4); return v;
}

static uint32_t readBytesAsUInt(const char * ptr, const std::string & type)
{
  if (type == "uchar"  || type == "uint8")  { uint8_t  v; std::memcpy(&v, ptr, 1); return v; }
  if (type == "char"   || type == "int8")   { int8_t   v; std::memcpy(&v, ptr, 1); return static_cast<uint32_t>(v); }
  if (type == "ushort" || type == "uint16") { uint16_t v; std::memcpy(&v, ptr, 2); return v; }
  if (type == "short"  || type == "int16")  { int16_t  v; std::memcpy(&v, ptr, 2); return static_cast<uint32_t>(v); }
  if (type == "uint"   || type == "uint32") { uint32_t v; std::memcpy(&v, ptr, 4); return v; }
  if (type == "int"    || type == "int32")  { int32_t  v; std::memcpy(&v, ptr, 4); return static_cast<uint32_t>(v); }
  uint32_t v; std::memcpy(&v, ptr, 4); return v;
}

}  // namespace

MeshSplatData loadMeshPly(const std::string & path)
{
  MeshSplatData out;

  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) { out.error = "Cannot open file: " + path; return out; }

  std::string line;
  std::getline(file, line);
  if (line.rfind("ply", 0) != 0) { out.error = "Not a PLY file"; return out; }

  enum class Format { Unknown, BinaryLE, ASCII } format = Format::Unknown;

  // Per-element header state.
  int  vertex_count = 0;
  int  face_count   = 0;
  std::vector<std::pair<std::string, std::string>> vertex_props;  // (name, type)
  // Face list property types: e.g. "property list uchar int vertex_indices".
  std::string face_count_type = "uchar";
  std::string face_index_type = "int";

  enum class Elem { None, Vertex, Face } cur = Elem::None;

  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line == "end_header") break;
    std::istringstream iss(line);
    std::string tok; iss >> tok;
    if (tok == "format") {
      std::string fmt; iss >> fmt;
      if (fmt == "binary_little_endian") format = Format::BinaryLE;
      else if (fmt == "ascii")           format = Format::ASCII;
      else { out.error = "Unsupported PLY format: " + fmt; return out; }
    } else if (tok == "element") {
      std::string elem; iss >> elem;
      if (elem == "vertex")    { cur = Elem::Vertex; iss >> vertex_count; }
      else if (elem == "face") { cur = Elem::Face;   iss >> face_count; }
      else                     { cur = Elem::None; }
    } else if (tok == "property") {
      std::string type; iss >> type;
      if (type == "list") {
        std::string count_type, item_type, name;
        iss >> count_type >> item_type >> name;
        if (cur == Elem::Face) { face_count_type = count_type; face_index_type = item_type; }
      } else if (cur == Elem::Vertex) {
        std::string name; iss >> name;
        vertex_props.push_back({name, type});
      }
    }
  }

  if (format == Format::Unknown) { out.error = "PLY format not specified"; return out; }
  if (vertex_count <= 0)         { out.error = "PLY vertex count is 0 or missing"; return out; }
  if (face_count   <= 0)         { out.error = "PLY has no faces — not a mesh splat"; return out; }

  std::unordered_map<std::string, PropEntry> props;
  size_t byte_off = 0;
  for (size_t i = 0; i < vertex_props.size(); ++i) {
    const auto & [n, t] = vertex_props[i];
    props[n] = PropEntry{t, byte_off, i};
    byte_off += plyTypeBytes(t);
  }
  const size_t stride = byte_off;

  if (!props.count("x") || !props.count("y") || !props.count("z")) {
    out.error = "PLY missing x/y/z properties"; return out;
  }

  int num_rest = 0;
  for (const auto & [n, t] : vertex_props)
    if (n.rfind("f_rest_", 0) == 0) ++num_rest;
  const int nrc = num_rest / 3;          // rest coeffs per channel
  const int tpc = 1 + nrc;               // total coeffs per channel (incl DC)
  out.sh_degree = (tpc >= 16) ? 3 : (tpc >= 9) ? 2 : (tpc >= 4) ? 1 : 0;
  out.sh_stride = (out.sh_degree + 1) * (out.sh_degree + 1);
  const int sh_stride = out.sh_stride;

  out.positions.resize(static_cast<size_t>(vertex_count) * 3);
  out.sh.assign(static_cast<size_t>(vertex_count) * sh_stride * 3, 0.0f);

  // ── Vertices ────────────────────────────────────────────────────────────
  auto store_vertex = [&](int i, auto get) {
    out.positions[i * 3 + 0] = get("x");
    out.positions[i * 3 + 1] = get("y");
    out.positions[i * 3 + 2] = get("z");

    float * sh = out.sh.data() + static_cast<size_t>(i) * sh_stride * 3;
    // DC term (coeff 0).
    sh[0] = get("f_dc_0");
    sh[1] = get("f_dc_1");
    sh[2] = get("f_dc_2");
    // Higher-order: PLY stores f_rest channel-major, reorder to coeff-major.
    for (int ci = 0; ci < nrc && (ci + 1) < sh_stride; ++ci) {
      sh[(ci + 1) * 3 + 0] = get("f_rest_" + std::to_string(ci));
      sh[(ci + 1) * 3 + 1] = get("f_rest_" + std::to_string(ci + nrc));
      sh[(ci + 1) * 3 + 2] = get("f_rest_" + std::to_string(ci + 2 * nrc));
    }
  };

  if (format == Format::BinaryLE) {
    const size_t total = stride * static_cast<size_t>(vertex_count);
    std::vector<char> data(total);
    if (!file.read(data.data(), static_cast<std::streamsize>(total))) {
      out.error = "Unexpected end of file in vertex block"; return out;
    }
    for (int i = 0; i < vertex_count; ++i) {
      const char * vp = data.data() + static_cast<size_t>(i) * stride;
      auto get = [&](const std::string & n, float d = 0.0f) {
        auto it = props.find(n);
        return (it == props.end()) ? d
          : readBytesAsFloat(vp + it->second.byte_offset, it->second.type);
      };
      store_vertex(i, get);
    }

    // ── Faces (binary) ────────────────────────────────────────────────────
    const size_t cbytes = plyTypeBytes(face_count_type);
    const size_t ibytes = plyTypeBytes(face_index_type);
    out.indices.reserve(static_cast<size_t>(face_count) * 3);
    std::vector<char> buf;
    for (int f = 0; f < face_count; ++f) {
      char cbuf[8];
      if (!file.read(cbuf, static_cast<std::streamsize>(cbytes))) {
        out.error = "Unexpected end of file in face block"; return out;
      }
      const uint32_t n = readBytesAsUInt(cbuf, face_count_type);
      buf.resize(static_cast<size_t>(n) * ibytes);
      if (n && !file.read(buf.data(), static_cast<std::streamsize>(buf.size()))) {
        out.error = "Unexpected end of file in face indices"; return out;
      }
      // Fan-triangulate (n == 3 is the common case → one triangle).
      for (uint32_t k = 1; k + 1 < n; ++k) {
        out.indices.push_back(readBytesAsUInt(buf.data() + 0 * ibytes,       face_index_type));
        out.indices.push_back(readBytesAsUInt(buf.data() + k * ibytes,       face_index_type));
        out.indices.push_back(readBytesAsUInt(buf.data() + (k + 1) * ibytes, face_index_type));
      }
    }
  } else {  // ASCII
    for (int i = 0; i < vertex_count; ++i) {
      if (!std::getline(file, line)) {
        out.error = "Unexpected end of ASCII vertex data"; return out;
      }
      if (!line.empty() && line.back() == '\r') line.pop_back();
      std::istringstream vss(line);
      std::vector<float> vals; float v;
      while (vss >> v) vals.push_back(v);
      auto get = [&](const std::string & n, float d = 0.0f) {
        auto it = props.find(n);
        return (it == props.end() || it->second.value_index >= vals.size())
          ? d : vals[it->second.value_index];
      };
      store_vertex(i, get);
    }
    out.indices.reserve(static_cast<size_t>(face_count) * 3);
    for (int f = 0; f < face_count; ++f) {
      if (!std::getline(file, line)) {
        out.error = "Unexpected end of ASCII face data"; return out;
      }
      std::istringstream fss(line);
      uint32_t n = 0; fss >> n;
      std::vector<uint32_t> idx(n);
      for (uint32_t k = 0; k < n; ++k) fss >> idx[k];
      for (uint32_t k = 1; k + 1 < n; ++k) {
        out.indices.push_back(idx[0]);
        out.indices.push_back(idx[k]);
        out.indices.push_back(idx[k + 1]);
      }
    }
  }

  return out;
}

}  // namespace gsplat_rviz_plugin
