# Mesh-Splat Support in `gsplat_rviz_plugin` — Complete Change Log

This document describes **every** change made to add **mesh-splat** rendering to the
RViz Gaussian-splat plugin (`gsplat_rviz_plugin`). It is intended to be exhaustive:
it covers the motivation, the data format, the rendering pipeline, the spherical-harmonics
math, every new file (with its full purpose), every edited file (with the exact before/after
of each hunk), the runtime crash that was hit and its fix, build/usage instructions, edge
cases, performance characteristics, and known limitations.

---

## 1. Background and Goal

### 1.1 What already existed
The plugin renders **3D Gaussian Splats (3DGS)**. The pipeline is:

- `GsplatDisplay` (an `rviz_common::Display`) exposes properties in the RViz GUI:
  a **Source** selector (`PLY File` / `Topic`), a file picker, a ROS topic field, plus an
  **Advanced** group (SH degree, alpha threshold, sort backend, clip box, WBOIT sub-group,
  scene capture).
- A **source** (`PlyFileSource` for files, `RosTopicSource` for topics) produces a
  `LoadResult` containing a `std::vector<SplatGPU>` and an `sh_degree`.
- `SplatCloud` (an Ogre `MovableObject` + `Renderable` + `RenderObjectListener`) draws all
  Gaussians in **one instanced draw call**: a quad VBO (`±2` corners), a per-instance index
  VBO (sorted back-to-front), an index buffer (two triangles), and a **texture-buffer object
  (TBO)** holding packed per-splat data. EWA splatting projects each 3D Gaussian's covariance
  to a 2D screen-space ellipse; SH color is evaluated in the vertex shader.
- WBOIT (Weighted Blended Order-Independent Transparency) is an alternate transparency mode
  implemented via an Ogre **compositor** plus extra material techniques/passes.

### 1.2 What a "mesh splat" is
A **mesh splat** (produced by *Mesh Splatting*, e.g. `custom_sh_mesh.ply`) is an ordinary
**triangle mesh** whose **vertices carry spherical-harmonics (SH) color**, exactly like the
SH stored on 3DGS Gaussians. The reference viewer is `custom_viewer.py`, which uses a
`diff_triangle_rasterization` CUDA rasterizer and `torch`.

Key differences from Gaussian splats:
- **It is a surface, not a point cloud.** Geometry is triangles (`face` element with
  `vertex_indices`), not oriented ellipsoids.
- **It is opaque.** The reference viewer forces `vertex_weight = 1` ("perfectly opaque meshes
  as designed by Mesh Splatting"). There is **no covariance, no opacity blending, no depth
  sort, and no WBOIT**.
- **Color is still view-dependent via SH.** Each vertex has `f_dc_*` (DC term) and `f_rest_*`
  (higher-order coefficients); the rendered color depends on the view direction.

Because it is an opaque mesh, the rendering path is **simpler** than the Gaussian path: a
standard indexed triangle draw with a per-vertex SH evaluation. No torch/CUDA is required —
the SH evaluation is a handful of multiply-adds done in a GLSL vertex shader.

### 1.3 The example PLY header (`custom_sh_mesh.ply`)
```
format binary_little_endian 1.0
element vertex 2353298
property float x
property float y
property float z
property float nx
property float ny
property float nz
property float f_dc_0
property float f_dc_1
property float f_dc_2
property float f_rest_0
...
property float f_rest_44      # 45 rest coeffs = 15 per channel × 3 channels → SH degree 3
property float vertex_weight
element face 1114103
property list uchar int vertex_indices
end_header
```
- **2,353,298 vertices**, **1,114,103 faces**.
- Vertex properties: `x y z`, `nx ny nz` (normals — unused for color), `f_dc_0..2` (DC SH, 3
  values), `f_rest_0..44` (45 higher-order SH values), `vertex_weight` (unused; opacity is
  forced to 1).
- `45 / 3 = 15` rest coefficients per channel → total `1 + 15 = 16` coefficients per channel
  → **SH degree 3** (`(3+1)² = 16`).
- Faces: `property list uchar int vertex_indices` — each face is a `uchar` count followed by
  `int` indices (triangles, so count = 3).

### 1.4 Requirement
> Allow loading meshes (just like WBOIT was added) in the splat plugin. Load and visualize
> mesh splats just like Gaussian splats in RViz. Keep changes minimal. No torch available.
> When "mesh splat" is selected, the PLY file path must directly take mesh-splat output PLYs.

---

## 2. Design Decisions

1. **New Source mode, not a new Display.** Added a third option, **"Mesh PLY File"**, to the
   existing **Source** enum (`PLY File` / `Topic` / `Mesh PLY File`). The existing file picker
   is **reused**: when the mode is "Mesh PLY File", the same "Splat File" field loads
   mesh-splat PLYs directly. This satisfies "the ply file path must directly take in mesh
   splat output ply files" with minimal UI surface.

2. **Dedicated, separate render object (`MeshSplatCloud`).** The Gaussian `SplatCloud` is
   heavily specialized for instanced quad/EWA splatting. Shoehorning triangle-mesh rendering
   into it would be invasive. Instead, a small parallel class draws the mesh. Both objects are
   attached to the display's scene node; only one is populated at a time (the other is
   cleared, so it renders nothing).

3. **Per-vertex SH in a TBO indexed by `gl_VertexID`.** 16 coefficients × 3 channels = 48
   floats per vertex is far too many vertex attributes. Instead the SH lives in a `GL_RGBA16F`
   TBO (one texel per coefficient), and the vertex shader fetches its vertex's coefficients
   using `gl_VertexID` (which, under indexed drawing, equals the vertex index). This mirrors
   the existing Gaussian SH-TBO pattern exactly (half-float packing, manual `glBindTexture` in
   `notifyRenderSingleObject`).

4. **SH evaluated per-vertex (Gouraud), in object space.** The direction is
   `normalize(position − cameraPositionObjectSpace)`, matching the 3DGS/Mesh-Splatting
   convention `dir = normalize(pos − campos)`. Object space is used (via Ogre's
   `camera_position_object_space` auto-param) so the SH is evaluated in the frame the
   coefficients were trained in, independent of the RViz scene-node transform. Per-vertex
   (rather than per-fragment) keeps it cheap and is accurate for the dense tessellation of
   mesh-splat output; it also avoids passing 16 coefficients through fragment varyings.

5. **Opaque pass, no sort/WBOIT.** The mesh material is a single opaque technique with depth
   test + write, no blending, `cull_hardware none` (the reference rasterizer renders meshes
   double-sided). It renders in `RENDER_QUEUE_MAIN`. Sorting/WBOIT/clip controls are left
   untouched and are simply irrelevant in mesh mode.

6. **Reuse the SH-degree slider.** The Advanced **SH Degree** property drives both clouds. On
   mesh load, its max is set to the mesh's degree and the value defaulted (SH1 if available).

---

## 3. New Files

### 3.1 `include/gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.hpp`
Declares the CPU-side mesh container and the loader entry point.

```cpp
#ifndef GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__MESH_PLY_LOADER_HPP_
#define GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__MESH_PLY_LOADER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "gsplat_rviz_plugin/visibility_control.hpp"

namespace gsplat_rviz_plugin
{

// CPU-side representation of a "mesh splat": an ordinary triangle mesh whose
// vertices carry spherical-harmonics colour (DC f_dc_* + higher-order f_rest_*),
// exactly the format produced by Mesh Splatting (see custom_sh_mesh.ply).
//
// Unlike Gaussian splats these are opaque surfaces — no covariance, opacity,
// sorting or WBOIT.  Colour is view-dependent via the SH coefficients.
struct MeshSplatData
{
  // Vertex positions, 3 floats per vertex (object space).
  std::vector<float>    positions;
  // Triangle indices into `positions`, 3 per triangle.
  std::vector<uint32_t> indices;
  // SH coefficients, coefficient-major RGB per vertex:
  //   [v0_c0.rgb, v0_c1.rgb, …, v0_c(stride-1).rgb, v1_c0.rgb, …]
  // where stride = (sh_degree+1)² and coeff 0 is the DC term.
  std::vector<float>    sh;
  int                   sh_degree    = 0;   // 0/1/2/3 from f_rest_* count
  int                   sh_stride    = 1;    // (sh_degree+1)² coeffs per vertex

  std::string           error;              // non-empty on failure

  bool ok() const { return error.empty(); }
  uint32_t vertexCount()   const { return static_cast<uint32_t>(positions.size() / 3); }
  uint32_t triangleCount() const { return static_cast<uint32_t>(indices.size() / 3); }
};

// Load a mesh-splat PLY (binary_little_endian or ascii) from `path`.
// On failure the returned data has a non-empty `error`.
GSPLAT_RVIZ_PLUGIN_PUBLIC
MeshSplatData loadMeshPly(const std::string & path);

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__SPLAT_LOADERS__MESH_PLY_LOADER_HPP_
```

**Field-by-field semantics**
- `positions` — flat `[x0,y0,z0, x1,y1,z1, …]`, object space, length `3·V`.
- `indices` — flat triangle list `[i0,i1,i2, …]`, 32-bit (vertex counts exceed 65535), length
  `3·T`.
- `sh` — **coefficient-major RGB per vertex**, length `V · stride · 3`. For vertex `v`,
  coefficient `c`: `sh[(v·stride + c)·3 + {0,1,2}]`. Coefficient `0` is the DC term.
- `sh_degree` — `0/1/2/3`, detected from the number of `f_rest_*` properties.
- `sh_stride` — `(sh_degree+1)²` (coefficients per vertex, **including** DC): `1/4/9/16`.
- `error` — empty on success; a human-readable message on failure.
- Helpers `ok()`, `vertexCount()`, `triangleCount()` derive counts from the vectors.

### 3.2 `src/gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.cpp`
Implements `loadMeshPly`. It is a self-contained PLY parser (it intentionally duplicates a few
tiny byte-reading helpers from `ply_loader.cpp`'s anonymous namespace rather than coupling to
it).

**Anonymous-namespace helpers**
- `struct PropEntry { std::string type; size_t byte_offset; size_t value_index; };` — describes
  one vertex scalar property (its PLY type string, its byte offset within a binary vertex
  record, and its column index within an ASCII vertex line).
- `plyTypeBytes(type)` — size in bytes of a PLY scalar type (`float/int/uint` = 4,
  `double/int64/uint64` = 8, `short/ushort` = 2, `char/uchar` = 1; default 4).
- `readBytesAsFloat(ptr, type)` — reads a value of the given PLY type from `ptr` and returns it
  as `float` (used for positions and SH).
- `readBytesAsUInt(ptr, type)` — reads an integer of the given PLY type and returns it as
  `uint32_t` (used for face counts and vertex indices).

**`loadMeshPly(path)` algorithm**
1. Open file binary. Read first line; must start with `ply`.
2. **Header parse loop** until `end_header`:
   - `format` → `binary_little_endian` or `ascii` (else error). Unknown → error.
   - `element vertex N` → enter Vertex element, record `vertex_count = N`.
   - `element face N` → enter Face element, record `face_count = N`.
   - `element <other>` → enter "None" element (ignored).
   - `property list <count_type> <item_type> <name>` → if inside the Face element, record
     `face_count_type` and `face_index_type` (defaults `uchar` and `int`).
   - `property <type> <name>` → if inside the Vertex element, append `(name, type)` to
     `vertex_props` (in header order).
3. **Validation**: format known; `vertex_count > 0`; `face_count > 0` (a PLY with no faces is
   rejected as "not a mesh splat").
4. **Build the vertex property map**: iterate `vertex_props` accumulating byte offsets →
   `props[name] = {type, byte_offset, value_index}`; `stride` = total bytes per vertex record.
   Require `x`, `y`, `z` present.
5. **Detect SH degree**: count properties whose name starts with `f_rest_` → `num_rest`;
   `nrc = num_rest/3` (rest per channel); `tpc = 1 + nrc` (total per channel incl. DC);
   `sh_degree = 16≤? 3 : 9≤? 2 : 4≤? 1 : 0` (using `tpc`); `sh_stride = (sh_degree+1)²`.
6. **Allocate outputs**: `positions` (`3·V`), `sh` zero-filled (`V·stride·3`).
7. **`store_vertex(i, get)`** (generic lambda; `get(name, default)` fetches a named float):
   - Stores `x,y,z` into `positions`.
   - `sh[0..2] = f_dc_0..2` (DC, coefficient 0).
   - For each rest coefficient `ci` in `[0, nrc)` and while `ci+1 < sh_stride`:
     `sh[(ci+1)·3 + 0/1/2] = f_rest_{ci} / f_rest_{ci+nrc} / f_rest_{ci+2·nrc}`. This
     **reorders PLY channel-major `f_rest` into coefficient-major RGB**, identical to the
     Gaussian loader's `fillSH`.
8. **Binary path** (`Format::BinaryLE`):
   - Read the whole vertex block (`stride·V` bytes) into a `std::vector<char>`. For each vertex
     `i`, build a `get` lambda that reads from `props` at the record pointer and call
     `store_vertex`.
   - **Faces**: `cbytes = plyTypeBytes(face_count_type)`, `ibytes = plyTypeBytes(face_index_type)`.
     For each of `face_count` faces: read the count (`cbytes`), read `n·ibytes` index bytes into
     a reusable buffer, then **fan-triangulate**: for `k` in `[1, n-1)` push
     `(idx[0], idx[k], idx[k+1])`. (For the common `n == 3` this yields exactly one triangle;
     `n > 3` polygons are triangulated; degenerate `n < 3` faces produce no triangles.)
9. **ASCII path** (`Format::ASCII`):
   - For each vertex line, tokenize floats; `get(name)` indexes by the property's `value_index`.
     Call `store_vertex`.
   - For each face line, read `n` then `n` indices; fan-triangulate as above.
10. Return `out` (with `error` empty on success).

**Coordinate frame**: positions and SH are kept exactly as stored (object space). No axis flip
is applied — consistent with the existing Gaussian loader, which also does not flip.

### 3.3 `include/gsplat_rviz_plugin/mesh_splat_cloud.hpp`
Declares `MeshSplatCloud`, the Ogre render object for the mesh.

```cpp
#ifndef GSPLAT_RVIZ_PLUGIN__MESH_SPLAT_CLOUD_HPP_
#define GSPLAT_RVIZ_PLUGIN__MESH_SPLAT_CLOUD_HPP_

#include <cstdint>

#include <OgreAxisAlignedBox.h>
#include <OgreHardwareVertexBuffer.h>
#include <OgreMaterial.h>
#include <OgreMovableObject.h>
#include <OgreRenderable.h>
#include <OgreRenderObjectListener.h>
#include <OgreRenderOperation.h>

#include "gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.hpp"
#include "gsplat_rviz_plugin/visibility_control.hpp"

namespace Ogre
{
class RenderQueue;
class SceneManager;
class SceneNode;
}

namespace gsplat_rviz_plugin
{

class GSPLAT_RVIZ_PLUGIN_PUBLIC MeshSplatCloud
  : public Ogre::MovableObject,
    public Ogre::Renderable,
    public Ogre::RenderObjectListener
{
public:
  explicit MeshSplatCloud(Ogre::SceneNode * parent_node);
  ~MeshSplatCloud() override;

  void setMesh(MeshSplatData data);
  void clear();

  void setShDegree(int d);
  int  getMaxShDegree()    const { return max_sh_degree_; }
  uint32_t getVertexCount() const { return vertex_count_; }
  uint32_t getTriangleCount() const { return triangle_count_; }

  // Ogre::MovableObject
  const Ogre::String & getMovableType() const override;
  const Ogre::AxisAlignedBox & getBoundingBox() const override { return bounds_; }
  Ogre::Real getBoundingRadius() const override;
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;
  void visitRenderables(Ogre::Renderable::Visitor * visitor, bool debug = false) override;

  // Ogre::Renderable
  const Ogre::MaterialPtr & getMaterial() const override { return material_; }
  void getRenderOperation(Ogre::RenderOperation & op) override;
  void getWorldTransforms(Ogre::Matrix4 * xform) const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override;
  const Ogre::LightList & getLights() const override;

  // Ogre::RenderObjectListener
  void notifyRenderSingleObject(
    Ogre::Renderable * rend,
    const Ogre::Pass * pass,
    const Ogre::AutoParamDataSource * source,
    const Ogre::LightList * pLightList,
    bool suppressRenderStateChanges) override;

private:
  void destroyTBO();
  void uploadTBO();   // must run with the GL context current

  Ogre::MaterialPtr     material_;
  Ogre::RenderOperation render_op_;
  Ogre::AxisAlignedBox  bounds_;
  Ogre::SceneManager *  scene_manager_ = nullptr;

  std::vector<float> pending_sh_;
  bool     upload_pending_ = false;
  uint32_t vertex_count_   = 0;
  uint32_t triangle_count_ = 0;
  int      max_sh_degree_    = 0;
  int      active_sh_degree_ = 0;
  int      sh_stride_        = 1;

  uint32_t sh_tbo_buf_ = 0;
  uint32_t sh_tbo_tex_ = 0;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__MESH_SPLAT_CLOUD_HPP_
```

**Member semantics**
- `material_` — the `gsplat_rviz_plugin/MeshSplat` material (loaded by name from the
  `rviz_rendering` resource group).
- `render_op_` — the Ogre render operation: triangle list, indexed, source-0 = position VBO.
- `bounds_` — AABB of the mesh (for culling / view-depth).
- `scene_manager_` — owning scene manager (used to (de)register the render-object listener).
- `pending_sh_` — staged coefficient-major RGB SH floats, held until the first render when the
  GL context is current and the TBO is created; freed afterward.
- `upload_pending_` — set by `setMesh`, consumed by the first `notifyRenderSingleObject`.
- `vertex_count_`, `triangle_count_` — geometry sizes.
- `max_sh_degree_` — highest degree available in the loaded mesh.
- `active_sh_degree_` — degree currently sent to the shader (user-controlled, clamped).
- `sh_stride_` — coefficients per vertex `(max_sh_degree_+1)²`; the TBO indexing stride.
- `sh_tbo_buf_`, `sh_tbo_tex_` — raw GL handles for the SH texture-buffer object.

### 3.4 `src/gsplat_rviz_plugin/mesh_splat_cloud.cpp`
Implements `MeshSplatCloud`.

- **`floatToHalf(float)`** — IEEE-754 float32 → binary16 (round-to-nearest-even, saturate to
  inf). Identical algorithm to `splat_cloud.cpp`'s; kept `static` (internal linkage) so there
  is no ODR clash. Used to pack SH into the `RGBA16F` TBO.

- **Constructor** `MeshSplatCloud(Ogre::SceneNode * parent_node) : Ogre::MovableObject("GsplatMeshSplatCloud")`:
  - **Passes an explicit, distinct name** `"GsplatMeshSplatCloud"` to the `MovableObject` base
    (see §5, the crash fix). `SplatCloud` uses the default empty name `""`, and Ogre keys
    attached objects by name **per scene node**; two empty names on the same node throws.
  - `scene_manager_ = parent_node->getCreator()`.
  - Initializes `render_op_`: `OT_TRIANGLE_LIST`, `useIndexes = true`, fresh `VertexData` and
    `IndexData`, and a vertex declaration with one element: source 0, `VET_FLOAT3`,
    `VES_POSITION` (so position binds to GLSL `layout(location = 0)`).
  - Loads `material_` = `"gsplat_rviz_plugin/MeshSplat"` from group `"rviz_rendering"` and
    `load()`s it.
  - `attachObject(this)` to the node; `addRenderObjectListener(this)` to the scene manager.

- **Destructor**: removes the render-object listener; detaches from the parent node; calls
  `destroyTBO()`; deletes `render_op_.vertexData` and `render_op_.indexData`.

- **`destroyTBO()`**: deletes the GL texture and buffer if present; zeroes the handles.

- **`setMesh(MeshSplatData data)`**:
  - `clear()` first (drops any previous mesh/TBO/buffers).
  - Copies counts/degree/stride; `active_sh_degree_ = min(1, sh_degree)` (default SH1 if
    available). If `vertex_count_ == 0` or `triangle_count_ == 0`, `clear()` and return.
  - Computes `bounds_` by merging all vertex positions.
  - Creates the **position VBO** (source 0): `3·sizeof(float)` stride, `vertex_count_` vertices,
    `HBU_STATIC_WRITE_ONLY`; uploads positions; sets `vertexStart/vertexCount`; binds to slot 0.
  - Creates the **32-bit index buffer** (`IT_32BIT`, `indices.size()` indices,
    `HBU_STATIC_WRITE_ONLY`); uploads indices; sets `indexStart = 0`, `indexCount = indices.size()`.
  - Moves `data.sh` into `pending_sh_`; sets `upload_pending_ = true` (TBO is created lazily at
    render time).
  - `node->needUpdate()` so bounds refresh.

- **`clear()`**: zeroes counts/degree/stride; clears `pending_sh_` (and `shrink_to_fit`);
  `destroyTBO()`; zeroes `vertexCount`; `unsetAllBindings()`; zeroes `indexCount` and resets the
  index buffer; nulls `bounds_`; `node->needUpdate()`.

- **`setShDegree(int d)`**: `active_sh_degree_ = clamp(d, 0, max_sh_degree_)`.

- **`uploadTBO()`** (must run with GL current):
  - `destroyTBO()`; if `pending_sh_` empty, return.
  - Packs each SH coefficient (RGB float triple) into one `RGBA16F` texel: R,G,B → halves, A=0.
    The packed vector has `(pending_sh_.size()/3)·4` `uint16_t`s. Frees `pending_sh_` afterward.
  - `glGenBuffers`/`glBindBuffer(GL_TEXTURE_BUFFER)`/`glBufferData(GL_STATIC_DRAW)`.
  - `glGenTextures`/`glBindTexture(GL_TEXTURE_BUFFER)`/`glTexBuffer(GL_RGBA16F, buf)`.

- **MovableObject overrides**:
  - `getMovableType()` → `"MeshSplatCloud"`.
  - `getBoundingRadius()` → half-size length (0 if null bounds).
  - `_updateRenderQueue(queue)` → if `triangle_count_ > 0`, add `this` to `RENDER_QUEUE_MAIN`
    at `mRenderQueuePriority`.
  - `visitRenderables(visitor)` → visit `this` if non-empty.

- **Renderable overrides**:
  - `getRenderOperation(op)` → `op = render_op_; op.numberOfInstances = 1`.
  - `getWorldTransforms(xform)` → the parent node's full transform (identity if detached).
  - `getSquaredViewDepth(cam)` → squared distance from bounds center to camera.
  - `getLights()` → `queryLights()`.

- **`notifyRenderSingleObject(rend, pass, …)`** (the listener hook, fires immediately before a
  renderable is drawn):
  - If `rend != this`, return (the same scene manager also drives `SplatCloud`'s listener).
  - If `upload_pending_`, call `uploadTBO()` and clear the flag (GL context is current here).
  - If the pass has a vertex program, set `sh_degree = active_sh_degree_` and
    `u_sh_stride = sh_stride_` (with `setIgnoreMissingParams(true)`).
  - Bind the SH TBO to texture unit 0 (`glActiveTexture(GL_TEXTURE0)` +
    `glBindTexture(GL_TEXTURE_BUFFER, sh_tbo_tex_)`). The material declares no `texture_unit`,
    so unit 0 is exactly this manual binding (same pattern as `SplatCloud`).

### 3.5 `ogre_media/materials/glsl120/mesh_splat.vert`
GLSL 330 vertex shader. Evaluates SH per vertex and outputs the clip-space position.

- Inputs/uniforms:
  - `layout(location = 0) in vec3 position;` — object-space vertex position.
  - `uniform samplerBuffer u_mesh_sh;` — the SH TBO (RGBA16F, one texel per coefficient).
  - `uniform mat4 worldViewProj;` — auto `worldviewproj_matrix`.
  - `uniform vec3 camPosObj;` — auto `camera_position_object_space`.
  - `uniform int sh_degree;` — active degree (0..3).
  - `uniform int u_sh_stride;` — coefficients per vertex `(max_degree+1)²`.
  - `out vec3 vColor;`
- SH constants `SH_C0..SH_C3_6` — identical to `splat.vert`.
- `fetchSH(base, k)` = `texelFetch(u_mesh_sh, base + k).rgb`.
- `main()`:
  - `base = gl_VertexID * u_sh_stride` (vertex's coefficient block start).
  - DC: `rgb = SH_C0 * fetchSH(base, 0) + vec3(0.5)`.
  - If `sh_degree > 0`: `d = normalize(position − camPosObj)`, then add order-1 terms
    (`fetchSH(base, 1..3)`), order-2 (`4..8`) if degree ≥ 2, order-3 (`9..15`) if degree ≥ 3.
    The index offsets are the Gaussian shader's `+1` (the mesh TBO includes DC at index 0,
    whereas the Gaussian SH TBO excludes DC).
  - `vColor = clamp(rgb, 0.0, 1.0)`.
  - `gl_Position = worldViewProj * vec4(position, 1.0)`.

**Math note**: 3DGS computes `color = C0·sh0 + (order terms) + 0.5`, then `max(color, 0)`. Adding
`0.5` to the DC term and summing the orders is algebraically identical (addition is
associative). Clamping the max to `1.0` only saturates values that would clamp in the
framebuffer anyway.

### 3.6 `ogre_media/materials/glsl120/mesh_splat.frag`
GLSL 330 fragment shader — trivial, since color was computed per vertex:
```glsl
in  vec3 vColor;
out vec4 frag_color;
void main() { frag_color = vec4(vColor, 1.0); }  // opaque
```

### 3.7 `ogre_media/materials/scripts/mesh_splat.program`
Registers the two GLSL programs and their default params / auto-bindings:
```
vertex_program gsplat_rviz_plugin/MeshSplat.vert glsl
{
    source mesh_splat.vert
    default_params
    {
        param_named_auto worldViewProj  worldviewproj_matrix
        param_named_auto camPosObj       camera_position_object_space
        param_named      u_mesh_sh       int 0
        param_named      sh_degree       int 0
        param_named      u_sh_stride     int 1
    }
}

fragment_program gsplat_rviz_plugin/MeshSplat.frag glsl
{
    source mesh_splat.frag
}
```

### 3.8 `ogre_media/materials/scripts/mesh_splat.material`
A single opaque technique:
```
material gsplat_rviz_plugin/MeshSplat
{
    technique
    {
        pass
        {
            lighting off
            depth_write on
            depth_check on
            cull_hardware none

            vertex_program_ref gsplat_rviz_plugin/MeshSplat.vert {}
            fragment_program_ref gsplat_rviz_plugin/MeshSplat.frag {}
        }
    }
}
```
- `lighting off` — color comes from SH, not scene lights.
- `depth_write on` / `depth_check on` — opaque surface with normal Z-buffering.
- `cull_hardware none` — render double-sided (mesh-splat triangles may have arbitrary winding).
- No `scene_blend` (opaque). No `texture_unit` (the SH TBO is bound manually on unit 0).

These scripts live under the two directories already passed to
`register_rviz_ogre_media_exports(...)`, so Ogre auto-discovers and parses them; no extra
registration is needed.

---

## 4. Edited Files

### 4.1 `CMakeLists.txt`
Added the two new translation units to the plugin's source list.

**Before:**
```cmake
  src/gsplat_rviz_plugin/displays/gsplat_display.cpp
  src/gsplat_rviz_plugin/splat_cloud.cpp
  src/gsplat_rviz_plugin/perf_monitor.cpp
```
**After:**
```cmake
  src/gsplat_rviz_plugin/displays/gsplat_display.cpp
  src/gsplat_rviz_plugin/splat_cloud.cpp
  src/gsplat_rviz_plugin/mesh_splat_cloud.cpp
  src/gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.cpp
  src/gsplat_rviz_plugin/perf_monitor.cpp
```
No other CMake changes were needed: the headers are picked up via the existing include dir, and
the Ogre media directories were already registered.

### 4.2 `include/gsplat_rviz_plugin/displays/gsplat_display.hpp`

**(a) Forward-declare the new render object** (next to the existing `class SplatCloud;`):
```cpp
namespace gsplat_rviz_plugin
{
class SplatCloud;
class MeshSplatCloud;   // ← added
```

**(b) Extend the source-mode enum and declare a loader helper:**

Before:
```cpp
  enum class SourceKind { None, File, Topic };
  enum class SourceMode { File = 0, Topic = 1 };
```
After:
```cpp
  enum class SourceKind { None, File, Topic };
  // File = Gaussian-splat PLY, Topic = streamed gaussians, MeshFile = mesh-splat PLY.
  enum class SourceMode { File = 0, Topic = 1, MeshFile = 2 };

  // Load a mesh-splat PLY from splat_path_property_ into mesh_cloud_.
  void loadMeshFile();
```

**(c) Add the mesh render-object member** (alongside `splat_cloud_`):

Before:
```cpp
  std::unique_ptr<SplatCloud>   splat_cloud_;
  std::unique_ptr<ISplatSource> source_;
```
After:
```cpp
  std::unique_ptr<SplatCloud>     splat_cloud_;
  std::unique_ptr<MeshSplatCloud> mesh_cloud_;
  std::unique_ptr<ISplatSource> source_;
```

### 4.3 `src/gsplat_rviz_plugin/displays/gsplat_display.cpp`

**(a) Includes** — added the mesh loader and render object:
```cpp
#include "gsplat_rviz_plugin/splat_loaders/ply_file_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/ros_topic_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.hpp"   // ← added
#include "gsplat_rviz_plugin/mesh_splat_cloud.hpp"                // ← added
#include "gsplat_rviz_plugin/splat_cloud.hpp"
```

**(b) Source dropdown** — added the third option and broadened the file-picker description:

Before:
```cpp
  source_mode_property_->addOption("PLY File", static_cast<int>(SourceMode::File));
  source_mode_property_->addOption("Topic",    static_cast<int>(SourceMode::Topic));

  splat_path_property_ = new rviz_common::properties::FilePickerProperty(
    "Splat File", "",
    "Path to a 3DGS-format PLY file to visualize.",
    this, SLOT(onSplatPathChanged()),
    this);
```
After:
```cpp
  source_mode_property_->addOption("PLY File",      static_cast<int>(SourceMode::File));
  source_mode_property_->addOption("Topic",         static_cast<int>(SourceMode::Topic));
  source_mode_property_->addOption("Mesh PLY File", static_cast<int>(SourceMode::MeshFile));

  splat_path_property_ = new rviz_common::properties::FilePickerProperty(
    "Splat File", "",
    "Path to a PLY file to visualize. In \"PLY File\" mode this is a "
    "3DGS Gaussian-splat PLY; in \"Mesh PLY File\" mode it is a mesh-splat "
    "PLY (triangle mesh with per-vertex spherical harmonics).",
    this, SLOT(onSplatPathChanged()),
    this);
```

**(c) `onInitialize`** — create the mesh cloud alongside the Gaussian cloud:

Before:
```cpp
  splat_cloud_ = std::make_unique<SplatCloud>(scene_node_);

  rebuildSorter();
```
After:
```cpp
  splat_cloud_ = std::make_unique<SplatCloud>(scene_node_);
  mesh_cloud_  = std::make_unique<MeshSplatCloud>(scene_node_);

  rebuildSorter();
```

**(d) `reset`** — also clear the mesh cloud:

Before:
```cpp
  if (splat_cloud_) splat_cloud_->clear();
  sh_degree_property_->setMax(0);
```
After:
```cpp
  if (splat_cloud_) splat_cloud_->clear();
  if (mesh_cloud_)  mesh_cloud_->clear();
  sh_degree_property_->setMax(0);
```

**(e) `onSplatPathChanged`** — accept both file modes, clear both clouds on empty path, and
dispatch to the mesh loader in mesh mode:

Before:
```cpp
void GsplatDisplay::onSplatPathChanged()
{
  if (currentMode() != SourceMode::File) return;
  if (!splat_cloud_) return;

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    splat_cloud_->clear();
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File", "No file selected.");
    return;
  }

  installSource(
    std::make_unique<PlyFileSource>(path.toStdString()),
    SourceKind::File);
}
```
After:
```cpp
void GsplatDisplay::onSplatPathChanged()
{
  const auto mode = currentMode();
  if (mode != SourceMode::File && mode != SourceMode::MeshFile) return;
  if (!splat_cloud_ || !mesh_cloud_) return;

  const QString path = splat_path_property_->getString();
  if (path.isEmpty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    splat_cloud_->clear();
    mesh_cloud_->clear();
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Splat File", "No file selected.");
    return;
  }

  if (mode == SourceMode::MeshFile) {
    loadMeshFile();
    return;
  }

  installSource(
    std::make_unique<PlyFileSource>(path.toStdString()),
    SourceKind::File);
}
```

**(f) New `loadMeshFile()`** (inserted immediately after `onSplatPathChanged`):
```cpp
void GsplatDisplay::loadMeshFile()
{
  if (!mesh_cloud_) return;

  // No ROS-topic source involved; bump the generation so any in-flight
  // gaussian load callback is dropped, and make sure the gaussian cloud
  // is empty while a mesh is shown.
  ++source_gen_;
  source_.reset();
  source_kind_ = SourceKind::None;
  if (splat_cloud_) splat_cloud_->clear();

  const std::string path = splat_path_property_->getString().toStdString();
  MeshSplatData data = loadMeshPly(path);
  if (!data.ok()) {
    mesh_cloud_->clear();
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Splat File", QString::fromStdString(data.error));
    return;
  }

  const uint32_t verts = data.vertexCount();
  const uint32_t tris  = data.triangleCount();
  const int sh_degree  = data.sh_degree;

  mesh_cloud_->setMesh(std::move(data));

  sh_degree_property_->setMax(sh_degree);
  sh_degree_property_->setValue(std::min(1, sh_degree));
  mesh_cloud_->setShDegree(sh_degree_property_->getInt());

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Splat File",
    QString("Loaded mesh splat: %1 vertices, %2 triangles (SH degree %3)")
      .arg(verts).arg(tris).arg(sh_degree));

  if (context_) context_->queueRender();
}
```
Behavior:
- Drops any in-flight Gaussian source callback (`++source_gen_`, reset `source_`).
- Clears the Gaussian cloud so only the mesh shows.
- Loads the mesh synchronously (like `PlyFileSource`). On error, clears the mesh and sets an
  Error status under the "Splat File" key.
- On success, hands the data to `mesh_cloud_->setMesh`, sets the SH-degree slider max and a
  sensible default value, pushes the degree to the mesh, and reports counts + degree in the
  status line. Requests a render.

**(g) `onShDegreeChanged`** — drive both clouds and request a render:

Before:
```cpp
void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
}
```
After:
```cpp
void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
  if (mesh_cloud_) {
    mesh_cloud_->setShDegree(sh_degree_property_->getInt());
  }
  if (context_) context_->queueRender();
}
```

**(h) `onSourceModeChanged`** — clear both clouds; show the file picker for both file modes;
fix the status-key cleanup for three modes:

Before:
```cpp
  if (splat_cloud_) splat_cloud_->clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);

  const auto mode = currentMode();
  splat_path_property_->setHidden(mode != SourceMode::File);
  topic_property_->setHidden(mode != SourceMode::Topic);

  deleteStatus(mode == SourceMode::File ? "Topic" : "Splat File");

  if (mode == SourceMode::File) {
    onSplatPathChanged();
  } else {
    onTopicChanged();
  }
```
After:
```cpp
  if (splat_cloud_) splat_cloud_->clear();
  if (mesh_cloud_)  mesh_cloud_->clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);

  const auto mode = currentMode();
  const bool file_mode = (mode == SourceMode::File || mode == SourceMode::MeshFile);
  splat_path_property_->setHidden(!file_mode);
  topic_property_->setHidden(mode != SourceMode::Topic);

  deleteStatus(mode == SourceMode::Topic ? "Splat File" : "Topic");

  if (file_mode) {
    onSplatPathChanged();
  } else {
    onTopicChanged();
  }
```
Notes:
- `file_mode` is true for both `File` and `MeshFile`; the file picker shows for both. The topic
  field shows only for `Topic`.
- Status-key cleanup: in `Topic` mode delete the "Splat File" status; otherwise (either file
  mode) delete the "Topic" status. Both file modes use the "Splat File" status key.

---

## 5. The Runtime Crash and Its Fix

### 5.1 Symptom
On adding the display in RViz:
```
terminate called after throwing an instance of 'Ogre::RuntimeAssertionException'
  what():  RuntimeAssertionException: Object was not attached because an object of the same
  name was already attached to this node. in attachObject at .../OgreSceneNode.cpp (line 115)
Aborted (core dumped)
```

### 5.2 Root cause
`onInitialize` attaches **two** `MovableObject`s to the same scene node: `SplatCloud` and
`MeshSplatCloud`. `SplatCloud` constructs its `MovableObject` base with the **default empty
name** `""`. The initial `MeshSplatCloud` also default-constructed the base → also `""`. Ogre's
`SceneNode::attachObject` indexes attached objects by name; two objects with the **same** name
(`""`) on the **same** node trip the assertion.

### 5.3 Fix
Give `MeshSplatCloud` a distinct, non-empty name by passing it to the `MovableObject` base
constructor:
```cpp
MeshSplatCloud::MeshSplatCloud(Ogre::SceneNode * parent_node)
  // Distinct from SplatCloud's empty name — Ogre keys attached objects by
  // name per scene node, and both clouds share the display's scene node.
  : Ogre::MovableObject("GsplatMeshSplatCloud")
{
  ...
}
```

### 5.4 Why a fixed name is safe
The name only has to be unique **within a single scene node**. Each `GsplatDisplay` owns its
own `scene_node_`, so even with multiple Gaussian Splat displays, each node holds exactly one
`SplatCloud` (`""`) and one `MeshSplatCloud` (`"GsplatMeshSplatCloud"`) — no collision. We
attach manually (`new` + `attachObject`) rather than via
`SceneManager::createMovableObject`, so there is **no global** name registry to satisfy; the
per-node key is all that matters.

---

## 6. Data & Control Flow Summary

**Load (mesh mode):**
```
User picks file (Source = "Mesh PLY File")
  → GsplatDisplay::onSplatPathChanged()
    → GsplatDisplay::loadMeshFile()
      → loadMeshPly(path)  ──►  MeshSplatData {positions, indices, sh, sh_degree, sh_stride}
      → splat_cloud_->clear()          (hide gaussians)
      → mesh_cloud_->setMesh(data)     (create VBO + 32-bit IBO; stage SH; mark upload_pending_)
      → sh_degree slider max/value set; mesh_cloud_->setShDegree(...)
      → status "Loaded mesh splat: V vertices, T triangles (SH degree D)"
```

**Render (per frame):**
```
Ogre scene render
  → MeshSplatCloud::_updateRenderQueue()       (enqueue in RENDER_QUEUE_MAIN if non-empty)
  → MeshSplatCloud::getRenderOperation()        (triangle list, 1 instance)
  → MeshSplatCloud::notifyRenderSingleObject()  (first time: uploadTBO(); set sh_degree &
                                                 u_sh_stride; bind SH TBO to unit 0)
  → GPU: mesh_splat.vert (per-vertex SH eval, MVP) → mesh_splat.frag (opaque color)
```

**SH indexing in the shader:** for a vertex drawn via the index buffer, `gl_VertexID` equals
the vertex's index, so `base = gl_VertexID * u_sh_stride` selects that vertex's coefficient
block in the TBO; `fetchSH(base, c)` reads coefficient `c` (`c = 0` is DC).

---

## 7. Build & Usage

### 7.1 Build
```bash
colcon build --packages-select gsplat_rviz_plugin
# then re-source the workspace install before launching RViz
source install/setup.bash
```

### 7.2 Use
1. Launch `rviz2`.
2. Add the Gaussian Splat display (the `gsplat_rviz_plugin` display).
3. Set **Source → "Mesh PLY File"**.
4. In the **Splat File** picker, choose a mesh-splat `.ply` (e.g. `custom_sh_mesh.ply`).
5. The mesh appears, opaque, with view-dependent SH color.
6. Use **Advanced → SH Degree** to switch between DC-only (0) and full degree (up to 3). Higher
   degree = more view-dependent detail.

To switch back to Gaussian splats, set **Source → "PLY File"** (or **"Topic"**); the mesh is
cleared and the Gaussian path resumes.

---

## 8. Edge Cases, Performance, Limitations

### 8.1 Memory and performance (for the 2.35M-vertex / 1.11M-face example)
- Vertex record stride ≈ 220 B (55 float properties) → vertex block ≈ 517 MB read into RAM
  during load; faces ≈ 14 MB; total ≈ the 532 MB file.
- `MeshSplatData.sh` (float, coefficient-major RGB) ≈ `2.35M × 16 × 3 × 4 B` ≈ 451 MB
  (transient, freed after the TBO is built).
- GPU SH TBO (`RGBA16F`, 16 texels/vertex) ≈ `2.35M × 16 × 8 B` ≈ 300 MB; positions ≈ 28 MB;
  indices ≈ 13 MB.
- Loading is **synchronous** (same as `PlyFileSource`), so expect a brief pause on selection.

### 8.2 Correctness details
- **SH convention** matches the reference rasterizer: `dir = normalize(pos − campos)`, evaluated
  in **object space** (so it is independent of the RViz scene-node transform). The SH constants
  and term ordering are copied verbatim from the Gaussian `splat.vert`.
- **DC + 0.5**: `color = SH_C0·f_dc + 0.5 + Σ(order terms)`, then clamped to `[0,1]`. Matches
  3DGS up to the harmless upper clamp.
- **Double-sided**: `cull_hardware none`, matching mesh-splat's design (opaque, no back-face
  culling).
- **Non-triangle faces**: fan-triangulated; degenerate faces (`n < 3`) contribute nothing.
- **32-bit indices**: required because vertex counts routinely exceed 65535.

### 8.3 Limitations / non-goals (kept minimal by design)
- **Per-vertex (Gouraud) SH**, not per-fragment. Accurate for the dense tessellation of
  mesh-splat output; large triangles would show interpolation softening. (Per-fragment would
  require passing all 16 coefficients through varyings or refetching per fragment — deliberately
  avoided.)
- **No ROS topic streaming for meshes** — mesh loading is file-only (matching the requirement
  that the file path takes mesh-splat PLYs directly).
- **Advanced controls (Sort Backend, Clip Box, WBOIT, Alpha Threshold) are Gaussian-only.** They
  are left visible but have no effect in mesh mode (the mesh is opaque and rendered in the main
  queue regardless). They were intentionally **not** hidden in mesh mode to keep the change
  minimal; gating their visibility per-mode is a possible follow-up.
- **No axis flip** is applied to positions or SH — consistent with the existing Gaussian loader.

### 8.4 Files that were intentionally NOT changed
- `plugins_description.xml` — no new Display class was added (the existing display gained a mode).
- `splat.vert/.frag`, `wboit_*` shaders/materials, the sorters, `perf_monitor`,
  `ply_loader.{hpp,cpp}`, `ply_file_source.*`, `ros_topic_source.*`, `splat_gpu.hpp`,
  `i_splat_source.hpp` — untouched; the mesh path is fully parallel.

---

## 9. Complete File Inventory

**New (8 files):**
- `include/gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.hpp`
- `src/gsplat_rviz_plugin/splat_loaders/mesh_ply_loader.cpp`
- `include/gsplat_rviz_plugin/mesh_splat_cloud.hpp`
- `src/gsplat_rviz_plugin/mesh_splat_cloud.cpp`
- `ogre_media/materials/glsl120/mesh_splat.vert`
- `ogre_media/materials/glsl120/mesh_splat.frag`
- `ogre_media/materials/scripts/mesh_splat.program`
- `ogre_media/materials/scripts/mesh_splat.material`

**Edited (3 files):**
- `CMakeLists.txt` — added the two new `.cpp` sources.
- `include/gsplat_rviz_plugin/displays/gsplat_display.hpp` — forward decl `MeshSplatCloud`;
  `SourceMode::MeshFile`; `loadMeshFile()`; `mesh_cloud_` member.
- `src/gsplat_rviz_plugin/displays/gsplat_display.cpp` — includes; "Mesh PLY File" option +
  picker description; create `mesh_cloud_`; clear it in `reset`/`onSourceModeChanged`; dispatch
  in `onSplatPathChanged`; new `loadMeshFile`; drive mesh in `onShDegreeChanged`; three-mode
  visibility/status handling in `onSourceModeChanged`.

**Crash fix (1 file, included above):**
- `src/gsplat_rviz_plugin/mesh_splat_cloud.cpp` — `MovableObject("GsplatMeshSplatCloud")` to give
  the mesh object a distinct per-node name.
```
