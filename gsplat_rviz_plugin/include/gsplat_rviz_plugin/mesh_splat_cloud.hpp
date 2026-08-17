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

// Renders a "mesh splat" — an opaque triangle mesh whose vertices carry SH
// colour.  Drawn in one indexed draw call: an Ogre position VBO + 32-bit IBO,
// with the per-vertex SH coefficients held in a GL_RGBA16F texture-buffer
// object indexed by gl_VertexID in the vertex shader.
//
// This mirrors SplatCloud's TBO-bind pattern but is far simpler: meshes are
// opaque, so there is no covariance, depth sort, or WBOIT.
class GSPLAT_RVIZ_PLUGIN_PUBLIC MeshSplatCloud
  : public Ogre::MovableObject,
    public Ogre::Renderable,
    public Ogre::RenderObjectListener
{
public:
  explicit MeshSplatCloud(Ogre::SceneNode * parent_node);
  ~MeshSplatCloud() override;

  // Upload a loaded mesh.  Position/index buffers are created immediately
  // (Ogre HardwareBufferManager); the SH TBO is created lazily on the first
  // render so a GL context is guaranteed current.
  void setMesh(MeshSplatData data);
  void clear();

  // Active SH degree sent to the shader (clamped to [0, max available]).
  void setShDegree(int d);
  int  getMaxShDegree()    const { return max_sh_degree_; }
  uint32_t getVertexCount() const { return vertex_count_; }
  uint32_t getTriangleCount() const { return triangle_count_; }

  // ── Ogre::MovableObject ────────────────────────────────────────────────
  const Ogre::String & getMovableType() const override;
  const Ogre::AxisAlignedBox & getBoundingBox() const override { return bounds_; }
  Ogre::Real getBoundingRadius() const override;
  void _updateRenderQueue(Ogre::RenderQueue * queue) override;
  void visitRenderables(Ogre::Renderable::Visitor * visitor, bool debug = false) override;

  // ── Ogre::Renderable ──────────────────────────────────────────────────
  const Ogre::MaterialPtr & getMaterial() const override { return material_; }
  void getRenderOperation(Ogre::RenderOperation & op) override;
  void getWorldTransforms(Ogre::Matrix4 * xform) const override;
  Ogre::Real getSquaredViewDepth(const Ogre::Camera * cam) const override;
  const Ogre::LightList & getLights() const override;

  // ── Ogre::RenderObjectListener ────────────────────────────────────────
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

  // Staged SH coefficients (coeff-major RGB per vertex) until the GL upload.
  std::vector<float> pending_sh_;
  bool     upload_pending_ = false;
  uint32_t vertex_count_   = 0;
  uint32_t triangle_count_ = 0;
  int      max_sh_degree_    = 0;
  int      active_sh_degree_ = 0;
  int      sh_stride_        = 1;   // coeffs per vertex = (max_sh_degree_+1)²

  // Raw GL TBO handles (GLuint).
  uint32_t sh_tbo_buf_ = 0;
  uint32_t sh_tbo_tex_ = 0;
};

}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__MESH_SPLAT_CLOUD_HPP_
