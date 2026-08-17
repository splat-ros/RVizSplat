#include "gsplat_rviz_plugin/mesh_splat_cloud.hpp"

// GLEW must be included before any other OpenGL header.
#include <GL/glew.h>

#include <algorithm>
#include <cstring>
#include <vector>

#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>
#include <OgreMaterialManager.h>
#include <OgreRenderQueue.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVertexIndexData.h>

namespace gsplat_rviz_plugin
{

static const Ogre::String MOT_MESH_SPLAT_CLOUD = "MeshSplatCloud";

// IEEE 754 float32 → binary16 (round-to-nearest-even, saturate to inf).
static inline uint16_t floatToHalf(float f)
{
  uint32_t x;
  std::memcpy(&x, &f, 4);
  const uint32_t sign = (x >> 16) & 0x8000u;
  const uint32_t e32  = (x >> 23) & 0xFFu;
  const uint32_t m32  = x & 0x7FFFFFu;

  if (e32 == 0xFFu) {
    return uint16_t(sign | 0x7C00u | (m32 ? 0x200u : 0u));
  }
  int e = int(e32) - 127 + 15;
  if (e >= 0x1F) return uint16_t(sign | 0x7C00u);
  if (e <= 0) {
    if (e < -10) return uint16_t(sign);
    const uint32_t m = m32 | 0x800000u;
    const uint32_t shift = uint32_t(14 - e);
    const uint32_t half  = (m >> shift) + ((m >> (shift - 1)) & 1u);
    return uint16_t(sign | half);
  }
  uint32_t m = (m32 >> 13) + ((m32 >> 12) & 1u);
  if (m & 0x400u) { m = 0; ++e; if (e >= 0x1F) return uint16_t(sign | 0x7C00u); }
  return uint16_t(sign | (uint32_t(e) << 10) | m);
}

MeshSplatCloud::MeshSplatCloud(Ogre::SceneNode * parent_node)
  // Distinct from SplatCloud's empty name — Ogre keys attached objects by
  // name per scene node, and both clouds share the display's scene node.
  : Ogre::MovableObject("GsplatMeshSplatCloud")
{
  scene_manager_ = parent_node->getCreator();

  render_op_.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  render_op_.useIndexes    = true;
  render_op_.vertexData    = new Ogre::VertexData();
  render_op_.indexData     = new Ogre::IndexData();
  render_op_.vertexData->vertexDeclaration->addElement(
    0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

  material_ = Ogre::MaterialManager::getSingleton().getByName(
    "gsplat_rviz_plugin/MeshSplat", "rviz_rendering");
  material_->load();

  parent_node->attachObject(this);
  scene_manager_->addRenderObjectListener(this);
}

MeshSplatCloud::~MeshSplatCloud()
{
  if (scene_manager_) {
    scene_manager_->removeRenderObjectListener(this);
  }
  if (auto * node = getParentSceneNode()) {
    node->detachObject(this);
  }
  destroyTBO();
  delete render_op_.vertexData;
  delete render_op_.indexData;
}

void MeshSplatCloud::destroyTBO()
{
  if (sh_tbo_tex_) { glDeleteTextures(1, &sh_tbo_tex_); sh_tbo_tex_ = 0; }
  if (sh_tbo_buf_) { glDeleteBuffers(1, &sh_tbo_buf_);  sh_tbo_buf_ = 0; }
}

void MeshSplatCloud::setMesh(MeshSplatData data)
{
  clear();

  vertex_count_     = data.vertexCount();
  triangle_count_   = data.triangleCount();
  max_sh_degree_    = data.sh_degree;
  active_sh_degree_ = std::min(1, data.sh_degree);  // default SH1 if available
  sh_stride_        = data.sh_stride;
  if (vertex_count_ == 0 || triangle_count_ == 0) { clear(); return; }

  // Bounds from positions.
  bounds_.setNull();
  for (uint32_t i = 0; i < vertex_count_; ++i) {
    bounds_.merge(Ogre::Vector3(
      data.positions[i * 3 + 0],
      data.positions[i * 3 + 1],
      data.positions[i * 3 + 2]));
  }

  auto & bm = Ogre::HardwareBufferManager::getSingleton();

  // Position VBO (source 0).
  auto vbuf = bm.createVertexBuffer(
    3 * sizeof(float), vertex_count_, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  vbuf->writeData(0, vbuf->getSizeInBytes(), data.positions.data(), true);
  render_op_.vertexData->vertexStart = 0;
  render_op_.vertexData->vertexCount = vertex_count_;
  render_op_.vertexData->vertexBufferBinding->setBinding(0, vbuf);

  // 32-bit index buffer — vertex counts routinely exceed 65535.
  auto ibuf = bm.createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_32BIT, data.indices.size(),
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  ibuf->writeData(0, ibuf->getSizeInBytes(), data.indices.data(), true);
  render_op_.indexData->indexBuffer = ibuf;
  render_op_.indexData->indexStart  = 0;
  render_op_.indexData->indexCount  = data.indices.size();

  // SH coefficients staged for a deferred GL upload (TBO needs GL current).
  pending_sh_     = std::move(data.sh);
  upload_pending_ = true;

  if (auto * node = getParentSceneNode()) node->needUpdate();
}

void MeshSplatCloud::clear()
{
  vertex_count_   = 0;
  triangle_count_ = 0;
  max_sh_degree_  = 0;
  active_sh_degree_ = 0;
  sh_stride_      = 1;
  upload_pending_ = false;
  pending_sh_.clear();
  pending_sh_.shrink_to_fit();
  destroyTBO();
  render_op_.vertexData->vertexCount = 0;
  render_op_.vertexData->vertexBufferBinding->unsetAllBindings();
  render_op_.indexData->indexCount = 0;
  render_op_.indexData->indexBuffer.reset();
  bounds_.setNull();
  if (auto * node = getParentSceneNode()) node->needUpdate();
}

void MeshSplatCloud::setShDegree(int d)
{
  active_sh_degree_ = std::clamp(d, 0, max_sh_degree_);
}

void MeshSplatCloud::uploadTBO()
{
  destroyTBO();
  if (pending_sh_.empty()) return;

  // One RGBA16F texel per SH coefficient (RGB used, A = 0).
  std::vector<uint16_t> packed(pending_sh_.size() / 3 * 4);
  const size_t coeffs = pending_sh_.size() / 3;
  for (size_t c = 0; c < coeffs; ++c) {
    packed[c * 4 + 0] = floatToHalf(pending_sh_[c * 3 + 0]);
    packed[c * 4 + 1] = floatToHalf(pending_sh_[c * 3 + 1]);
    packed[c * 4 + 2] = floatToHalf(pending_sh_[c * 3 + 2]);
    packed[c * 4 + 3] = 0;
  }
  pending_sh_.clear();
  pending_sh_.shrink_to_fit();

  glGenBuffers(1, &sh_tbo_buf_);
  glBindBuffer(GL_TEXTURE_BUFFER, sh_tbo_buf_);
  glBufferData(
    GL_TEXTURE_BUFFER,
    static_cast<GLsizeiptr>(packed.size() * sizeof(uint16_t)),
    packed.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_TEXTURE_BUFFER, 0);

  glGenTextures(1, &sh_tbo_tex_);
  glBindTexture(GL_TEXTURE_BUFFER, sh_tbo_tex_);
  glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA16F, sh_tbo_buf_);
  glBindTexture(GL_TEXTURE_BUFFER, 0);
}

// ── MovableObject ─────────────────────────────────────────────────────────

const Ogre::String & MeshSplatCloud::getMovableType() const { return MOT_MESH_SPLAT_CLOUD; }

Ogre::Real MeshSplatCloud::getBoundingRadius() const
{
  return bounds_.isNull() ? 0.0f : bounds_.getHalfSize().length();
}

void MeshSplatCloud::_updateRenderQueue(Ogre::RenderQueue * queue)
{
  if (triangle_count_ == 0) return;
  queue->addRenderable(this, Ogre::RENDER_QUEUE_MAIN, mRenderQueuePriority);
}

void MeshSplatCloud::visitRenderables(Ogre::Renderable::Visitor * visitor, bool)
{
  if (triangle_count_ > 0) visitor->visit(this, 0, false);
}

// ── Renderable ────────────────────────────────────────────────────────────

void MeshSplatCloud::getRenderOperation(Ogre::RenderOperation & op)
{
  op = render_op_;
  op.numberOfInstances = 1;
}

void MeshSplatCloud::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  auto * node = getParentSceneNode();
  *xform = node ? node->_getFullTransform() : Ogre::Matrix4::IDENTITY;
}

Ogre::Real MeshSplatCloud::getSquaredViewDepth(const Ogre::Camera * cam) const
{
  if (bounds_.isNull()) return 0.0f;
  return (bounds_.getCenter() - cam->getDerivedPosition()).squaredLength();
}

const Ogre::LightList & MeshSplatCloud::getLights() const { return queryLights(); }

// ── RenderObjectListener ──────────────────────────────────────────────────

void MeshSplatCloud::notifyRenderSingleObject(
  Ogre::Renderable * rend,
  const Ogre::Pass * pass,
  const Ogre::AutoParamDataSource *,
  const Ogre::LightList *,
  bool)
{
  if (rend != this) return;

  if (upload_pending_) {
    uploadTBO();
    upload_pending_ = false;
  }

  if (pass && pass->hasVertexProgram()) {
    auto params = pass->getVertexProgramParameters();
    if (params) {
      params->setIgnoreMissingParams(true);
      params->setNamedConstant("sh_degree", active_sh_degree_);
      params->setNamedConstant("u_sh_stride", sh_stride_);
    }
  }

  if (sh_tbo_tex_) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, sh_tbo_tex_);
  }
}

}  // namespace gsplat_rviz_plugin
