#include "gsplat_rviz_plugin/displays/gsplat_display.hpp"

#include <algorithm>
#include <utility>

#include <QMetaObject>

#include <OgreCamera.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"

#include "gsplat_rviz_plugin/sorters/i_splat_sorter.hpp"
#include "gsplat_rviz_plugin/sorters/sorter_factory.hpp"
#include "gsplat_rviz_plugin/splat_loaders/blob_topic_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/ply_file_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/ros_topic_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/snapshot_ref_source.hpp"
#include "gsplat_rviz_plugin/splat_loaders/tile_stream_source.hpp"
#include "gsplat_rviz_plugin/splat_cloud.hpp"
#include "gsplat_rviz_plugin/splat_gpu.hpp"
#include "gsplat_rviz_plugin/transparency/wboit_compositor.hpp"

namespace gsplat_rviz_plugin
{
namespace displays
{

GsplatDisplay::GsplatDisplay()
{
  // Top-level properties — data source, mode selector, SH degree. Always
  // visible; the two load paths (file and topic) are mutually exclusive at
  // runtime but both shown so users can switch without re-opening a sub-group.
  source_mode_property_ = new rviz_common::properties::EnumProperty(
    "Source", "PLY File",
    "Where splats come from. Exactly one of the two source fields below is "
    "active at a time.",
    this, SLOT(onSourceModeChanged()), this);
  source_mode_property_->addOption("PLY File", static_cast<int>(SourceMode::File));
  source_mode_property_->addOption("SplatArray Topic", static_cast<int>(SourceMode::Topic));
  source_mode_property_->addOption("Snapshot Ref", static_cast<int>(SourceMode::SnapshotRef));
  source_mode_property_->addOption("Blob Topic", static_cast<int>(SourceMode::BlobTopic));
  source_mode_property_->addOption(
    "Tile Stream", static_cast<int>(SourceMode::TileStream));

  splat_path_property_ = new rviz_common::properties::FilePickerProperty(
    "Splat File", "",
    "Path to a 3DGS-format PLY file to visualize.",
    this, SLOT(onSplatPathChanged()),
    this);

  topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Topic", "",
    "gsplat_msgs/msg/SplatArray",
    "gsplat_msgs/SplatArray topic to subscribe to. Each message replaces "
    "the currently displayed splats.",
    this, SLOT(onTopicChanged()), this);

  snapshot_ref_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Snapshot Ref", "",
    "gsplat_msgs/msg/SplatSnapshotRef",
    "gsplat_msgs/SplatSnapshotRef topic containing file:// or absolute path "
    "snapshot references.",
    this, SLOT(onSnapshotRefTopicChanged()), this);

  blob_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Blob Topic", "",
    "gsplat_msgs/msg/SplatBlobChunk",
    "gsplat_msgs/SplatBlobChunk topic. Chunks are assembled into a full "
    "snapshot before replacing the displayed splats.",
    this, SLOT(onBlobTopicChanged()), this);

  // Default mode is PLY File — hide topic fields until the user flips modes.
  topic_property_->hide();
  snapshot_ref_topic_property_->hide();
  blob_topic_property_->hide();

  tile_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Tile Stream Topic", "",
    "gsplat_msgs/msg/SplatTileChunk",
    "gsplat_msgs/SplatTileChunk topic to subscribe to. Chunks are assembled "
    "into a persistent tile cache and committed versions replace the displayed "
    "snapshot.",
    this, SLOT(onTileTopicChanged()), this);
  tile_topic_property_->hide();

  buildAdvancedGroup();
}

// Advanced group layout (collapsed by default):
//
//   Advanced
//   ├── SH Degree
//   ├── Alpha Threshold
//   ├── Sort Backend
//   ├── Clip Box  (BoolProperty, expands to show Min/Max)
//   │   ├── Clip Min
//   │   └── Clip Max
//   ├── WBOIT  (sub-group, collapsed)
//   │   ├── Transparency Mode
//   │   ├── WBOIT Weight Scale
//   │   ├── WBOIT Weight Exponent
//   │   └── WBOIT Alpha Discard
//   └── Scene Capture  (sub-group, collapsed)
//       ├── Save Path
//       └── Capture
void GsplatDisplay::buildAdvancedGroup()
{
  auto * advanced_group = new rviz_common::properties::Property(
    "Advanced", QVariant(),
    "SH degree, alpha threshold, sort backend, ROI clipping, transparency "
    "fallback, and scene capture. Defaults are sensible for most scenes.",
    this);
  advanced_group->collapse();

  sh_degree_property_ = new rviz_common::properties::IntProperty(
    "SH Degree", 0,
    "Spherical harmonics degree used for view-dependent colour (0 = DC only). "
    "Lower values are faster; maximum is set by the loaded data.",
    advanced_group, SLOT(onShDegreeChanged()), this);
  sh_degree_property_->setMin(0);
  sh_degree_property_->setMax(0);

  alpha_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Alpha Threshold", 0.05f,
    "Splat fragments with opacity below this value are discarded.",
    advanced_group, SLOT(onAlphaThresholdChanged()), this);
  alpha_threshold_property_->setMin(0.0f);
  alpha_threshold_property_->setMax(1.0f);

  sorter_kind_property_ = new rviz_common::properties::EnumProperty(
    "Sort Backend", "CUDA",
    "Depth-sort backend. CUDA falls back to CPU if no device is available.",
    advanced_group, SLOT(onSorterKindChanged()), this);
  sorter_kind_property_->addOption("CPU",  static_cast<int>(SorterKind::Cpu));
  sorter_kind_property_->addOption("CUDA", static_cast<int>(SorterKind::Cuda));

  // ROI clip — axis-aligned box in the scene's local frame.
  clip_enabled_property_ = new rviz_common::properties::BoolProperty(
    "Clip Box", false,
    "Cull splats whose centre falls outside [Clip Min, Clip Max].",
    advanced_group, SLOT(onClipChanged()), this);

  clip_min_property_ = new rviz_common::properties::VectorProperty(
    "Clip Min", Ogre::Vector3(-10.0f, -10.0f, -10.0f),
    "Minimum corner of the clip AABB.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  clip_max_property_ = new rviz_common::properties::VectorProperty(
    "Clip Max", Ogre::Vector3(10.0f, 10.0f, 10.0f),
    "Maximum corner of the clip AABB.",
    clip_enabled_property_, SLOT(onClipChanged()), this);

  // WBOIT sub-group.
  auto * wboit_group = new rviz_common::properties::Property(
    "WBOIT", QVariant(),
    "Order-independent transparency. Approximation; use when the sort is "
    "a bottleneck (very large clouds, fast camera motion, streaming splats).",
    advanced_group);
  wboit_group->collapse();

  transparency_mode_property_ = new rviz_common::properties::EnumProperty(
    "Transparency Mode", "Sorted",
    "Sorted = exact back-to-front. WBOIT = order-independent approximation.",
    wboit_group, SLOT(onTransparencyModeChanged()), this);
  transparency_mode_property_->addOption("Sorted", 0);
  transparency_mode_property_->addOption("WBOIT",  1);

  wboit_weight_scale_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Weight Scale", 5.0f,
    "Depth-discrimination strength in the WBOIT weight function.",
    wboit_group, SLOT(onWboitTuningChanged()), this);
  wboit_weight_scale_property_->setMin(0.1f);
  wboit_weight_scale_property_->setMax(50.0f);

  wboit_weight_exponent_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Weight Exponent", 2.0f,
    "Alpha-suppression power in the WBOIT weight function.",
    wboit_group, SLOT(onWboitTuningChanged()), this);
  wboit_weight_exponent_property_->setMin(0.5f);
  wboit_weight_exponent_property_->setMax(6.0f);

  wboit_alpha_discard_property_ = new rviz_common::properties::FloatProperty(
    "WBOIT Alpha Discard", 0.01f,
    "Fragment alpha cutoff in the WBOIT accumulation pass.",
    wboit_group, SLOT(onWboitTuningChanged()), this);
  wboit_alpha_discard_property_->setMin(0.0f);
  wboit_alpha_discard_property_->setMax(0.1f);

  // Scene Capture sub-group — save the current viewport to a PNG/JPEG on disk.
  auto * capture_group = new rviz_common::properties::Property(
    "Scene Capture", QVariant(),
    "Capture the current RViz viewport to an image file.",
    advanced_group);
  capture_group->collapse();

  capture_path_property_ = new rviz_common::properties::StringProperty(
    "Save Path", "/tmp/gsplat_capture.png",
    "Absolute file path for the captured image (PNG or JPEG, "
    "determined by extension).",
    capture_group, SLOT(onCaptureTriggerChanged()), this);

  // BoolProperty acts as a momentary trigger: checking it fires the capture
  // and the slot immediately resets it to false.
  capture_trigger_property_ = new rviz_common::properties::BoolProperty(
    "Capture", false,
    "Check to save the current viewport to the path above.",
    capture_group, SLOT(onCaptureTriggerChanged()), this);
}

GsplatDisplay::~GsplatDisplay()
{
  if (compositor_viewport_) {
    transparency::wboit_compositor::detach(compositor_viewport_);
    compositor_viewport_ = nullptr;
    wboit_compositor_active_ = false;
  }

  ++source_gen_;
  source_.reset();
  source_kind_ = SourceKind::None;
}

void GsplatDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  splat_cloud_ = std::make_unique<SplatCloud>(scene_node_);

  rebuildSorter();

  topic_property_->initialize(context_->getRosNodeAbstraction());
  snapshot_ref_topic_property_->initialize(context_->getRosNodeAbstraction());
  blob_topic_property_->initialize(context_->getRosNodeAbstraction());
  tile_topic_property_->initialize(context_->getRosNodeAbstraction());

  // Seed uniforms from the initial property values.
  onClipChanged();
  onWboitTuningChanged();
  applyTransparencyMode();
}

void GsplatDisplay::onEnable()
{
  if (currentMode() == SourceMode::Topic) {
    onTopicChanged();
  } else if (currentMode() == SourceMode::SnapshotRef) {
    onSnapshotRefTopicChanged();
  } else if (currentMode() == SourceMode::BlobTopic) {
    onBlobTopicChanged();
  } else if (currentMode() == SourceMode::TileStream) {
    onTileTopicChanged();
  }
  applyTransparencyMode();
}

void GsplatDisplay::onDisable()
{
  if (source_kind_ == SourceKind::Topic ||
      source_kind_ == SourceKind::SnapshotRef ||
      source_kind_ == SourceKind::BlobTopic ||
      source_kind_ == SourceKind::TileStream) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
  }
  if (wboit_compositor_active_) {
    transparency::wboit_compositor::disable(compositor_viewport_);
    wboit_compositor_active_ = false;
  }
  if (splat_cloud_) splat_cloud_->setOitEnabled(false);
}

void GsplatDisplay::reset()
{
  rviz_common::Display::reset();
  if (splat_cloud_) splat_cloud_->clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);
}

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

void GsplatDisplay::onShDegreeChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setShDegree(sh_degree_property_->getInt());
  }
}

void GsplatDisplay::onAlphaThresholdChanged()
{
  if (splat_cloud_) {
    splat_cloud_->setAlphaThreshold(alpha_threshold_property_->getFloat());
  }
}

void GsplatDisplay::onTopicChanged()
{
  if (currentMode() != SourceMode::Topic) return;

  if (!isEnabled()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    return;
  }

  const std::string topic = topic_property_->getTopicStd();
  if (topic.empty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Topic", "No topic selected.");
    return;
  }

  auto node_abs = context_ ? context_->getRosNodeAbstraction().lock() : nullptr;
  if (!node_abs) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic", "RViz ROS node unavailable.");
    return;
  }

  installSource(
    std::make_unique<RosTopicSource>(node_abs->get_raw_node(), topic),
    SourceKind::Topic);
}

void GsplatDisplay::onSnapshotRefTopicChanged()
{
  if (currentMode() != SourceMode::SnapshotRef) return;

  if (!isEnabled()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    return;
  }

  const std::string topic = snapshot_ref_topic_property_->getTopicStd();
  if (topic.empty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Snapshot Ref", "No snapshot ref topic selected.");
    return;
  }

  auto node_abs = context_ ? context_->getRosNodeAbstraction().lock() : nullptr;
  if (!node_abs) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Snapshot Ref", "RViz ROS node unavailable.");
    return;
  }

  installSource(
    std::make_unique<SnapshotRefSource>(node_abs->get_raw_node(), topic),
    SourceKind::SnapshotRef);
}

void GsplatDisplay::onBlobTopicChanged()
{
  if (currentMode() != SourceMode::BlobTopic) return;

  if (!isEnabled()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    return;
  }

  const std::string topic = blob_topic_property_->getTopicStd();
  if (topic.empty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Blob Topic", "No blob topic selected.");
    return;
  }

  auto node_abs = context_ ? context_->getRosNodeAbstraction().lock() : nullptr;
  if (!node_abs) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Blob Topic", "RViz ROS node unavailable.");
    return;
  }

  installSource(
    std::make_unique<BlobTopicSource>(node_abs->get_raw_node(), topic),
    SourceKind::BlobTopic);
}

void GsplatDisplay::onTileTopicChanged()
{
  if (currentMode() != SourceMode::TileStream) return;

  if (!isEnabled()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    return;
  }

  const std::string topic = tile_topic_property_->getTopicStd();
  if (topic.empty()) {
    ++source_gen_;
    source_.reset();
    source_kind_ = SourceKind::None;
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Tile Stream", "No tile stream topic selected.");
    return;
  }

  auto node_abs = context_ ? context_->getRosNodeAbstraction().lock() : nullptr;
  if (!node_abs) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Tile Stream", "RViz ROS node unavailable.");
    return;
  }

  installSource(
    std::make_unique<TileStreamSource>(node_abs->get_raw_node(), topic),
    SourceKind::TileStream);
}

void GsplatDisplay::onSorterKindChanged()
{
  rebuildSorter();
}

GsplatDisplay::SourceMode GsplatDisplay::currentMode() const
{
  return source_mode_property_
    ? static_cast<SourceMode>(source_mode_property_->getOptionInt())
    : SourceMode::File;
}

void GsplatDisplay::onSourceModeChanged()
{
  ++source_gen_;
  source_.reset();
  source_kind_ = SourceKind::None;
  if (splat_cloud_) splat_cloud_->clear();
  sh_degree_property_->setMax(0);
  sh_degree_property_->setValue(0);

  const auto mode = currentMode();
  splat_path_property_->setHidden(mode != SourceMode::File);
  topic_property_->setHidden(mode != SourceMode::Topic);
  snapshot_ref_topic_property_->setHidden(mode != SourceMode::SnapshotRef);
  blob_topic_property_->setHidden(mode != SourceMode::BlobTopic);
  tile_topic_property_->setHidden(mode != SourceMode::TileStream);

  deleteStatus("Splat File");
  deleteStatus("Topic");
  deleteStatus("Snapshot Ref");
  deleteStatus("Blob Topic");
  deleteStatus("Tile Stream");

  if (mode == SourceMode::File) {
    onSplatPathChanged();
  } else if (mode == SourceMode::Topic) {
    onTopicChanged();
  } else if (mode == SourceMode::SnapshotRef) {
    onSnapshotRefTopicChanged();
  } else if (mode == SourceMode::BlobTopic) {
    onBlobTopicChanged();
  } else if (mode == SourceMode::TileStream) {
    onTileTopicChanged();
  }
}

void GsplatDisplay::onClipChanged()
{
  if (!splat_cloud_) return;
  splat_cloud_->setClipEnabled(clip_enabled_property_->getBool());
  splat_cloud_->setClipBox(
    clip_min_property_->getVector(),
    clip_max_property_->getVector());
  if (context_) context_->queueRender();
}

void GsplatDisplay::onTransparencyModeChanged()
{
  applyTransparencyMode();
}

void GsplatDisplay::onWboitTuningChanged()
{
  if (!splat_cloud_) return;
  splat_cloud_->setWboitWeightScale(wboit_weight_scale_property_->getFloat());
  splat_cloud_->setWboitWeightExponent(wboit_weight_exponent_property_->getFloat());
  splat_cloud_->setWboitAlphaDiscard(wboit_alpha_discard_property_->getFloat());
  if (context_) context_->queueRender();
}

void GsplatDisplay::onCaptureTriggerChanged()
{
  if (!capture_trigger_property_->getBool()) return;

  // Reset the checkbox immediately so it behaves like a momentary button.
  capture_trigger_property_->blockSignals(true);
  capture_trigger_property_->setBool(false);
  capture_trigger_property_->blockSignals(false);

  const std::string path = capture_path_property_->getStdString();
  if (path.empty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Scene Capture", "Save path is empty.");
    return;
  }

  captureScreenshot(path);
  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "Scene Capture", QString("Saved to %1").arg(QString::fromStdString(path)));
}

void GsplatDisplay::applyTransparencyMode()
{
  const bool wboit = (transparency_mode_property_ &&
                      transparency_mode_property_->getOptionInt() == 1);

  if (splat_cloud_) splat_cloud_->setOitEnabled(wboit);

  if (auto * vp = resolveViewport()) {
    if (wboit) {
      transparency::wboit_compositor::ensureDefined();
      transparency::wboit_compositor::enable(vp);
      compositor_viewport_ = vp;
      wboit_compositor_active_ = true;
    } else if (wboit_compositor_active_) {
      transparency::wboit_compositor::disable(compositor_viewport_);
      wboit_compositor_active_ = false;
    }
  }

  if (context_) context_->queueRender();
}

Ogre::Viewport * GsplatDisplay::resolveViewport() const
{
  if (!context_) return nullptr;
  auto * vm = context_->getViewManager();
  if (!vm) return nullptr;
  auto * vc = vm->getCurrent();
  if (!vc) return nullptr;
  auto * cam = vc->getCamera();
  if (!cam) return nullptr;
  return cam->getViewport();
}

void GsplatDisplay::rebuildSorter()
{
  if (!splat_cloud_) return;

  const auto kind = static_cast<SorterKind>(
    sorter_kind_property_ ? sorter_kind_property_->getOptionInt()
                          : static_cast<int>(SorterKind::Cuda));
  splat_cloud_->setSorter(makeSorter(kind));
}

void GsplatDisplay::installSource(
  std::unique_ptr<ISplatSource> source, SourceKind kind)
{
  const uint64_t gen = ++source_gen_;
  source_.reset();
  source_kind_ = kind;
  source_ = std::move(source);

  source_->start(
    [this, kind, gen](LoadResult r) {
      QMetaObject::invokeMethod(
        this,
        [this, kind, gen, r = std::move(r)]() mutable {
          onLoadResult(std::move(r), kind, gen);
        });
    });
}

void GsplatDisplay::onLoadResult(
  LoadResult result, SourceKind kind, uint64_t gen)
{
  if (gen != source_gen_ || !splat_cloud_) return;

  const char * status_key =
    (kind == SourceKind::Topic) ? "Topic" :
    (kind == SourceKind::SnapshotRef) ? "Snapshot Ref" :
    (kind == SourceKind::BlobTopic) ? "Blob Topic" :
    (kind == SourceKind::TileStream) ? "Tile Stream" :
    "Splat File";

  if (!result.ok()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      status_key, QString::fromStdString(result.error));
    return;
  }

  const int count = static_cast<int>(result.splats.size());
  const int sh_degree = result.sh_degree;

  splat_cloud_->setSplats(std::move(result.splats), sh_degree);

  sh_degree_property_->setMax(sh_degree);
  if (kind == SourceKind::Topic ||
      kind == SourceKind::SnapshotRef ||
      kind == SourceKind::BlobTopic ||
      kind == SourceKind::TileStream) {
    const int current = sh_degree_property_->getInt();
    if (current > sh_degree) sh_degree_property_->setValue(sh_degree);
  } else {
    sh_degree_property_->setValue(std::min(1, sh_degree));
  }

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    status_key,
    (kind == SourceKind::Topic ||
     kind == SourceKind::SnapshotRef ||
     kind == SourceKind::BlobTopic ||
     kind == SourceKind::TileStream)
      ? QString("Received %1 gaussians (SH degree %2)").arg(count).arg(sh_degree)
      : QString("Loaded %1 gaussians").arg(count));
}

void GsplatDisplay::captureScreenshot(const std::string & path)
{
  auto * vm = dynamic_cast<rviz_common::VisualizationManager *>(context_);
  if (!vm) return;
  auto * panel = vm->getRenderPanel();
  if (!panel) return;
  auto * win = panel->getRenderWindow();
  if (!win) return;
  win->captureScreenShot(path);
}

}  // namespace displays
}  // namespace gsplat_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  gsplat_rviz_plugin::displays::GsplatDisplay,
  rviz_common::Display)
