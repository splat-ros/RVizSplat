#ifndef GSPLAT_RVIZ_PLUGIN__DISPLAYS__GSPLAT_DISPLAY_HPP_
#define GSPLAT_RVIZ_PLUGIN__DISPLAYS__GSPLAT_DISPLAY_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include <OgreVector3.h>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "gsplat_rviz_plugin/splat_loaders/i_splat_source.hpp"
#include "gsplat_rviz_plugin/visibility_control.hpp"

namespace Ogre
{
class Viewport;
}

namespace gsplat_rviz_plugin
{
class SplatCloud;

namespace displays
{

class GSPLAT_RVIZ_PLUGIN_PUBLIC GsplatDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  GsplatDisplay();
  ~GsplatDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void reset() override;

private Q_SLOTS:
  void onSourceModeChanged();
  void onSplatPathChanged();
  void onShDegreeChanged();
  void onAlphaThresholdChanged();
  void onTopicChanged();
  void onSnapshotRefTopicChanged();
  void onBlobTopicChanged();
  void onTileTopicChanged();
  void onSorterKindChanged();
  void onClipChanged();
  void onTransparencyModeChanged();
  void onWboitTuningChanged();
  void onCaptureTriggerChanged();

private:
  enum class SourceKind { None, File, Topic, SnapshotRef, BlobTopic, TileStream };
  enum class SourceMode { File = 0, Topic = 1, SnapshotRef = 2, BlobTopic = 3, TileStream = 4 };

  void rebuildSorter();
  void installSource(std::unique_ptr<ISplatSource> source, SourceKind kind);
  void onLoadResult(LoadResult result, SourceKind kind, uint64_t gen);
  SourceMode currentMode() const;
  void captureScreenshot(const std::string & path);

  // Push transparency_mode_property_ → SplatCloud::setOitEnabled and
  // attach/detach the WBOIT compositor on the current viewport. Called
  // from property slots on the Qt main thread, which is safe (Ogre
  // chain mutations here run between render frames, not during a chain
  // walk).
  void applyTransparencyMode();

  // Resolve the Ogre viewport RViz is rendering to, or nullptr if it
  // isn't ready yet (can happen very early during onInitialize). The
  // view controller is created by RViz before Displays, so this is
  // non-null from any property-slot call.
  Ogre::Viewport * resolveViewport() const;

  // UI construction — builds the "Advanced" group (SH degree, alpha
  // threshold, sort backend, clip box, WBOIT sub-group) and parents it
  // under this display. Members that live inside Advanced (sh_degree_,
  // alpha_threshold_, sorter_kind_, clip_*, transparency_mode_, wboit_*)
  // are assigned by this helper.
  void buildAdvancedGroup();

  // Top-level (always visible).
  rviz_common::properties::EnumProperty *       source_mode_property_;
  rviz_common::properties::FilePickerProperty * splat_path_property_;
  rviz_common::properties::RosTopicProperty *   topic_property_;
  rviz_common::properties::RosTopicProperty *   snapshot_ref_topic_property_;
  rviz_common::properties::RosTopicProperty *   blob_topic_property_;
  rviz_common::properties::RosTopicProperty *   tile_topic_property_;

  // Under "Advanced" group.
  rviz_common::properties::IntProperty *        sh_degree_property_;
  rviz_common::properties::FloatProperty *      alpha_threshold_property_;
  rviz_common::properties::EnumProperty *       sorter_kind_property_;

  // Under "Advanced" group — ROI clip AABB in the scene's local frame.
  rviz_common::properties::BoolProperty *       clip_enabled_property_;
  rviz_common::properties::VectorProperty *     clip_min_property_;
  rviz_common::properties::VectorProperty *     clip_max_property_;

  // Under "Advanced / WBOIT" sub-group. Sorted is the default transparency
  // mode; WBOIT is an order-independent approximation for cases where the
  // sort is the bottleneck.
  rviz_common::properties::EnumProperty *       transparency_mode_property_;
  rviz_common::properties::FloatProperty *      wboit_weight_scale_property_;
  rviz_common::properties::FloatProperty *      wboit_weight_exponent_property_;
  rviz_common::properties::FloatProperty *      wboit_alpha_discard_property_;

  // Under "Advanced / Scene Capture" sub-group.
  rviz_common::properties::StringProperty *     capture_path_property_;
  rviz_common::properties::BoolProperty *       capture_trigger_property_;

  std::unique_ptr<SplatCloud>   splat_cloud_;
  std::unique_ptr<ISplatSource> source_;
  SourceKind                    source_kind_ = SourceKind::None;
  // Bumped each time source_ is (re)assigned or cleared. Queued main-thread
  // deliveries from a prior source compare the captured generation against
  // the current one and drop if stale — prevents an in-flight callback from
  // clobbering state after the user switches topic/file.
  uint64_t                      source_gen_  = 0;

  // WBOIT compositor attach state. compositor_viewport_ stays set once
  // the compositor is attached to the viewport, even after a "disable"
  // toggle — Ogre's chain entry is kept, only the enabled flag flips.
  // Only cleared on destruction, where we fully detach.
  Ogre::Viewport * compositor_viewport_{nullptr};
  bool             wboit_compositor_active_{false};
};

}  // namespace displays
}  // namespace gsplat_rviz_plugin

#endif  // GSPLAT_RVIZ_PLUGIN__DISPLAYS__GSPLAT_DISPLAY_HPP_
