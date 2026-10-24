// treemapper_node: thin ROS 1 wrapper around a registered treemapper::Mapper (default cand_obj).
//
// Inputs (all topic and frame names are parameters, see README.md and config/treemapper.yaml):
//   disparity (sensor_msgs/Image 32FC1) + left/right camera_info, detections (treemapper_msgs/Detection2DArray
//   or the existing detector's spot_spraying_msgs/DetectionResult, decoded without that package), drone pose
//   (TF world <- body, or a geometry_msgs/PoseStamped topic), /mission_state (std_msgs/String), hop target
//   (std_msgs/Int32, in THIS mapper's ID space).
// Outputs: ~map (TreeMap, latched, map_rate Hz + phase boundaries + after every sprayed change), ~markers,
//   ~health (Health heartbeat, also while asleep), ~counters (DropCounters per stage), ~stage_timing,
//   services ~get_trees and ~set_tree_sprayed.
//
// Gate behaviour on missing information (brief section 7.6), each one logged when it triggers:
//   * no / invalid calibration -> fail closed: the mapper is not created, every frame is dropped and
//     counted (frames_dropped_no_calibration), health = HOLDING (waiting) or FAULT (rejected calibration).
//   * pose unavailable at a frame stamp -> fail closed: the frame reaches the mapper with pose.valid = false,
//     which holds processing (counted pose_unavailable), health = HOLDING.
//   * no / unknown mission phase -> fail open: phase UNKNOWN = scanning behaviour (permissive), WARN once per
//     distinct string, listed in health.warnings.
//   * no hop target -> -1 (none).
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <treemapper_msgs/GetTrees.h>
#include <treemapper_msgs/Health.h>
#include <treemapper_msgs/MissionPhase.h>
#include <treemapper_msgs/SetTreeSprayed.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <set>
#include <sstream>

#include "treemapper/mapper_registry.hpp"
#include "treemapper_ros/conversions.hpp"
#include "treemapper_ros/input_sync.hpp"
#include "treemapper_ros/node_support.hpp"

namespace treemapper_ros {

using treemapper::MissionPhase;
using treemapper::TreeId;

namespace {
double nowSec() { return ros::Time::now().toSec(); }
double wallMs(std::chrono::steady_clock::time_point a) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
}
std::string xmlToString(XmlRpc::XmlRpcValue& v) {
  switch (v.getType()) {
    case XmlRpc::XmlRpcValue::TypeBoolean: return static_cast<bool>(v) ? "true" : "false";
    case XmlRpc::XmlRpcValue::TypeInt: return std::to_string(static_cast<int>(v));
    case XmlRpc::XmlRpcValue::TypeDouble: {
      std::ostringstream s;
      s.precision(17);
      s << static_cast<double>(v);
      return s.str();
    }
    case XmlRpc::XmlRpcValue::TypeString: return static_cast<std::string>(v);
    default: throw std::invalid_argument("mapper_overrides values must be scalars");
  }
}
}  // namespace

class TreemapperNode {
 public:
  TreemapperNode(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("mapper", mapper_name_, "cand_obj");
    if (!pnh_.getParam("mapper_config_file", config_file_) || config_file_.empty()) {
      fatal("parameter ~mapper_config_file is required (path to config/<mapper>.cfg)");
    }
    if (config_file_[0] != '/') config_file_ = ros::package::getPath("treemapper_ros") + "/" + config_file_;
    // Load + validate the configuration now (fail at startup, not at the first camera_info).
    try {
      mapper_config_ = readCfgFile(config_file_);
      std::map<std::string, std::string> ov;
      XmlRpc::XmlRpcValue x;
      if (pnh_.getParam("mapper_overrides", x)) {
        if (x.getType() != XmlRpc::XmlRpcValue::TypeStruct) throw std::invalid_argument("~mapper_overrides must be a dict");
        for (auto it = x.begin(); it != x.end(); ++it) ov[it->first] = xmlToString(it->second);
      }
      for (const auto& kv : ov) ROS_WARN("mapper override: %s = %s", kv.first.c_str(), kv.second.c_str());
      mapper_config_ = applyOverrides(mapper_config_, ov);
      // dry run with a dummy calibration: unknown keys / bad values / unknown mapper throw here
      treemapper::StereoCalibration dummy;
      dummy.width = 640; dummy.height = 400; dummy.fx = dummy.fy = 400; dummy.cx = 320; dummy.cy = 200;
      dummy.baseline = 0.1; dummy.stride = 4;
      treemapper::createMapper(mapper_name_, mapper_config_, dummy);
    } catch (const std::exception& e) {
      fatal(std::string("mapper configuration rejected: ") + e.what());
    }

    pnh_.param<std::string>("world_frame", world_frame_, "world_nwu");
    pnh_.param<std::string>("body_frame", body_frame_, "body");
    pnh_.param<std::string>("pose_source", pose_source_, "tf");
    pnh_.param<double>("pose_wait", pose_wait_, 0.1);
    pnh_.param<double>("pose_max_age", pose_max_age_, 0.5);
    pnh_.param<std::string>("camera_mount_source", mount_source_, "tf");
    pnh_.param<std::string>("camera_frame", camera_frame_, "");
    pnh_.param<bool>("camera_frame_is_optical", camera_frame_is_optical_, true);
    pnh_.param<int>("disparity_stride", stride_, 4);
    pnh_.param<double>("disparity_quantum_px", quantum_px_, 0.0);
    {  // string or number (roslaunch turns "4.30" into a double)
      XmlRpc::XmlRpcValue v;
      if (pnh_.getParam("disparity_offset", v)) disp_offset_mode_ = xmlToString(v);
    }
    pnh_.param<double>("detection_scale", detection_scale_, 1.0);
    FrameSync::Params sp;
    pnh_.param<double>("sync_wait", sp.wait_s, 0.5);
    pnh_.param<double>("sync_nearest_tol", sp.nearest_tol_s, 0.02);
    int cache = 400;
    pnh_.param<int>("disparity_cache", cache, 400);
    sp.cache = static_cast<std::size_t>(std::max(1, cache));
    pnh_.param<bool>("sync_early_resolve", sp.early_resolve, true);
    sync_ = FrameSync(sp);
    pnh_.param<double>("heartbeat_frame_period", heartbeat_frame_period_, 0.2);
    pnh_.param<double>("health_rate", health_rate_, 2.0);
    pnh_.param<double>("map_rate", map_rate_, 5.0);
    pnh_.param<double>("input_stale_warn", input_stale_warn_, 2.0);
    if (pose_source_ != "tf" && pose_source_ != "pose_topic") fatal("~pose_source must be 'tf' or 'pose_topic'");
    if (mount_source_ != "tf" && mount_source_ != "param") fatal("~camera_mount_source must be 'tf' or 'param'");
    if (mount_source_ == "param") {
      std::vector<double> m;
      if (!pnh_.getParam("camera_mount", m) || m.size() != 7) {
        fatal("~camera_mount_source = param needs ~camera_mount = [x, y, z, qx, qy, qz, qw] (optical camera -> body)");
      }
      mount_.tx = m[0]; mount_.ty = m[1]; mount_.tz = m[2];
      mount_.qx = m[3]; mount_.qy = m[4]; mount_.qz = m[5]; mount_.qw = m[6];
      if (!camera_frame_is_optical_) mount_ = mount_.compose(rosCameraToOptical());
      mount_ready_ = true;
    }

    std::string t_left, t_right, t_disp, t_det, t_phase, t_target, t_pose;
    pnh_.param<std::string>("left_camera_info_topic", t_left, "/cam/bottom/left/half/camera_info");
    pnh_.param<std::string>("right_camera_info_topic", t_right, "/cam/bottom/right/half/camera_info");
    pnh_.param<std::string>("disparity_topic", t_disp, "/disparity_image_down_float");
    pnh_.param<std::string>("detections_topic", t_det, "/object_detection/detections");
    pnh_.param<std::string>("mission_state_topic", t_phase, "/mission_state");
    pnh_.param<std::string>("hop_target_topic", t_target, "/spray_target");
    pnh_.param<std::string>("pose_topic", t_pose, "/treemapper/drone_pose");

    pub_map_ = pnh_.advertise<treemapper_msgs::TreeMap>("map", 2, true);
    pub_markers_ = pnh_.advertise<visualization_msgs::MarkerArray>("markers", 2, true);
    pub_health_ = pnh_.advertise<treemapper_msgs::Health>("health", 5);
    pub_counters_ = pnh_.advertise<treemapper_msgs::DropCounters>("counters", 10);
    pub_timing_ = pnh_.advertise<treemapper_msgs::StageTiming>("stage_timing", 10);
    srv_get_ = pnh_.advertiseService("get_trees", &TreemapperNode::onGetTrees, this);
    srv_spray_ = pnh_.advertiseService("set_tree_sprayed", &TreemapperNode::onSetSprayed, this);

    sub_left_ = nh_.subscribe(t_left, 2, &TreemapperNode::onLeftInfo, this);
    sub_right_ = nh_.subscribe(t_right, 2, &TreemapperNode::onRightInfo, this);
    sub_disp_ = nh_.subscribe(t_disp, 20, &TreemapperNode::onDisparity, this, ros::TransportHints().tcpNoDelay());
    sub_det_ = nh_.subscribe<topic_tools::ShapeShifter>(t_det, 50, &TreemapperNode::onDetections, this,
                                                        ros::TransportHints().tcpNoDelay());
    sub_phase_ = nh_.subscribe(t_phase, 10, &TreemapperNode::onPhase, this);
    sub_target_ = nh_.subscribe(t_target, 10, &TreemapperNode::onTarget, this);
    if (pose_source_ == "pose_topic") {
      // own queue + thread: the main thread may wait for a pose (pose_wait) while these keep arriving
      pose_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.0));
      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
          t_pose, 100, [this](const geometry_msgs::PoseStamped::ConstPtr& m) { onPose(m); }, ros::VoidPtr(),
          &pose_queue_);
      sub_pose_ = nh_.subscribe(so);
      pose_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &pose_queue_);
      pose_spinner_->start();
    }

    timer_frames_ = nh_.createTimer(ros::Duration(0.05), &TreemapperNode::onFrameTimer, this);
    timer_map_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, map_rate_)), &TreemapperNode::onMapTimer, this);
    // wall clock: the heartbeat must not stop when a simulated /clock pauses or ends
    timer_health_ = nh_.createWallTimer(ros::WallDuration(1.0 / std::max(0.1, health_rate_)), &TreemapperNode::onHealthTimer, this);
    timer_counters_ = nh_.createTimer(ros::Duration(1.0), &TreemapperNode::onCountersTimer, this);
    ROS_INFO("treemapper_node: mapper '%s' (config %s, %zu keys), world '%s', body '%s', pose from %s, mount from %s",
             mapper_name_.c_str(), config_file_.c_str(), mapper_config_.size(), world_frame_.c_str(),
             body_frame_.c_str(), pose_source_.c_str(), mount_source_.c_str());
    ROS_INFO("treemapper_node: disparity %s, detections %s, phase %s, hop target %s", t_disp.c_str(), t_det.c_str(),
             t_phase.c_str(), t_target.c_str());
  }

  void shutdown() {
    guarded("shutdown", [&] {
      if (mapper_) {
        for (auto& f : sync_.flush()) processFrame(f);
        publishMap(nowSec());
      }
    });
  }

 private:
  [[noreturn]] void fatal(const std::string& m) {
    ROS_FATAL("treemapper_node: %s", m.c_str());
    throw FatalConfigError(m);  // the one exception CallbackGuard lets through: bad configuration ends the node
  }

  // ------------------------------------------------------------------ crash containment (B1)
  // Every ROS callback, timer and service body runs inside guard_: an exception is logged, counted
  // (callback_exceptions*, stage `input`) and reported as health FAULT for fault_hold_s, and the node keeps
  // running with its map and its IDs. A crash would lose the map and, with respawn, restart IDs at 0.
  template <typename F>
  bool guarded(const char* where, F&& body) {
    return guard_.run(where, ros::WallTime::now().toSec(), std::forward<F>(body),
                      [](const std::string& m) { ROS_ERROR_THROTTLE(1.0, "%s", m.c_str()); });
  }
  void onLeftInfo(const sensor_msgs::CameraInfo::ConstPtr& m) { guarded("left_camera_info", [&] { onLeftInfoBody(m); }); }
  void onRightInfo(const sensor_msgs::CameraInfo::ConstPtr& m) { guarded("right_camera_info", [&] { onRightInfoBody(m); }); }
  void onDisparity(const sensor_msgs::Image::ConstPtr& m) { guarded("disparity", [&] { onDisparityBody(m); }); }
  void onDetections(const topic_tools::ShapeShifter::ConstPtr& m) { guarded("detections", [&] { onDetectionsBody(m); }); }
  void onPhase(const std_msgs::String::ConstPtr& m) { guarded("mission_state", [&] { onPhaseBody(m); }); }
  void onTarget(const std_msgs::Int32::ConstPtr& m) { guarded("hop_target", [&] { onTargetBody(m); }); }
  void onPose(const geometry_msgs::PoseStamped::ConstPtr& m) { guarded("pose", [&] { onPoseBody(m); }); }
  void onFrameTimer(const ros::TimerEvent& e) { guarded("frame_timer", [&] { onFrameTimerBody(e); }); }
  void onMapTimer(const ros::TimerEvent& e) { guarded("map_timer", [&] { onMapTimerBody(e); }); }
  void onHealthTimer(const ros::WallTimerEvent& e) { guarded("health_timer", [&] { onHealthTimerBody(e); }); }
  void onCountersTimer(const ros::TimerEvent& e) { guarded("counters_timer", [&] { onCountersTimerBody(e); }); }
  bool onGetTrees(treemapper_msgs::GetTrees::Request& req, treemapper_msgs::GetTrees::Response& res) {
    if (!guarded("get_trees", [&] { onGetTreesBody(req, res); })) {
      res = treemapper_msgs::GetTrees::Response();
      res.found = false;
      res.message = "internal error: " + guard_.lastError();
    }
    return true;
  }
  bool onSetSprayed(treemapper_msgs::SetTreeSprayed::Request& req, treemapper_msgs::SetTreeSprayed::Response& res) {
    if (!guarded("set_tree_sprayed", [&] { onSetSprayedBody(req, res); })) {
      res.success = false;
      res.message = "internal error: " + guard_.lastError();
    }
    return true;
  }

  // ------------------------------------------------------------------ calibration
  void onLeftInfoBody(const sensor_msgs::CameraInfo::ConstPtr& m) {
    if (mapper_) return;
    left_ = *m;
    have_left_ = true;
    tryCreateMapper();
  }
  void onRightInfoBody(const sensor_msgs::CameraInfo::ConstPtr& m) {
    if (mapper_) return;
    right_ = *m;
    have_right_ = true;
    tryCreateMapper();
  }
  void tryCreateMapper() {
    if (mapper_ || !have_left_ || !have_right_) return;
    if (!mount_ready_) {  // camera_mount_source = tf: body <- camera (static), looked up once
      const std::string cam = camera_frame_.empty() ? left_.header.frame_id : camera_frame_;
      try {
        auto tr = tf_buffer_.lookupTransform(body_frame_, cam, ros::Time(0));
        mount_ = fromMsg(tr.transform);
        if (!camera_frame_is_optical_) mount_ = mount_.compose(rosCameraToOptical());
        mount_ready_ = true;
        ROS_INFO("camera mount %s <- %s: t = (%.3f, %.3f, %.3f), q = (%.4f, %.4f, %.4f, %.4f)%s", body_frame_.c_str(),
                 cam.c_str(), mount_.tx, mount_.ty, mount_.tz, mount_.qx, mount_.qy, mount_.qz, mount_.qw,
                 camera_frame_is_optical_ ? "" : " (composed with ROS camera -> optical)");
      } catch (const tf2::TransformException& e) {
        warn("mount", "waiting for the camera mount TF " + body_frame_ + " <- " + cam + ": " + e.what());
        return;
      }
    }
    double lp[12], rp[12];
    for (int k = 0; k < 12; ++k) { lp[k] = left_.P[k]; rp[k] = right_.P[k]; }
    std::string err, form;
    treemapper::StereoCalibration cal;
    if (!makeCalibration(static_cast<int>(left_.width), static_cast<int>(left_.height), lp, rp, stride_, mount_, &cal,
                         &err, &form)) {
      calibration_error_ = err;
      ++node_counters_["camera_info_rejected"];
      ROS_ERROR_THROTTLE(5.0, "calibration rejected (fail closed, no mapping): %s", err.c_str());
      have_left_ = have_right_ = false;  // retry with the next pair
      return;
    }
    if (disp_offset_mode_ == "auto") {
      disp_offset_px_ = left_.P[2] - right_.P[2];
    } else if (disp_offset_mode_ == "none") {
      disp_offset_px_ = 0.0;
    } else {
      try { disp_offset_px_ = std::stod(disp_offset_mode_); } catch (const std::exception&) {
        fatal("~disparity_offset must be 'none', 'auto' or a number (px), got '" + disp_offset_mode_ + "'");
      }
    }
    ROS_INFO("disparity offset: %s -> %.4f px subtracted (cx_left - cx_right = %.4f px)", disp_offset_mode_.c_str(),
             disp_offset_px_, left_.P[2] - right_.P[2]);
    calibration_error_.clear();
    try {
      mapper_ = treemapper::createMapper(mapper_name_, mapper_config_, cal);
    } catch (const std::exception& e) {
      fatal(std::string("cannot create mapper: ") + e.what());
    }
    mapper_->setLogger([](const std::string& s) { ROS_INFO("[mapper] %s", s.c_str()); });
    cal_ = cal;
    ROS_INFO("calibration latched: %dx%d fx %.2f fy %.2f cx %.2f cy %.2f baseline %.4f m (P[3] form %s), stride %d",
             cal.width, cal.height, cal.fx, cal.fy, cal.cx, cal.cy, cal.baseline, form.c_str(), cal.stride);
    sub_left_.shutdown();
    sub_right_.shutdown();
  }

  // ------------------------------------------------------------------ inputs
  void onDisparityBody(const sensor_msgs::Image::ConstPtr& m) {
    const double t = nowSec();
    last_input_t_ = t;
    ++node_counters_["disparity_msgs"];
    if (!mapper_) { ++node_counters_["disparity_dropped_no_calibration"]; return; }
    const auto t0 = std::chrono::steady_clock::now();
    DisparityGrid g;
    const DisparityStatus st = sampleDisparity(m->encoding, m->width, m->height, m->step, m->is_bigendian != 0,
                                               m->data.data(), m->data.size(), stride_, cal_.width, cal_.height,
                                               disp_offset_px_, quantum_px_, &g);
    sample_ms_ += wallMs(t0);
    ++sample_n_;
    if (st == DisparityStatus::kMalformed) { ++node_counters_["disparity_rejected_malformed"]; return; }
    if (st == DisparityStatus::kResolutionMismatch) {
      ++node_counters_["disparity_rejected_resolution"];
      warn("resolution", "disparity " + std::to_string(m->width) + "x" + std::to_string(m->height) +
                             " does not match the calibration " + std::to_string(cal_.width) + "x" +
                             std::to_string(cal_.height) + ": frame dropped");
      return;
    }
    const StampNs s = static_cast<StampNs>(m->header.stamp.toNSec());
    for (auto& f : sync_.addDisparity(t, s, std::move(g))) processFrame(f);
  }

  void onDetectionsBody(const topic_tools::ShapeShifter::ConstPtr& m) {
    const double t = nowSec();
    last_input_t_ = t;
    std::vector<treemapper::Detection2D> boxes;
    StampNs stamp = 0;
    if (m->getDataType() == "treemapper_msgs/Detection2DArray") {
      auto d = m->instantiate<treemapper_msgs::Detection2DArray>();
      boxes = fromMsg(*d, detection_scale_);
      stamp = static_cast<StampNs>(d->header.stamp.toNSec());
    } else if (m->getDataType() == kSpotDetectionResultType && m->getMD5Sum() == kSpotDetectionResultMd5) {
      std::vector<std::uint8_t> buf(m->size());
      ros::serialization::OStream os(buf.data(), static_cast<std::uint32_t>(buf.size()));
      m->write(os);
      std::string err;
      if (!parseSpotDetectionResult(buf.data(), buf.size(), &stamp, &boxes, &err)) {
        ++node_counters_["detections_rejected_malformed"];
        warn("det_malformed", "malformed " + std::string(kSpotDetectionResultType) + ": " + err);
        return;
      }
      scaleBoxes(&boxes, detection_scale_);
    } else {
      ++node_counters_["detections_rejected_type"];
      warn("det_type", "unsupported detection type '" + m->getDataType() + "' (md5 " + m->getMD5Sum() +
                           "); expected treemapper_msgs/Detection2DArray or " + kSpotDetectionResultType);
      return;
    }
    if (!mapper_) { ++node_counters_["frames_dropped_no_calibration"]; return; }
    for (auto& f : sync_.addDetections(t, stamp, std::move(boxes), phase_raw_, hop_target_)) processFrame(f);
  }

  void onPhaseBody(const std_msgs::String::ConstPtr& m) {
    have_phase_msg_ = true;
    if (m->data == phase_raw_) return;
    const MissionPhase p = treemapper::parseMissionPhase(m->data);
    if (p == MissionPhase::kUnknown && !unknown_phases_.count(m->data)) {
      unknown_phases_.insert(m->data);
      ROS_WARN("unknown mission phase '%s': behaving as SCAN (permissive, fail open)", m->data.c_str());
    }
    ROS_INFO("mission phase %s -> %s", phase_raw_.empty() ? "(none)" : phase_raw_.c_str(), m->data.c_str());
    // phase-boundary snapshot: the map as it stood after the last frame of the previous phase
    if (mapper_ && last_frame_t_ > last_snap_t_) publishMap(last_frame_t_);
    phase_raw_ = m->data;
    phase_ = p;
    // heartbeat frame carrying the new phase; detections still waiting for their disparity arrived before the
    // change, so they go first (the offline reader orders frames by receive time the same way)
    if (mapper_) {
      if (sync_.pending() == 0) feedHeartbeat(nowSec());
      else phase_heartbeat_due_ = true;
    }
  }

  void onTargetBody(const std_msgs::Int32::ConstPtr& m) {
    if (m->data != hop_target_) ROS_INFO("hop target %d -> %d", hop_target_, m->data);
    hop_target_ = m->data < 0 ? treemapper::kNoTree : m->data;
  }

  void onPoseBody(const geometry_msgs::PoseStamped::ConstPtr& m) {
    geometry_msgs::TransformStamped ts;
    ts.header = m->header;
    if (ts.header.frame_id.empty()) ts.header.frame_id = world_frame_;
    ts.child_frame_id = body_frame_;
    ts.transform.translation.x = m->pose.position.x;
    ts.transform.translation.y = m->pose.position.y;
    ts.transform.translation.z = m->pose.position.z;
    ts.transform.rotation = m->pose.orientation;
    pose_buffer_->setTransform(ts, "pose_topic");
  }

  // ------------------------------------------------------------------ processing
  /// Pose at `stamp` (or the latest one if `latest`); pose.valid = false when unavailable (holds the mapper).
  treemapper::DronePose lookupPose(double stamp, bool latest) {
    tf2_ros::Buffer& buf = pose_buffer_ ? *pose_buffer_ : tf_buffer_;
    try {
      geometry_msgs::TransformStamped tr;
      if (latest) {
        tr = buf.lookupTransform(world_frame_, body_frame_, ros::Time(0));
        if (std::fabs(stamp - tr.header.stamp.toSec()) > pose_max_age_) throw tf2::TransformException("latest pose too old");
      } else {
        // wait for a late pose, but not on every frame once poses keep failing (3 in a row)
        const double wait = pose_failures_ < 3 ? pose_wait_ : 0.0;
        tr = buf.lookupTransform(world_frame_, body_frame_, ros::Time(stamp), ros::Duration(wait));
      }
      if (!pose_ok_) ROS_INFO("pose available (%s <- %s)", world_frame_.c_str(), body_frame_.c_str());
      pose_ok_ = true;
      pose_failures_ = 0;
      pose_error_.clear();
      return toDronePose(tr.transform, stamp);
    } catch (const tf2::TransformException& e) {
      if (pose_ok_) ROS_WARN("pose lost: %s (mapper holds, fail closed)", e.what());
      pose_ok_ = false;
      ++pose_failures_;
      pose_error_ = e.what();
      ++node_counters_["pose_unavailable"];
      ROS_WARN_THROTTLE(5.0, "no pose %s <- %s at %.3f: %s (frame held)", world_frame_.c_str(), body_frame_.c_str(),
                        stamp, e.what());
      treemapper::DronePose p;
      p.t = stamp;
      p.valid = false;
      return p;
    }
  }

  void processFrame(SyncedFrame& f) {
    const auto t0 = std::chrono::steady_clock::now();
    treemapper::FrameInput in;
    in.t = std::max(f.t, last_frame_t_);  // the mapper's rate limits integrate over t: never step it backwards
    in.stamp = toSec(f.stamp);
    stamp_lag_ = f.t - in.stamp;
    in.pose = lookupPose(in.stamp, false);
    in.phase = treemapper::parseMissionPhase(f.phase);
    in.hop_target = f.hop_target;
    in.boxes = std::move(f.boxes);
    if (f.has_disparity) {
      in.grid_w = f.disparity.grid_w;
      in.grid_h = f.disparity.grid_h;
      in.disparity = std::move(f.disparity.data);
    }
    mapper_->process(in);
    last_frame_t_ = std::max(last_frame_t_, f.t);
    ++frames_processed_;
    frame_ms_ += wallMs(t0);
    ++frame_n_;
  }

  void feedHeartbeat(double t) {
    treemapper::FrameInput in;
    in.t = std::max(t, last_frame_t_);
    // header-stamp clock = receive clock - the latency seen on the last detection frame (the two clocks differ in a
    // replay of separately recorded bags; on the drone they are one clock and this is the detector latency)
    in.stamp = t - stamp_lag_;
    in.pose = lookupPose(in.stamp, true);
    in.phase = phase_;
    in.hop_target = hop_target_;
    mapper_->process(in);
    last_frame_t_ = std::max(last_frame_t_, t);
    ++node_counters_["heartbeat_frames"];
  }

  void onFrameTimerBody(const ros::TimerEvent&) {
    if (!mapper_) { tryCreateMapper(); return; }
    const double t = nowSec();
    for (auto& f : sync_.poll(t)) processFrame(f);
    if (sync_.pending() == 0 && (phase_heartbeat_due_ || t - last_frame_t_ >= heartbeat_frame_period_)) {
      phase_heartbeat_due_ = false;
      feedHeartbeat(t);
    }
  }

  // ------------------------------------------------------------------ outputs
  void publishMap(double stamp) {
    if (!mapper_) return;
    if (stamp <= last_snap_t_) stamp = last_snap_t_ + 1e-6;  // strictly increasing snapshot stamps
    last_snap_t_ = stamp;
    std_msgs::Header h;
    h.stamp = ros::Time(stamp);
    h.frame_id = world_frame_;
    const auto trees = mapper_->publishedTrees();
    tracker_.update(stamp, phase_, trees);
    pub_map_.publish(toMapMsg(h, trees, &tracker_));
    pub_markers_.publish(toMarkers(h, trees, &tracker_, hop_target_));
    n_trees_ = trees.size();
    // per-stage timing since the previous snapshot (mapper stages + the node's own)
    auto ms = mapper_->takeStageTiming();
    if (frame_n_) ms["node_frame_total"] = frame_ms_ / static_cast<double>(frame_n_);
    if (sample_n_) ms["node_disparity_sample"] = sample_ms_ / static_cast<double>(sample_n_);
    frame_ms_ = sample_ms_ = 0.0;
    frame_n_ = sample_n_ = 0;
    pub_timing_.publish(toTimingMsg(h, ms));
  }
  void onMapTimerBody(const ros::TimerEvent&) { publishMap(nowSec()); }

  void onHealthTimerBody(const ros::WallTimerEvent&) {
    const double t = nowSec();
    treemapper_msgs::Health h;
    h.header.stamp = ros::Time::now();
    h.header.frame_id = world_frame_;
    h.phase.header = h.header;
    h.phase.phase = static_cast<std::uint8_t>(phase_);
    h.phase.raw = phase_raw_;
    h.phase.target_id = hop_target_;
    h.trees_published = static_cast<std::uint32_t>(n_trees_);
    h.last_input_age_s = last_input_t_ > 0.0 ? t - last_input_t_ : -1.0;
    if (!have_phase_msg_) h.warnings.push_back("no mission_state received: permissive (scan) behaviour");
    else if (phase_ == MissionPhase::kUnknown) h.warnings.push_back("unknown mission phase '" + phase_raw_ + "': permissive");
    if (last_input_t_ > 0.0 && t - last_input_t_ > input_stale_warn_ && !isSleepPhase(phase_)) {
      h.warnings.push_back("no camera/detection input for " + std::to_string(t - last_input_t_) + " s");
    }
    if (guard_.total() > 0) h.warnings.push_back(guard_.lastError());
    if (guard_.faultActive(ros::WallTime::now().toSec())) {
      h.state = treemapper_msgs::Health::FAULT;
      h.reason = guard_.lastError() + "; node kept alive";
    } else if (!calibration_error_.empty()) {
      h.state = treemapper_msgs::Health::FAULT;
      h.reason = "calibration rejected: " + calibration_error_;
    } else if (!mapper_) {
      h.state = treemapper_msgs::Health::HOLDING;
      h.reason = std::string("waiting for calibration (") + (have_left_ ? "" : "left camera_info ") +
                 (have_right_ ? "" : "right camera_info ") + (mount_ready_ ? "" : "camera mount TF") + ")";
    } else if (isSleepPhase(phase_)) {
      h.state = treemapper_msgs::Health::SLEEPING;
      h.reason = "phase " + phase_raw_ + ": sleeping, not dead";
    } else if (!pose_ok_ && frames_processed_ > 0) {
      h.state = treemapper_msgs::Health::HOLDING;
      h.reason = "pose not available: " + pose_error_;
    } else {
      h.state = treemapper_msgs::Health::AWAKE;
      h.reason = "mapping (" + std::string(phase_raw_.empty() ? "no phase" : phase_raw_) + ")";
    }
    pub_health_.publish(h);
  }

  void onCountersTimerBody(const ros::TimerEvent&) {
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = world_frame_;
    auto input = node_counters_;
    for (const auto& kv : sync_.counters()) input["sync_" + kv.first] = kv.second;
    for (const auto& kv : guard_.counters()) input[kv.first] = kv.second;
    pub_counters_.publish(toCountersMsg(h, "input", input));
    if (mapper_) pub_counters_.publish(toCountersMsg(h, "mapper", mapper_->counters()));
  }

  bool onGetTreesBody(treemapper_msgs::GetTrees::Request& req, treemapper_msgs::GetTrees::Response& res) {
    if (!mapper_) {
      res.found = false;
      res.message = "mapper not running yet (waiting for calibration)";
      return true;
    }
    for (const auto& e : mapper_->publishedTrees()) {
      if (req.id < 0 || e.id == req.id) res.trees.push_back(toMsg(e, tracker_.find(e.id)));
    }
    res.found = req.id < 0 || !res.trees.empty();
    res.message = req.id < 0 ? std::to_string(res.trees.size()) + " trees"
                             : (res.found ? "ok" : "unknown id " + std::to_string(req.id));
    return true;
  }

  bool onSetSprayedBody(treemapper_msgs::SetTreeSprayed::Request& req, treemapper_msgs::SetTreeSprayed::Response& res) {
    if (!mapper_) {
      res.success = false;
      res.message = "mapper not running yet";
      return true;
    }
    if (!req.sprayed) {
      res.success = false;
      res.message = "clearing the sprayed flag is not supported: sprayed trees are protected (rule 5)";
      ROS_WARN("set_tree_sprayed(%d, false) refused: %s", req.id, res.message.c_str());
      return true;
    }
    res.success = mapper_->markSprayed(req.id);
    res.message = res.success ? "tree " + std::to_string(req.id) + " marked sprayed" : "unknown id " + std::to_string(req.id);
    ROS_INFO("set_tree_sprayed(%d): %s", req.id, res.message.c_str());
    if (res.success) publishMap(nowSec());
    return true;
  }

  void warn(const std::string& key, const std::string& m) {
    auto& last = warn_last_[key];
    const double t = ros::WallTime::now().toSec();
    if (t - last < 5.0) return;
    last = t;
    ROS_WARN("%s", m.c_str());
  }

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> pose_buffer_;
  ros::CallbackQueue pose_queue_;
  std::unique_ptr<ros::AsyncSpinner> pose_spinner_;

  std::string mapper_name_, config_file_;
  treemapper::MapperConfig mapper_config_;
  std::unique_ptr<treemapper::Mapper> mapper_;
  treemapper::StereoCalibration cal_;
  std::string world_frame_, body_frame_, pose_source_, mount_source_, camera_frame_;
  bool camera_frame_is_optical_{true};
  treemapper::Transform mount_;
  bool mount_ready_{false};
  int stride_{4};
  std::string disp_offset_mode_{"none"};
  double disp_offset_px_{0.0};
  double quantum_px_{0.0}, detection_scale_{1.0}, pose_wait_{0.1}, pose_max_age_{0.5};
  double heartbeat_frame_period_{0.2}, health_rate_{2.0}, map_rate_{5.0}, input_stale_warn_{2.0};

  sensor_msgs::CameraInfo left_, right_;
  bool have_left_{false}, have_right_{false};
  std::string calibration_error_;

  FrameSync sync_;
  std::string phase_raw_;
  MissionPhase phase_{MissionPhase::kUnknown};
  bool have_phase_msg_{false};
  bool phase_heartbeat_due_{false};
  std::set<std::string> unknown_phases_;
  TreeId hop_target_{treemapper::kNoTree};
  bool pose_ok_{false};
  int pose_failures_{0};
  std::string pose_error_;

  double last_frame_t_{0.0}, last_snap_t_{0.0}, last_input_t_{0.0};
  double stamp_lag_{0.0};  ///< receive time - header stamp of the last detection frame
  std::uint64_t frames_processed_{0};
  std::size_t n_trees_{0};
  double frame_ms_{0.0}, sample_ms_{0.0};
  std::uint64_t frame_n_{0}, sample_n_{0};
  TreeMetaTracker tracker_;
  CallbackGuard guard_{10.0};
  std::map<std::string, std::uint64_t> node_counters_;
  std::map<std::string, double> warn_last_;

  ros::Publisher pub_map_, pub_markers_, pub_health_, pub_counters_, pub_timing_;
  ros::ServiceServer srv_get_, srv_spray_;
  ros::Subscriber sub_left_, sub_right_, sub_disp_, sub_det_, sub_phase_, sub_target_, sub_pose_;
  ros::Timer timer_frames_, timer_map_, timer_counters_;
  ros::WallTimer timer_health_;
};

}  // namespace treemapper_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "treemapper");
  ros::NodeHandle nh, pnh("~");
  try {
    treemapper_ros::TreemapperNode node(nh, pnh);
    ros::spin();
    node.shutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("treemapper_node exiting: %s", e.what());
    return 1;
  }
  return 0;
}
