// corridor_nav_node: thin ROS 1 wrapper around corridor_nav::Navigator (nav/corridor_nav, ROS-free).
//
// Inputs:  /treemapper/map (TreeMap), TF world <- body (the pose), the stereo disparity (32FC1) and both half
//          camera_infos (field of view, nadir clearance), services ~start / ~abort.
// Outputs: /corridor_nav/setpoint (PoseStamped, the carrot point, yaw only), /corridor_nav/spray (Bool, every tick,
//          false once on shutdown), /mission_state (String) and /spray_target (Int32) for TreeMapper, ~status
//          (NavStatus, 2 Hz), ~plan_markers, ~timing. TreeMapper is told about sprayed trees through
//          /treemapper/set_tree_sprayed, called from a std::async worker so the control tick never blocks on it.
//
// Clocks: Inputs.t = ros::Time::now() (sim time under use_sim_time); the map's stamp is TreeMapper's header stamp
// (it republishes at 5 Hz stamped with ROS now, also while asleep); the pose time is the stamp of the TF transform.
//
// Camera mount (wide-axis alignment): ~camera_mount = [x, y, z, qx, qy, qz, qw] (optical camera -> body, the format of
// treemapper_ros' camera_mount) gives the yaw of the image x-axis on the body; without it ~camera_mount_source = tf
// looks up body_frame <- left camera_info frame_id once (waits up to ~camera_mount_wait s); if neither works the yaw
// is unknown (NaN, warned once) and the navigator uses the old fov_min rule with yaw = heading. The camera frame must
// be an optical frame (x right, y down, z forward), as treemapper_ros' default.
//
// Until camera_info (and, without `home` in the field file, a pose) has arrived there is no Navigator: the node
// publishes IDLE, nozzle closed and a hover setpoint at the current pose if there is one.
//
// All callbacks (timer, subscribers, services) run on the one spinner thread; only the set_tree_sprayed worker
// runs elsewhere and it touches nothing but its own arguments and its return value. A callback exception is
// counted (node_callback_exceptions_*), logged, and leaves the node running with the nozzle closed.
#include <ros/package.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <treemapper_msgs/SetTreeSprayed.h>
#include <treemapper_msgs/StageTiming.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <csignal>
#include <cmath>
#include <deque>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "corridor_nav/config.hpp"
#include "corridor_nav/guard.hpp"
#include "corridor_nav/navigator.hpp"
#include "corridor_nav_ros/conversions.hpp"
#include "corridor_nav_ros/field_file.hpp"

namespace corridor_nav_ros {

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

double wallMs(std::chrono::steady_clock::time_point a) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
}

double yawOf(const treemapper::DronePose& p) {
  return std::atan2(2.0 * (p.qw * p.qz + p.qx * p.qy), 1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz));
}

geometry_msgs::Point point(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
}  // namespace

class CorridorNavNode {
 public:
  CorridorNavNode(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
    std::string config_file, field_file, left_topic, right_topic, disparity_topic, map_topic;
    double rate = 20.0;
    pnh_.param<std::string>("config_file", config_file, "config/core/corridor_nav.cfg");
    if (!pnh_.getParam("field_file", field_file) || field_file.empty()) {
      throw std::invalid_argument("parameter ~field_file is required (YAML with polygon, optional heading_rad, home)");
    }
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("body_frame", body_frame_, "body");
    pnh_.param<std::string>("map_topic", map_topic, "/treemapper/map");
    pnh_.param<std::string>("set_sprayed_service", set_sprayed_service_, "/treemapper/set_tree_sprayed");
    pnh_.param<std::string>("disparity_topic", disparity_topic, "/disparity_image_down_float");
    pnh_.param<std::string>("left_info_topic", left_topic, "/cam/bottom/left/half/camera_info");
    pnh_.param<std::string>("right_info_topic", right_topic, "/cam/bottom/right/half/camera_info");
    pnh_.param<double>("rate", rate, 20.0);
    if (!(rate > 0.0 && rate <= 200.0)) throw std::invalid_argument("~rate must be in (0, 200] Hz");
    pnh_.param<std::string>("camera_mount_source", mount_source_, "tf");
    pnh_.param<double>("camera_mount_wait", mount_wait_s_, 5.0);
    if (mount_source_ != "tf" && mount_source_ != "none") {
      throw std::invalid_argument("~camera_mount_source must be 'tf' or 'none'");
    }
    if (!(mount_wait_s_ >= 0.0)) throw std::invalid_argument("~camera_mount_wait must be >= 0");
    if (pnh_.hasParam("camera_mount")) {
      std::vector<double> m;
      if (!pnh_.getParam("camera_mount", m) || m.size() != 7) {
        throw std::invalid_argument("~camera_mount must be [x, y, z, qx, qy, qz, qw] (optical camera -> body)");
      }
      geometry_msgs::Quaternion q;
      q.x = m[3];
      q.y = m[4];
      q.z = m[5];
      q.w = m[6];
      resolveMountYaw(imageXAxisBodyYaw(q), "~camera_mount");
    } else if (mount_source_ == "none") {
      resolveMountYaw(std::numeric_limits<double>::quiet_NaN(), "~camera_mount_source none");
    }

    // Fail at startup, not at the first camera_info: a bad cfg key or field file ends the node.
    if (config_file.empty() || config_file[0] != '/') {
      config_file = ros::package::getPath("corridor_nav_ros") + "/" + config_file;
    }
    cfg_ = corridor_nav::configFrom(readCfgFile(config_file))
               .sanitized([](const std::string& m) { ROS_WARN("config: %s", m.c_str()); });
    field_ = loadFieldFile(field_file, &field_has_home_);
    ROS_INFO("corridor_nav: config %s, field %s (%zu vertices%s)", config_file.c_str(), field_file.c_str(),
             field_.polygon.size(), field_has_home_ ? "" : ", no home: the first pose becomes home");

    pub_state_ = nh_.advertise<std_msgs::String>("/mission_state", 1, true);
    pub_target_ = nh_.advertise<std_msgs::Int32>("/spray_target", 1, true);
    pub_setpoint_ = nh_.advertise<geometry_msgs::PoseStamped>("/corridor_nav/setpoint", 1);
    pub_spray_ = nh_.advertise<std_msgs::Bool>("/corridor_nav/spray", 1);
    pub_status_ = pnh_.advertise<corridor_nav_msgs::NavStatus>("status", 1);
    pub_markers_ = pnh_.advertise<visualization_msgs::MarkerArray>("plan_markers", 1);
    pub_timing_ = pnh_.advertise<treemapper_msgs::StageTiming>("timing", 1);
    srv_start_ = pnh_.advertiseService("start", &CorridorNavNode::onStart, this);
    srv_abort_ = pnh_.advertiseService("abort", &CorridorNavNode::onAbort, this);

    sub_map_ = nh_.subscribe(map_topic, 2, &CorridorNavNode::onMap, this);
    sub_disparity_ = nh_.subscribe(disparity_topic, 1, &CorridorNavNode::onDisparity, this);
    sub_left_ = nh_.subscribe(left_topic, 1, &CorridorNavNode::onLeftInfo, this);
    sub_right_ = nh_.subscribe(right_topic, 1, &CorridorNavNode::onRightInfo, this);

    timer_tick_ = nh_.createTimer(ros::Duration(1.0 / rate), &CorridorNavNode::onTick, this);
    timer_status_ = nh_.createTimer(ros::Duration(0.5), &CorridorNavNode::onStatusTimer, this);
  }

  /// Nozzle closed, once, before the process ends. The calls still pending are abandoned, not waited for.
  void shutdown() {
    publishSpray(false);
    for (auto& f : spray_calls_) {
      if (f.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        // A std::async future joins in its destructor; a hung service must not hold the node up at exit.
        new std::future<std::optional<corridor_nav::SprayResult>>(std::move(f));
      }
    }
    spray_calls_.clear();
  }

 private:
  using SprayFuture = std::future<std::optional<corridor_nav::SprayResult>>;
  static constexpr std::size_t kMaxPendingCalls = 8;

  // --- subscribers ------------------------------------------------------------------------------------------
  void onMap(const treemapper_msgs::TreeMap::ConstPtr& m) {
    guarded("map", [&] {
      corridor_nav::MapSnapshot s;
      s.stamp = m->header.stamp.toSec();
      s.trees = fromMsg(*m);
      pending_map_ = std::move(s);   // only the newest map matters
    });
  }

  void onDisparity(const sensor_msgs::Image::ConstPtr& m) {
    guarded("disparity", [&] {
      disparity_ = m;
      ++disparity_seq_;
    });
  }

  void onLeftInfo(const sensor_msgs::CameraInfo::ConstPtr& m) {
    guarded("left_info", [&] {
      left_info_ = m;
      updateCalibration();
    });
  }

  void onRightInfo(const sensor_msgs::CameraInfo::ConstPtr& m) {
    guarded("right_info", [&] {
      right_info_ = m;
      updateCalibration();
    });
  }

  void updateCalibration() {
    fov_.reset();
    stereo_.reset();
    if (!left_info_) return;
    fov_ = fovFrom(*left_info_);
    if (fov_) fov_->x_axis_body_yaw_rad = mount_yaw_;
    if (right_info_) stereo_ = stereoFrom(*left_info_, *right_info_);
    ++calibration_seq_;
  }

  // --- services ---------------------------------------------------------------------------------------------
  bool onStart(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (!nav_) {
      res.success = false;
      res.message = "navigator not ready: " + waitingFor();
    } else if (nav_->state() != corridor_nav::NavState::kIdle) {
      res.success = false;
      res.message = std::string("not IDLE (state ") + corridor_nav::toString(nav_->state()) + ")";
    } else {
      start_requested_ = true;
      res.success = true;
      res.message = "start requested";
    }
    ROS_INFO("~start: %s (%s)", res.success ? "accepted" : "refused", res.message.c_str());
    return true;
  }

  bool onAbort(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    abort_requested_ = true;
    res.success = true;
    res.message = nav_ ? "abort requested" : "navigator not started: nothing flying, nozzle closed";
    ROS_WARN("~abort: %s", res.message.c_str());
    return true;
  }

  // --- control tick -----------------------------------------------------------------------------------------
  void onTick(const ros::TimerEvent&) {
    ros::Time stamp;
    treemapper::DronePose pose;
    try {
      stamp = ros::Time::now();
      if (stamp.isZero()) return;   // sim time, no /clock yet
      pose = lookupPose();
      tick(stamp, pose);
    } catch (const std::exception& e) {
      onException("tick", e.what(), stamp);
      safeOutputs(stamp, pose);
    } catch (...) {
      onException("tick", "unknown exception", stamp);
      safeOutputs(stamp, pose);
    }
  }

  void tick(const ros::Time& stamp, const treemapper::DronePose& pose) {
    if (!nav_) tryCreateNavigator(pose);
    if (!nav_) {
      publishIdle(stamp, pose);
      return;
    }
    const double t = stamp.toSec();
    corridor_nav::Inputs in;
    in.t = t;
    in.pose = pose;
    in.map = std::move(pending_map_);
    pending_map_.reset();
    in.clearance = clearance(pose);
    in.start_requested = start_requested_;
    in.abort_requested = abort_requested_;
    start_requested_ = abort_requested_ = false;
    in.spray_result = nextSprayResult();

    const auto t0 = std::chrono::steady_clock::now();
    const corridor_nav::Outputs out = nav_->step(in);
    step_ms_ = wallMs(t0);

    pub_setpoint_.publish(toSetpoint(out, world_frame_, stamp));
    publishSpray(out.nozzle);
    publishPhase(out.phase, out.target, stamp);
    treemapper_msgs::StageTiming tm;
    tm.header.stamp = stamp;
    tm.stages = {"step"};
    tm.ms = {step_ms_};
    pub_timing_.publish(tm);
    if (out.request_sprayed) launchSprayCall(*out.request_sprayed);
  }

  treemapper::DronePose lookupPose() {
    try {
      const auto tf = tf_buffer_.lookupTransform(world_frame_, body_frame_, ros::Time(0));
      return toDronePose(tf);
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(5.0, "no pose %s <- %s: %s", world_frame_.c_str(), body_frame_.c_str(), e.what());
      return treemapper::DronePose();   // valid = false
    }
  }

  /// Sets the yaw of the image x-axis (NaN = unknown) and says what the navigator will do with it.
  void resolveMountYaw(double yaw, const std::string& source) {
    mount_yaw_ = yaw;
    mount_resolved_ = true;
    if (std::isfinite(yaw)) {
      ROS_INFO("camera mount from %s: image x-axis at body yaw %.1f deg -> corridors are flown with the wide axis "
               "across track, sized with hfov",
               source.c_str(), yaw * 180.0 / M_PI);
    } else {
      ROS_WARN("camera mount from %s: image x-axis yaw unknown (no mount, or the x-axis is near vertical): sizing "
               "corridors with min(hfov, vfov), yaw = corridor heading",
               source.c_str());
    }
    if (fov_) fov_->x_axis_body_yaw_rad = mount_yaw_;
  }

  /// camera_mount_source = tf: body <- left camera frame, looked up once (waiting up to ~camera_mount_wait s of ROS
  /// time from the first attempt). True when the mount is settled, known or not.
  bool mountSettled() {
    if (mount_resolved_) return true;
    const std::string cam = left_info_ ? left_info_->header.frame_id : std::string();
    if (cam.empty()) {
      resolveMountYaw(std::numeric_limits<double>::quiet_NaN(), "camera_info (empty frame_id)");
      return true;
    }
    const ros::Time now = ros::Time::now();
    if (mount_wait_start_.isZero()) mount_wait_start_ = now;
    try {
      const auto tr = tf_buffer_.lookupTransform(body_frame_, cam, ros::Time(0));
      resolveMountYaw(imageXAxisBodyYaw(tr.transform.rotation), "TF " + body_frame_ + " <- " + cam);
      return true;
    } catch (const tf2::TransformException& e) {
      if ((now - mount_wait_start_).toSec() < mount_wait_s_) {
        ROS_WARN_THROTTLE(2.0, "waiting for the camera mount TF %s <- %s: %s", body_frame_.c_str(), cam.c_str(),
                          e.what());
        return false;
      }
      resolveMountYaw(std::numeric_limits<double>::quiet_NaN(),
                      "TF " + body_frame_ + " <- " + cam + " (not available: " + e.what() + ")");
      return true;
    }
  }

  void tryCreateNavigator(const treemapper::DronePose& pose) {
    if (!fov_) return;
    if (!mountSettled()) return;
    fov_->x_axis_body_yaw_rad = mount_yaw_;
    corridor_nav::Field field = field_;
    if (!field_has_home_) {
      if (!pose.valid) return;
      field.home = {pose.x, pose.y};
    }
    nav_ = std::make_unique<corridor_nav::Navigator>(
        cfg_, field, *fov_, [](const std::string& m) { ROS_WARN("navigator: %s", m.c_str()); });
    const std::string mount =
        std::isfinite(mount_yaw_) ? std::to_string(mount_yaw_ * 180.0 / M_PI) + " deg" : std::string("unknown");
    ROS_INFO("corridor_nav: navigator ready (hfov %.1f deg, vfov %.1f deg, image x-axis body yaw %s, home %.2f %.2f); "
             "call ~start",
             fov_->hfov_rad * 180.0 / M_PI, fov_->vfov_rad * 180.0 / M_PI, mount.c_str(), field.home.x, field.home.y);
  }

  /// Nadir clearance of the latest disparity, recomputed when a new image (or calibration) arrived. Invalid (which
  /// the core treats as blind: no descent) while the disparity or the stereo calibration is missing.
  corridor_nav::Clearance clearance(const treemapper::DronePose& pose) {
    if (!disparity_ || !stereo_) {
      clearance_ = corridor_nav::Clearance();
      return clearance_;
    }
    if (disparity_seq_ != clearance_disparity_seq_ || calibration_seq_ != clearance_calibration_seq_) {
      clearance_disparity_seq_ = disparity_seq_;
      clearance_calibration_seq_ = calibration_seq_;
      corridor_nav::DisparityView view;
      if (!disparityView(*disparity_, &view)) {
        ROS_WARN_THROTTLE(10.0, "disparity image is not a packed little-endian 32FC1: clearance invalid");
        clearance_ = corridor_nav::Clearance();
      } else {
        double roll = 0.0, pitch = 0.0;
        rollPitch(pose, &roll, &pitch);
        clearance_ = corridor_nav::nadirClearance(view, *stereo_, roll, pitch, cfg_);
      }
    }
    return clearance_;
  }

  // --- set_tree_sprayed worker --------------------------------------------------------------------------------
  void launchSprayCall(treemapper::TreeId id) {
    pollSprayCalls();
    if (spray_calls_.size() >= kMaxPendingCalls) {
      ++node_counters_["node_spray_calls_dropped"];
      ROS_ERROR_THROTTLE(5.0, "%zu set_tree_sprayed calls still pending: not sending another", spray_calls_.size());
      return;
    }
    const std::string service = set_sprayed_service_;
    spray_calls_.push_back(std::async(std::launch::async, [service, id]() -> std::optional<corridor_nav::SprayResult> {
      treemapper_msgs::SetTreeSprayed srv;
      srv.request.id = id;
      srv.request.sprayed = true;
      try {
        if (!ros::service::call(service, srv)) return std::nullopt;   // no answer: the core retries after its timeout
      } catch (...) {
        return std::nullopt;
      }
      corridor_nav::SprayResult r;
      r.id = id;
      r.ok = srv.response.success;
      return r;
    }));
  }

  void pollSprayCalls() {
    for (auto it = spray_calls_.begin(); it != spray_calls_.end();) {
      if (it->wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        ++it;
        continue;
      }
      std::optional<corridor_nav::SprayResult> r;
      try {
        r = it->get();
      } catch (...) {
      }
      if (r) {
        spray_results_.push_back(*r);
      } else {
        ++node_counters_["node_spray_call_no_answer"];
      }
      it = spray_calls_.erase(it);
    }
  }

  std::optional<corridor_nav::SprayResult> nextSprayResult() {
    pollSprayCalls();
    if (spray_results_.empty()) return std::nullopt;
    auto r = spray_results_.front();
    spray_results_.pop_front();
    return r;
  }

  // --- publishing -------------------------------------------------------------------------------------------
  void publishSpray(bool open) {
    std_msgs::Bool b;
    b.data = open;
    pub_spray_.publish(b);
  }

  /// /mission_state and /spray_target (latched): on change, and once a second so a late subscriber that missed the
  /// latch of a restarted publisher still converges.
  void publishPhase(const std::string& phase, treemapper::TreeId target, const ros::Time& stamp) {
    const bool due = !published_phase_ || stamp.toSec() - phase_pub_t_ >= 1.0 || phase != last_phase_ || target != last_target_;
    if (!due) return;
    std_msgs::String s;
    s.data = phase;
    pub_state_.publish(s);
    std_msgs::Int32 i;
    i.data = target;
    pub_target_.publish(i);
    published_phase_ = true;
    phase_pub_t_ = stamp.toSec();
    last_phase_ = phase;
    last_target_ = target;
  }

  void publishIdle(const ros::Time& stamp, const treemapper::DronePose& pose) {
    publishSpray(false);
    publishPhase("IDLE", treemapper::kNoTree, stamp);
    if (pose.valid) pub_setpoint_.publish(hoverSetpoint(pose, stamp));
  }

  /// After an exception in the tick: nozzle closed, hover where the drone is.
  void safeOutputs(const ros::Time& stamp, const treemapper::DronePose& pose) {
    try {
      publishSpray(false);
      if (pose.valid && !stamp.isZero()) pub_setpoint_.publish(hoverSetpoint(pose, stamp));
    } catch (...) {
    }
  }

  geometry_msgs::PoseStamped hoverSetpoint(const treemapper::DronePose& pose, const ros::Time& stamp) const {
    corridor_nav::Outputs o;
    o.setpoint = {pose.x, pose.y, pose.z};
    o.yaw = yawOf(pose);
    return toSetpoint(o, world_frame_, stamp);
  }

  void onStatusTimer(const ros::TimerEvent&) {
    ros::Time stamp;
    try {
      stamp = ros::Time::now();
      if (stamp.isZero()) return;
      corridor_nav::Status status;   // default: IDLE, nothing planned
      corridor_nav::Counters counters;
      if (nav_) {
        status = nav_->status();
        counters = nav_->counters();
      } else {
        status.warnings.push_back("navigator not ready: " + waitingFor());
      }
      for (const auto& kv : node_counters_) counters[kv.first] = kv.second;
      if (!last_error_.empty() && stamp.toSec() - last_error_t_ < 10.0) status.warnings.push_back(last_error_);
      pub_status_.publish(toMsg(status, counters, stamp));
      pub_markers_.publish(markers(stamp));
    } catch (const std::exception& e) {
      onException("status", e.what(), stamp);
    } catch (...) {
      onException("status", "unknown exception", stamp);
    }
  }

  visualization_msgs::MarkerArray markers(const ros::Time& stamp) const {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker clear;
    clear.header.stamp = stamp;
    clear.header.frame_id = world_frame_;
    clear.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(clear);
    if (!nav_) return arr;
    auto strip = [&](const char* ns, int id, float r, float g, float b, double width) {
      visualization_msgs::Marker m;
      m.header = clear.header;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = width;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0f;
      return m;
    };
    for (const auto& c : nav_->corridors()) {
      auto m = strip("corridor", c.index, 0.2f, 0.5f, 1.0f, 0.1);
      for (std::size_t k = 0; k <= c.strip.size() && !c.strip.empty(); ++k) {
        const auto& p = c.strip[k % c.strip.size()];
        m.points.push_back(point(p.x, p.y, 0.0));
      }
      arr.markers.push_back(m);
    }
    int id = 0;
    for (const auto& path : nav_->plannedPaths()) {
      auto m = strip("path", id++, 1.0f, 0.8f, 0.0f, 0.12);
      for (const auto& p : path.pts) m.points.push_back(point(p.x, p.y, p.z));
      arr.markers.push_back(m);
    }
    return arr;
  }

  // --- helpers ----------------------------------------------------------------------------------------------
  std::string waitingFor() const {
    if (!fov_) return "no valid left camera_info yet";
    if (!mount_resolved_) return "camera mount TF " + body_frame_ + " <- the left camera frame not available yet";
    if (!field_has_home_) return "no pose " + world_frame_ + " <- " + body_frame_ + " yet (field file has no home)";
    return "starting";
  }

  template <class F>
  void guarded(const char* where, F&& f) {
    try {
      f();
    } catch (const std::exception& e) {
      onException(where, e.what(), ros::Time::now());
    } catch (...) {
      onException(where, "unknown exception", ros::Time::now());
    }
  }

  void onException(const char* where, const std::string& what, const ros::Time& now) {
    ++node_counters_[std::string("node_callback_exceptions_") + where];
    ++node_counters_["node_callback_exceptions"];
    last_error_ = std::string("callback exception in ") + where + ": " + what;
    last_error_t_ = now.toSec();
    ROS_ERROR("%s (node kept alive)", last_error_.c_str());
  }

  ros::NodeHandle nh_, pnh_;
  std::string world_frame_, body_frame_, set_sprayed_service_;
  corridor_nav::Config cfg_;   ///< sanitized
  corridor_nav::Field field_;
  bool field_has_home_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::CameraInfo::ConstPtr left_info_, right_info_;
  std::optional<corridor_nav::CameraFov> fov_;
  std::string mount_source_;
  double mount_wait_s_{5.0};
  double mount_yaw_{std::numeric_limits<double>::quiet_NaN()};   ///< body yaw of the image x-axis; NaN = unknown
  bool mount_resolved_{false};
  ros::Time mount_wait_start_;
  std::optional<corridor_nav::StereoIntrinsics> stereo_;
  unsigned calibration_seq_{0};
  sensor_msgs::Image::ConstPtr disparity_;
  unsigned disparity_seq_{0};
  corridor_nav::Clearance clearance_;
  unsigned clearance_disparity_seq_{0}, clearance_calibration_seq_{0};

  std::unique_ptr<corridor_nav::Navigator> nav_;
  std::optional<corridor_nav::MapSnapshot> pending_map_;
  bool start_requested_{false}, abort_requested_{false};
  double step_ms_{0.0};

  std::vector<SprayFuture> spray_calls_;
  std::deque<corridor_nav::SprayResult> spray_results_;

  bool published_phase_{false};
  double phase_pub_t_{0.0};
  std::string last_phase_;
  treemapper::TreeId last_target_{treemapper::kNoTree};

  std::map<std::string, std::uint64_t> node_counters_;
  std::string last_error_;
  double last_error_t_{0.0};

  ros::Publisher pub_state_, pub_target_, pub_setpoint_, pub_spray_, pub_status_, pub_markers_, pub_timing_;
  ros::ServiceServer srv_start_, srv_abort_;
  ros::Subscriber sub_map_, sub_disparity_, sub_left_, sub_right_;
  ros::Timer timer_tick_, timer_status_;
};

}  // namespace corridor_nav_ros

int main(int argc, char** argv) {
  // Our own SIGINT/SIGTERM handling: roscpp's tears the publishers down before spin() returns, and the nozzle-closed
  // message on the way out must still get out.
  ros::init(argc, argv, "corridor_nav", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, corridor_nav_ros::onSignal);
  std::signal(SIGTERM, corridor_nav_ros::onSignal);
  ros::NodeHandle nh, pnh("~");
  try {
    corridor_nav_ros::CorridorNavNode node(nh, pnh);
    while (ros::ok() && !corridor_nav_ros::g_stop) {
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.05));
    }
    node.shutdown();
    ros::WallDuration(0.2).sleep();   // let the publisher thread write the message out
  } catch (const std::exception& e) {
    ROS_FATAL("corridor_nav_node exiting: %s", e.what());
    ros::shutdown();
    return 1;
  }
  ros::shutdown();
  return 0;
}
