"""sim_frames: an Isaac closed-loop run directory -> TMF1 frame file (what treemapper_node consumed).

The synthetic bag below is written with ``rosbag`` (skipped when it is not importable, as on the host). Its messages
have the types of the closed-loop recording: ``treemapper_msgs/Detection2DArray`` (built from the repo's own .msg files,
so the test needs no catkin workspace), 32FC1 disparity, CameraInfo, ``/tf`` world -> body, ``/mission_state``,
``/spray_target``."""
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "ros", "sim_perception", "src"))

rosbag = pytest.importorskip("rosbag")
import genpy.dynamic  # noqa: E402  (ships with rosbag)
import rospy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import Int32, String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from treemapper_tools import sim_frames as sf  # noqa: E402
from treemapper_tools import sim_inputs as si  # noqa: E402
from sim_perception import disparity_grid as dgrid  # noqa: E402  (the recording node's decimation, no ROS needed)
from treemapper_tools.frames import read_frames  # noqa: E402

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
CAMERA = json.load(open(os.path.join(ROOT, "sim", "specs", "camera.json")))
MSG_DIR = os.path.join(ROOT, "ros", "treemapper_msgs", "msg")
W, H = 640, 400
HEADER_TXT = "uint32 seq\ntime stamp\nstring frame_id\n"


def det_array_class():
    def text(name):
        return open(os.path.join(MSG_DIR, name + ".msg")).read()
    sep = "=" * 80 + "\nMSG: "
    full = (text("Detection2DArray") + sep + "std_msgs/Header\n" + HEADER_TXT +
            sep + "treemapper_msgs/Detection2D\n" + text("Detection2D"))
    return genpy.dynamic.generate_dynamic("treemapper_msgs/Detection2DArray", full)["treemapper_msgs/Detection2DArray"]


Det2DArray = det_array_class()


def disparity_grid_class():
    def text(name):
        return open(os.path.join(MSG_DIR, name + ".msg")).read()
    full = text("DisparityGrid") + "=" * 80 + "\nMSG: std_msgs/Header\n" + HEADER_TXT
    return genpy.dynamic.generate_dynamic("treemapper_msgs/DisparityGrid", full)["treemapper_msgs/DisparityGrid"]


DisparityGrid = disparity_grid_class()


def plain_class(name):
    full = (open(os.path.join(MSG_DIR, name.split("/")[-1] + ".msg")).read() +
            "=" * 80 + "\nMSG: std_msgs/Header\n" + HEADER_TXT)
    return genpy.dynamic.generate_dynamic(name, full)[name]


StageTiming = plain_class("treemapper_msgs/StageTiming")
DropCounters = plain_class("treemapper_msgs/DropCounters")


def timing_msg(stamp):
    m = StageTiming()
    m.header.stamp = T(stamp)
    m.stages, m.ms = ["sgbm"], [40.0]
    return m


def grid_counters_msg(t, received, published=None, malformed=0, stamp_gap=0):
    m = DropCounters()
    m.header.stamp = T(t)
    m.stage = "disparity_grid"
    m.reasons = ["received", "published", "malformed", "stamp_gap"]
    m.counts = [received, received if published is None else published, malformed, stamp_gap]
    return m


def grid_of(img):
    """What sim_perception's disparity_grid_node publishes for the Image ``img``."""
    f = dgrid.grid_fields(img.width, img.height, img.step, img.encoding, img.is_bigendian, img.data, 4)
    m = DisparityGrid()
    m.header = img.header
    for k, v in f.items():
        setattr(m, k, v)
    return m


def write_disparity(bag, img, t, mode):
    """mode: 'grid' = what --record-frames records, 'full' = the full image (older bags), 'both'."""
    if mode in ("full", "both"):
        bag.write("/disparity_image_down_float", img, T(t))
    if mode in ("grid", "both"):
        bag.write("/rec/disparity_grid", grid_of(img), T(t))


def T(sec):
    return rospy.Time.from_sec(sec)


def cam_info(side, stamp, w=W, h=H):
    m = CameraInfo()
    m.header.stamp = T(stamp)
    m.header.frame_id = "bottom_%s_camera" % side
    m.width, m.height = w, h
    m.K = CAMERA[side]["K"]
    m.P = CAMERA[side]["P"]
    return m


def disparity(stamp, value=8.0, w=W, h=H, encoding="32FC1", step=None, bad_pixels=True, unique=False):
    m = Image()
    m.header.stamp = T(stamp)
    m.header.frame_id = "bottom_left_camera"
    m.width, m.height, m.encoding = w, h, encoding
    d = np.full((h, w), value, np.float32)
    if unique:           # every pixel different (stamp-dependent), a few NaN / zero / negative ones: any wrong sample shows
        d = (1.0 + (np.arange(w * h, dtype=np.float32).reshape(h, w) % 977) / 16.0 + stamp).astype(np.float32)
        d[5::7, 5::11] = np.nan
        d[2::8, 2::8] = 0.0
        d[6::8, 6::8] = -1.5
    if bad_pixels:
        d[0::4, 0::4] = np.nan          # a column of grid pixels the sampler must mark invalid
        d[2::4, 2::4][:, :5] = 0.0
    m.step = step if step is not None else 4 * w
    m.data = d.tobytes()
    return m


def det_elem_class():
    full = open(os.path.join(MSG_DIR, "Detection2D.msg")).read()
    return genpy.dynamic.generate_dynamic("treemapper_msgs/Detection2D", full)["treemapper_msgs/Detection2D"]


Det2D = det_elem_class()


def detections(stamp, boxes=((100.0, 50.0, 40.0, 60.0, 0.9, 0),)):
    m = Det2DArray()
    m.header.stamp = T(stamp)
    for x, y, w, h, s, lab in boxes:
        d = Det2D()
        d.x, d.y, d.width, d.height, d.score, d.label = x, y, w, h, s, lab
        m.detections.append(d)
    return m


def tf_msg(t, x, y=0.0, z=5.0, parent="world", child="body"):
    tr = TransformStamped()
    tr.header.stamp = T(t)
    tr.header.frame_id = parent
    tr.child_frame_id = child
    tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = x, y, z
    tr.transform.rotation.w = 1.0
    return TFMessage(transforms=[tr])


def make_run(tmp_path, n=3, t0=10.0, with_disparity=True, disp="grid", unique=False, sgbm_extra=0, grid_counters=True, phases=(("SCAN", 9.0), ("PLANNING", 10.12)),
             targets=((-1, 9.0), (4, 10.12)), tf_from=9.0, tf_to=11.0, info_t=9.5, extra=None):
    """A run dir with a bag of n detection+disparity pairs 0.1 s apart (stamp t0 + 0.1 k, disparity received 0.04 s
    after the stamp, the detection 0.05 s after), /tf at 100 Hz (x = t, z = 5), phases and targets."""
    run = tmp_path / "run"
    run.mkdir()
    with rosbag.Bag(str(run / "mission.bag"), "w") as bag:
        bag.write("/cam/bottom/left/half/camera_info", cam_info("left", info_t), T(info_t))
        bag.write("/cam/bottom/right/half/camera_info", cam_info("right", info_t), T(info_t))
        for ph, t in phases:
            bag.write("/mission_state", String(data=ph), T(t))
        for tid, t in targets:
            bag.write("/spray_target", Int32(data=tid), T(t))
        k = 0
        t = tf_from
        while t <= tf_to + 1e-9:
            bag.write("/tf", tf_msg(t, x=t), T(t))
            t = round(t + 0.01, 6)
        for k in range(n):
            stamp = round(t0 + 0.1 * k, 6)
            if with_disparity:
                write_disparity(bag, disparity(stamp, unique=unique), stamp + 0.04, disp)
                bag.write("/stereo_sgbm/timing", timing_msg(stamp), T(stamp + 0.041))       # one per published image
            bag.write("/object_detection/detections", detections(stamp), T(stamp + 0.05))
        for k in range(sgbm_extra):     # images the stereo node published that never reached the grid topic
            bag.write("/stereo_sgbm/timing", timing_msg(50.0 + k), T(50.0 + k))
        if grid_counters and disp in ("grid", "both") and with_disparity:
            bag.write("/disparity_grid/counters", grid_counters_msg(t0 + 0.1 * n, received=n), T(t0 + 0.1 * n))
        if extra:
            extra(bag)
    return run


def frames_of(run, name="frames.bin"):
    cal, meta, it = read_frames(str(run / name))
    return cal, meta, list(it)


# ---------------------------------------------------------------------------------------------------- pairing, pose

@pytest.mark.parametrize("mode", ["grid", "full"])
def test_sim_frames_pairs_and_poses(tmp_path, mode):
    run = make_run(tmp_path, disp=mode)
    meta = sf.convert(str(run), heartbeat=0.0)
    cal, _, frames = frames_of(run)
    assert len(frames) == 3 and meta["frames"] == 3 and meta["frames_with_disparity"] == 3
    assert meta["pose_invalid"] == 0 and all(f.pose_valid for f in frames)
    # pose = world -> body from /tf interpolated AT THE IMAGE STAMP (x = t in the synthetic bag)
    for f, stamp in zip(frames, (10.0, 10.1, 10.2)):
        assert f.stamp == pytest.approx(stamp) and f.pose[0] == pytest.approx(stamp, abs=1e-6)
        assert f.pose[2] == pytest.approx(5.0) and f.pose[6] == pytest.approx(1.0)
    # phase and hop target are the ones in force when the detection was received (10.05, 10.15, 10.25)
    assert [f.phase for f in frames] == ["SCAN", "PLANNING", "PLANNING"]
    assert [f.target_id for f in frames] == [-1, 4, 4]
    assert [f.t for f in frames] == pytest.approx([10.05, 10.15, 10.25])
    # boxes are the detections', in pixels, and the disparity is the stride-4 grid of the 32FC1 image
    assert frames[0].boxes.shape == (1, 6) and frames[0].boxes[0].tolist() == pytest.approx([100, 50, 40, 60, 0.9, 0])
    gh, gw = cal.grid_shape
    assert frames[0].disparity.shape == (gh, gw) == (100, 160)
    assert np.nanmedian(frames[0].disparity) == pytest.approx(8.0)
    assert np.isnan(frames[0].disparity[0, 0])          # the NaN pixels of the synthetic image stay invalid


def test_sim_frames_calibration_from_camera_info(tmp_path):
    run = make_run(tmp_path)
    sf.convert(str(run), heartbeat=0.0)
    cal, meta, _ = frames_of(run)
    L = CAMERA["left"]["P"]
    assert (cal.width, cal.height) == (W, H) and cal.stride == 4 and cal.frame == "world"
    assert (cal.fx, cal.fy, cal.cx, cal.cy) == pytest.approx((L[0], L[5], L[2], L[6]))
    # right P[3] = -fx * B in millimetres: B = 0.16 m, the same rule treemapper_node applies
    assert cal.baseline == pytest.approx(0.16, abs=1e-4)


def test_sim_frames_body_cam_from_camera_json(tmp_path):
    run = make_run(tmp_path)
    sf.convert(str(run), heartbeat=0.0)
    cal, _, _ = frames_of(run)
    assert cal.T_body_cam == pytest.approx(CAMERA["mount"], abs=1e-12)


def test_sim_launch_files_use_the_camera_json_mount():
    """The mount treemapper_node runs with (launch args) is the one sim_frames writes (camera.json)."""
    floats = lambda s: [float(v) for v in re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", s)]
    closed = ET.parse(os.path.join(ROOT, "ros", "corridor_nav_ros", "launch", "sim_closed_loop.launch")).getroot()
    mount = [a.get("default") for a in closed.iter("arg") if a.get("name") == "camera_mount"][0]
    assert floats(mount) == pytest.approx(CAMERA["mount"], abs=1e-12)
    tm = open(os.path.join(ROOT, "ros", "treemapper_ros", "launch", "treemapper_sim.launch")).read()
    m = re.search(r'param="camera_mount">\[(.*?)\]</rosparam>', tm, re.S)
    assert floats(m.group(1)) == pytest.approx(CAMERA["mount"], abs=1e-12)


def test_sim_closed_loop_launch_records_the_frame_topics_only_on_request():
    root = ET.parse(os.path.join(ROOT, "ros", "corridor_nav_ros", "launch", "sim_closed_loop.launch")).getroot()
    arg = [a for a in root.iter("arg") if a.get("name") == "record_frames"]
    assert len(arg) == 1 and arg[0].get("default") == "false"
    when = [a for a in root.iter("arg") if a.get("name") == "rec_frame_topics" and a.get("if") == "$(arg record_frames)"]
    # the decimated grid and the two camera_info; the full ~1 MB image is NOT recorded
    assert len(when) == 1 and when[0].get("value").split() == ["/rec/disparity_grid", "/disparity_grid/counters",
                                                               "/cam/bottom/left/half/camera_info",
                                                               "/cam/bottom/right/half/camera_info"]
    unless = [a for a in root.iter("arg") if a.get("name") == "rec_frame_topics" and a.get("unless")]
    assert len(unless) == 1 and unless[0].get("unless") == "$(arg record_frames)" and unless[0].get("value") == ""
    rec = [n for n in root.iter("node") if n.get("name") == "mission_recorder"][0]
    assert "$(arg rec_frame_topics)" in rec.get("args") and "/disparity_image_down_float" not in rec.get("args")
    # the recording-only decimation node exists only with record_frames, reading the full image, stride 4
    node = [n for n in root.iter("node") if n.get("type") == "disparity_grid_node.py"]
    assert len(node) == 1 and node[0].get("pkg") == "sim_perception" and node[0].get("if") == "$(arg record_frames)"
    params = {p.get("name"): p.get("value") for p in node[0].iter("param")}
    assert params == {"in_topic": "/disparity_image_down_float", "out_topic": "/rec/disparity_grid", "stride": "4"}


def test_grid_node_script_is_installed_and_the_message_is_registered():
    cm = open(os.path.join(ROOT, "ros", "sim_perception", "CMakeLists.txt")).read()
    assert "scripts/disparity_grid_node.py" in cm
    assert os.access(os.path.join(ROOT, "ros", "sim_perception", "scripts", "disparity_grid_node.py"), os.X_OK)
    assert "DisparityGrid.msg" in open(os.path.join(ROOT, "ros", "treemapper_msgs", "CMakeLists.txt")).read()


# ---------------------------------------------------------------------------------------------------- the live node's rules

def test_frames_before_the_calibration_latch_are_dropped(tmp_path):
    # camera_info arrives at 10.12: the node (no mapper yet) drops the first detection (received 10.05)
    run = make_run(tmp_path, info_t=10.12)
    meta = sf.convert(str(run), heartbeat=0.0)
    _, _, frames = frames_of(run)
    assert [round(f.stamp, 3) for f in frames] == [10.1, 10.2] and meta["t_start"] == pytest.approx(10.12)


def test_missing_disparity_gives_frames_without_it(tmp_path):
    run = make_run(tmp_path, with_disparity=False)
    meta = sf.convert(str(run), heartbeat=0.0)
    assert meta["frames"] == 3 and meta["frames_with_disparity"] == 0


@pytest.mark.parametrize("mode", ["grid", "full"])
def test_malformed_and_wrong_size_disparity_are_rejected_and_counted(tmp_path, mode):
    def extra(bag):
        write_disparity(bag, disparity(10.0, step=W), 10.041, mode)                    # step = width
        write_disparity(bag, disparity(10.1, w=320, h=200), 10.141, mode)               # other size
    run = make_run(tmp_path, extra=extra, disp=mode)
    meta = sf.convert(str(run), heartbeat=0.0)
    assert meta["disparity_rejected"] == 2 and meta["frames_with_disparity"] == 3
    assert meta["counts"]["disparity_rejected_malformed"] == 1
    assert meta["counts"]["disparity_rejected_resolution"] == 1


def test_pose_invalid_outside_the_tf_range(tmp_path):
    run = make_run(tmp_path, tf_from=10.05, tf_to=11.0)    # first stamp 10.0 is before the first /tf sample
    meta = sf.convert(str(run), heartbeat=0.0)
    _, _, frames = frames_of(run)
    assert meta["pose_invalid"] == 1 and [f.pose_valid for f in frames] == [False, True, True]


def test_heartbeat_frames_at_phase_changes(tmp_path):
    run = make_run(tmp_path)
    meta = sf.convert(str(run))                              # default heartbeat 0.2 s
    _, _, frames = frames_of(run)
    hb = [f for f in frames if f.disparity is None]
    assert meta["frames"] == len(frames) and meta["frames_with_disparity"] == 3 and len(hb) >= 1
    assert any(f.phase == "PLANNING" and f.t == pytest.approx(10.12) for f in hb)   # the phase-change heartbeat
    assert all(f.boxes.shape == (0, 6) for f in hb)
    assert [f.t for f in frames] == sorted(f.t for f in frames)


def test_mission_json_in_the_run_dir_is_used_when_present(tmp_path):
    run = make_run(tmp_path, phases=(("SCAN", 9.0),), targets=((-1, 9.0),))
    # scorer-written mission.json says PLANNING from 10.12 and target 7 (the bag does not)
    json.dump({"phases": [[9.0, "SCAN"], [10.12, "PLANNING"]], "targets": [[9.0, -1], [10.12, 7]], "pose": []},
              open(str(run / "mission.json"), "w"))
    sf.convert(str(run), heartbeat=0.0)
    _, _, frames = frames_of(run)
    assert [f.phase for f in frames] == ["SCAN", "PLANNING", "PLANNING"] and [f.target_id for f in frames] == [-1, 7, 7]


def test_meta_keys_and_run_provenance(tmp_path):
    run = make_run(tmp_path)
    json.dump({"strategy": "corridor", "seed": 3, "git_commit": "abc", "spec": {"name": "x"}}, open(str(run / "run.json"), "w"))
    meta = sf.convert(str(run), heartbeat=0.0)
    on_disk = json.load(open(str(run / "frames.bin.meta.json")))
    for k in ("frames", "frames_with_disparity", "pose_invalid", "disparity_rejected", "t_start", "t_end"):
        assert k in meta and on_disk[k] == meta[k]
    assert on_disk["run"]["strategy"] == "corridor" and on_disk["run"]["seed"] == 3
    assert meta["t_end"] >= meta["t_start"]


# ---------------------------------------------------------------------------------------------------- check()

def test_check_flags_missing_disparity():
    bad = sf.check({"frames": 3, "frames_with_disparity": 0, "pose_invalid": 0, "disparity_rejected": 0})
    assert len(bad) == 1 and "disparity" in bad[0]


def test_check_flags_rejected_disparity():
    ok = {"frames": 100, "detection_frames": 100, "frames_with_disparity": 100, "pose_invalid": 0, "disparity_rejected": 0}
    assert sf.check(ok) == []
    bad = sf.check(dict(ok, disparity_rejected=1))
    assert len(bad) == 1 and "disparity_rejected 1" in bad[0]


def test_check_flags_fewer_than_80_percent_of_detection_frames_paired():
    # 10 detection frames + 5 heartbeats (heartbeats have no disparity and do not count)
    meta = {"frames": 15, "detection_frames": 10, "frames_with_disparity": 8, "pose_invalid": 0, "disparity_rejected": 0}
    assert sf.check(meta) == []                                         # exactly 80 %: allowed
    bad = sf.check(dict(meta, frames_with_disparity=7))
    assert len(bad) == 1 and "7 of 10" in bad[0]


def test_convert_reports_detection_frames_for_the_pairing_check(tmp_path):
    run = make_run(tmp_path)
    meta = sf.convert(str(run))                                          # heartbeats on
    assert meta["detection_frames"] == 3 and meta["frames"] > 3 and sf.check(meta) == []


def test_cli_fails_when_a_disparity_is_rejected_and_keeps_the_bag(tmp_path, capsys):
    run = make_run(tmp_path, extra=lambda bag: write_disparity(bag, disparity(10.1, w=320, h=200), 10.141, "grid"))
    before = bag_counts(str(run / "mission.bag"))
    assert sf.main(["--run", str(run), "--filter-bag"]) == 1
    assert "disparity_rejected 1" in capsys.readouterr().err
    assert bag_counts(str(run / "mission.bag")) == before


def test_cli_fails_when_too_few_detections_have_a_disparity(tmp_path, capsys):
    def extra(bag):          # 10 more detections without any disparity image
        for k in range(10):
            bag.write("/object_detection/detections", detections(round(20.0 + 0.1 * k, 6)), T(20.05 + 0.1 * k))
    run = make_run(tmp_path, extra=extra, tf_to=22.0)
    assert sf.main(["--run", str(run), "--heartbeat", "0"]) == 1
    assert "3 of 13 detection frames" in capsys.readouterr().err


# ---------------------------------------------------------------------------------------------------- grid loss accounting

def grid_ok_meta(**kw):
    m = {"frames": 100, "detection_frames": 100, "frames_with_disparity": 100, "pose_invalid": 0, "disparity_rejected": 0,
         "disparity_topic": "/rec/disparity_grid", "sgbm_published": 1000, "grid_messages": 1000,
         "grid_node": {"received": 1000, "published": 1000, "malformed": 0, "stamp_gap": 0}}
    m.update(kw)
    return m


def test_grid_bag_with_counters_and_no_loss_passes(tmp_path):
    run = make_run(tmp_path)
    meta = sf.convert(str(run))
    assert meta["disparity_topic"] == "/rec/disparity_grid"
    assert (meta["sgbm_published"], meta["grid_messages"], meta["grid_lost"]) == (3, 3, 0)
    assert meta["grid_node"] == {"received": 3, "published": 3, "malformed": 0, "stamp_gap": 0}
    assert meta["grid_stamp_gap"] == 0 and meta["grid_malformed"] == 0
    assert sf.check(meta) == []
    assert sf.main(["--run", str(run), "--filter-bag"]) == 0


def test_grid_loss_of_two_percent_fails_and_keeps_the_bag(tmp_path, capsys):
    run = make_run(tmp_path, n=50, sgbm_extra=1, tf_to=16.0)           # the stereo node published 51 images, 50 grids arrived
    meta = sf.convert(str(run))
    assert (meta["sgbm_published"], meta["grid_messages"], meta["grid_lost"]) == (51, 50, 1)
    bad = sf.check(meta)
    assert len(bad) == 1 and "grid_lost 1 of 51" in bad[0]
    before = bag_counts(str(run / "mission.bag"))
    assert sf.main(["--run", str(run), "--filter-bag"]) == 1
    assert "grid_lost 1 of 51" in capsys.readouterr().err and bag_counts(str(run / "mission.bag")) == before


def test_grid_loss_check_boundary():
    assert sf.check(grid_ok_meta(sgbm_published=1010, grid_messages=1000)) == []          # exactly 1 %
    assert len(sf.check(grid_ok_meta(sgbm_published=1011, grid_messages=1000))) == 1


def test_grid_counters_missing_fails_loud(tmp_path, capsys):
    run = make_run(tmp_path, grid_counters=False)
    meta = sf.convert(str(run))
    assert meta["grid_node"] is None
    bad = sf.check(meta)
    assert len(bad) == 1 and "/disparity_grid/counters is missing" in bad[0]
    assert sf.main(["--run", str(run), "--filter-bag"]) == 1


def test_stereo_published_count_missing_fails_loud():
    bad = sf.check(grid_ok_meta(sgbm_published=None, grid_messages=1000))
    assert len(bad) == 1 and "/stereo_sgbm/timing" in bad[0]
    assert len(sf.check(grid_ok_meta(sgbm_published=0))) == 1


def test_full_image_bag_skips_the_loss_check(tmp_path):
    run = make_run(tmp_path, disp="full", sgbm_extra=5, grid_counters=False)    # more published than ever seen: irrelevant
    meta = sf.convert(str(run))
    assert meta["disparity_topic"] == "/disparity_image_down_float" and meta["grid_lost"] is None and meta["grid_node"] is None
    assert sf.check(meta) == []
    assert sf.main(["--run", str(run)]) == 0
    # a bare meta without a topic never looks at the counters (the contract's old callers)
    assert sf.check({"frames": 3, "detection_frames": 3, "frames_with_disparity": 3, "pose_invalid": 0}) == []


def test_check_flags_pose_invalid_above_one_percent_and_empty():
    ok = {"frames": 1000, "frames_with_disparity": 900, "pose_invalid": 10, "disparity_rejected": 0}
    assert sf.check(ok) == []                                           # exactly 1 %: allowed
    bad = sf.check(dict(ok, pose_invalid=11))
    assert len(bad) == 1 and "pose" in bad[0]
    assert sf.check({"frames": 0, "frames_with_disparity": 0, "pose_invalid": 0})  # nothing at all is a failure
    assert sf.check({}) != []                                           # missing keys never pass


# ---------------------------------------------------------------------------------------------------- CLI and bag filter

def rospy_ser(m):
    from io import BytesIO
    buf = BytesIO()
    m.serialize(buf)
    return buf.getvalue()


def bag_counts(path):
    with rosbag.Bag(path) as b:
        return {t: i.message_count for t, i in b.get_type_and_topic_info().topics.items()}


def test_cli_ok_writes_both_files_and_keeps_the_bag(tmp_path, capsys):
    run = make_run(tmp_path)
    before = bag_counts(str(run / "mission.bag"))
    assert sf.main(["--run", str(run)]) == 0
    assert (run / "frames.bin").exists() and (run / "frames.bin.meta.json").exists()
    assert bag_counts(str(run / "mission.bag")) == before                  # no --filter-bag: untouched


def test_cli_out_option(tmp_path):
    run = make_run(tmp_path)
    out = tmp_path / "elsewhere" / "f.bin"
    assert sf.main(["--run", str(run), "--out", str(out)]) == 0
    assert out.exists() and os.path.exists(str(out) + ".meta.json") and not (run / "frames.bin").exists()


def test_cli_fails_without_disparity_and_keeps_the_bag(tmp_path, capsys):
    run = make_run(tmp_path, with_disparity=False)
    before = bag_counts(str(run / "mission.bag"))
    assert sf.main(["--run", str(run), "--filter-bag"]) != 0
    assert "disparity" in capsys.readouterr().err
    assert bag_counts(str(run / "mission.bag")) == before and not (run / "mission.bag.filtered").exists()


def test_cli_fails_on_missing_bag(tmp_path, capsys):
    (tmp_path / "run").mkdir()
    assert sf.main(["--run", str(tmp_path / "run")]) != 0


def test_filter_bag_drops_only_the_frame_topics_and_replaces_after_check(tmp_path):
    run = make_run(tmp_path)
    before = bag_counts(str(run / "mission.bag"))
    assert sf.main(["--run", str(run), "--filter-bag"]) == 0
    after = bag_counts(str(run / "mission.bag"))
    dropped = {"/rec/disparity_grid", "/disparity_grid/counters", "/cam/bottom/left/half/camera_info",
               "/cam/bottom/right/half/camera_info"}
    assert set(after) == set(before) - dropped
    assert all(after[t] == before[t] for t in after)                       # every kept message is still there
    assert sorted(os.listdir(str(run))) == ["frames.bin", "frames.bin.meta.json", "mission.bag"]   # no temp file left
    # the filtered bag is still a normal run bag: mission_score reads its topics unchanged
    with rosbag.Bag(str(run / "mission.bag")) as b:
        msgs = list(b.read_messages(topics=["/mission_state", "/tf"]))
    assert len(msgs) == before["/mission_state"] + before["/tf"]


def test_filter_bag_drops_the_full_image_as_well_when_it_is_there(tmp_path):
    run = make_run(tmp_path, disp="both")
    assert sf.main(["--run", str(run), "--filter-bag"]) == 0
    assert not {"/rec/disparity_grid", "/disparity_image_down_float", "/cam/bottom/left/half/camera_info"} & set(
        bag_counts(str(run / "mission.bag")))


def test_filter_failure_removes_the_partial_copy_and_keeps_the_bag(tmp_path, capsys, monkeypatch):
    # the filter dies half way (here: after writing part of the copy): main fails loudly, the partial
    # mission.bag.filtered is removed and mission.bag is byte-for-byte what it was
    run = make_run(tmp_path)
    bag = run / "mission.bag"
    before_bytes = bag.read_bytes()

    def broken_filter(src, dst, drop=sf.DROP_TOPICS):
        with open(dst, "wb") as f:
            f.write(b"#ROSBAG V2.0\n partial")
        raise RuntimeError("disk full")

    monkeypatch.setattr(sf, "filter_bag", broken_filter)
    assert sf.main(["--run", str(run), "--filter-bag"]) == 2
    err = capsys.readouterr().err
    assert "cannot filter" in err and "kept as it is" in err and "disk full" in err
    assert not (run / "mission.bag.filtered").exists()
    assert bag.read_bytes() == before_bytes
    assert (run / "frames.bin").exists()                   # the conversion itself had passed its check


def test_filter_bag_function_is_a_byte_faithful_message_copy(tmp_path):
    run = make_run(tmp_path)
    dst = str(tmp_path / "copy.bag")
    n = sf.filter_bag(str(run / "mission.bag"), dst)
    assert n["dropped"] == 3 + 1 + 2 and n["kept"] > 0     # 3 grids + their counters + 2 camera_info
    keep = ["/mission_state", "/spray_target", "/tf", "/object_detection/detections"]
    with rosbag.Bag(str(run / "mission.bag")) as a, rosbag.Bag(dst) as b:
        ka = [(t, rospy_ser(m), bt.to_sec()) for t, m, bt in a.read_messages(topics=keep)]
        kb = [(t, rospy_ser(m), bt.to_sec()) for t, m, bt in b.read_messages(topics=keep)]
    assert ka == kb and len(ka) > 100                      # same messages, same order, same times


# ---------------------------------------------------------------------------------------------------- full image vs grid

def record_region(path):
    """The bytes of a TMF1 file after its header (calibration block + all frame records)."""
    import struct
    b = open(path, "rb").read()
    (n,) = struct.unpack("<I", b[4:8])
    return b[8 + n:]


def test_frames_from_the_decimated_grid_equal_frames_from_the_full_image(tmp_path):
    """One synthetic full disparity image per stamp, recorded once as the full image and once as the grid message the
    recording node publishes: identical frames (every field, every disparity value, the record bytes), identical counts."""
    (tmp_path / "a").mkdir()
    (tmp_path / "b").mkdir()
    runs = {}
    for name, mode in (("a", "full"), ("b", "grid")):
        def extra(bag, mode=mode):
            write_disparity(bag, disparity(10.0, step=W), 10.041, mode)
            write_disparity(bag, disparity(10.1, w=320, h=200), 10.141, mode)
            write_disparity(bag, disparity(10.2, unique=True), 10.243, mode)       # duplicate stamp, ignored
        runs[name] = make_run(tmp_path / name, n=5, unique=True, disp=mode, extra=extra)
    ma = sf.convert(str(runs["a"]))
    mb = sf.convert(str(runs["b"]))
    assert ma["disparity_topic"] == "/disparity_image_down_float" and mb["disparity_topic"] == "/rec/disparity_grid"
    _, _, fa = frames_of(runs["a"])
    _, _, fb = frames_of(runs["b"])
    assert len(fa) == len(fb) > 5
    for x, y in zip(fa, fb):
        assert (x.t, x.stamp, x.pose, x.pose_valid, x.phase, x.target_id) == (y.t, y.stamp, y.pose, y.pose_valid, y.phase, y.target_id)
        assert np.array_equal(x.boxes, y.boxes)
        assert (x.disparity is None) == (y.disparity is None)
        if x.disparity is not None:
            assert np.array_equal(x.disparity, y.disparity, equal_nan=True)
    assert record_region(str(runs["a"] / "frames.bin")) == record_region(str(runs["b"] / "frames.bin"))
    keys = ("frames", "frames_with_disparity", "pose_invalid", "disparity_rejected", "detection_frames", "t_start", "t_end")
    assert {k: ma[k] for k in keys} == {k: mb[k] for k in keys}
    assert ma["counts"] == mb["counts"] and ma["disparity_rejected"] == 2
    # the sampled values are real (not all NaN) and include the NaN / zero / negative source pixels as invalid
    d = next(f.disparity for f in fa if f.disparity is not None)
    assert np.isfinite(d).sum() > 0.3 * d.size and np.isnan(d).sum() > 0


def test_a_grid_message_is_much_smaller_than_the_full_image():
    img = disparity(10.0, unique=True)
    from io import BytesIO
    full, grid = BytesIO(), BytesIO()
    img.serialize(full)
    grid_of(img).serialize(grid)
    assert len(full.getvalue()) > 1_000_000 and len(grid.getvalue()) < 64_300       # 160 x 100 float32 + header


# ---------------------------------------------------------------------------------------------------- harness flag

def test_sim_inputs_records_the_record_frames_flag(tmp_path):
    spec = os.path.join(ROOT, "sim", "specs", "3x3_smoke.json")
    base = os.path.join(ROOT, "config", "corridor_nav.cfg")
    assert si.main(["--spec", spec, "--run-dir", str(tmp_path), "--cfg", base, "--strategy", "corridor"]) == 0
    assert json.load(open(str(tmp_path / "run.json")))["record_frames"] is False
    d2 = tmp_path / "b"
    d2.mkdir()
    assert si.main(["--spec", spec, "--run-dir", str(d2), "--cfg", base, "--strategy", "corridor",
                    "--record-frames"]) == 0
    run = json.load(open(str(d2 / "run.json")))
    assert run["record_frames"] is True and run["record_images"] is False


def test_sim_mission_script_parses_the_flag():
    text = open(os.path.join(ROOT, "scripts", "sim_mission.sh")).read()
    assert re.search(r"--record-frames\) RECORD_FRAMES=true; shift ;;", text)
    assert "record_frames:=$RECORD_FRAMES" in text
    assert "--record-frames" in text.split("set -euo pipefail")[0]         # documented in the usage header
    assert "mission.bag.filtered" in text and "frames.bin.meta.json" in text   # --force-clean knows the new outputs
