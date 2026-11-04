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

rosbag = pytest.importorskip("rosbag")
import genpy.dynamic  # noqa: E402  (ships with rosbag)
import rospy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import Int32, String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

from treemapper_tools import sim_frames as sf  # noqa: E402
from treemapper_tools import sim_inputs as si  # noqa: E402
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


def disparity(stamp, value=8.0, w=W, h=H, encoding="32FC1", step=None, bad_pixels=True):
    m = Image()
    m.header.stamp = T(stamp)
    m.header.frame_id = "bottom_left_camera"
    m.width, m.height, m.encoding = w, h, encoding
    d = np.full((h, w), value, np.float32)
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


def make_run(tmp_path, n=3, t0=10.0, with_disparity=True, phases=(("SCAN", 9.0), ("PLANNING", 10.12)),
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
                bag.write("/disparity_image_down_float", disparity(stamp), T(stamp + 0.04))
            bag.write("/object_detection/detections", detections(stamp), T(stamp + 0.05))
        if extra:
            extra(bag)
    return run


def frames_of(run, name="frames.bin"):
    cal, meta, it = read_frames(str(run / name))
    return cal, meta, list(it)


# ---------------------------------------------------------------------------------------------------- pairing, pose

def test_sim_frames_pairs_and_poses(tmp_path):
    run = make_run(tmp_path)
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
    assert len(when) == 1 and when[0].get("value").split() == ["/disparity_image_down_float",
                                                               "/cam/bottom/left/half/camera_info",
                                                               "/cam/bottom/right/half/camera_info"]
    unless = [a for a in root.iter("arg") if a.get("name") == "rec_frame_topics" and a.get("unless")]
    assert len(unless) == 1 and unless[0].get("unless") == "$(arg record_frames)" and unless[0].get("value") == ""
    rec = [n for n in root.iter("node") if n.get("name") == "mission_recorder"][0]
    assert "$(arg rec_frame_topics)" in rec.get("args")


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


def test_malformed_and_wrong_size_disparity_are_rejected_and_counted(tmp_path):
    def extra(bag):
        bag.write("/disparity_image_down_float", disparity(10.0, step=W), T(10.041))              # step = width
        bag.write("/disparity_image_down_float", disparity(10.1, w=320, h=200), T(10.141))         # other size
    run = make_run(tmp_path, extra=extra)
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
    dropped = {"/disparity_image_down_float", "/cam/bottom/left/half/camera_info", "/cam/bottom/right/half/camera_info"}
    assert set(after) == set(before) - dropped
    assert all(after[t] == before[t] for t in after)                       # every kept message is still there
    assert sorted(os.listdir(str(run))) == ["frames.bin", "frames.bin.meta.json", "mission.bag"]   # no temp file left
    # the filtered bag is still a normal run bag: mission_score reads its topics unchanged
    with rosbag.Bag(str(run / "mission.bag")) as b:
        msgs = list(b.read_messages(topics=["/mission_state", "/tf"]))
    assert len(msgs) == before["/mission_state"] + before["/tf"]


def test_filter_bag_function_is_a_byte_faithful_message_copy(tmp_path):
    run = make_run(tmp_path)
    dst = str(tmp_path / "copy.bag")
    n = sf.filter_bag(str(run / "mission.bag"), dst)
    assert n["dropped"] == 3 + 2 and n["kept"] > 0         # 3 disparity images + 2 camera_info
    keep = ["/mission_state", "/spray_target", "/tf", "/object_detection/detections"]
    with rosbag.Bag(str(run / "mission.bag")) as a, rosbag.Bag(dst) as b:
        ka = [(t, rospy_ser(m), bt.to_sec()) for t, m, bt in a.read_messages(topics=keep)]
        kb = [(t, rospy_ser(m), bt.to_sec()) for t, m, bt in b.read_messages(topics=keep)]
    assert ka == kb and len(ka) > 100                      # same messages, same order, same times


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
