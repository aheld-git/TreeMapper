"""sim_inputs: the run cfg differs from the shipped cfg only in strategy and carrot_lead_s; field.yaml has exactly the
keys and shapes ros/corridor_nav_ros/src/field_file.cpp accepts."""
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from treemapper_tools import sim_inputs as si

ROOT = os.path.join(os.path.dirname(__file__), "..", "..")
BASE_CFG = os.path.join(ROOT, "config", "corridor_nav.cfg")
SPEC = os.path.join(ROOT, "sim", "specs", "3x3_smoke.json")


def cfg_dict(text):
    out = {}
    for line in text.splitlines():
        body = line.split("#", 1)[0]
        if "=" in body:
            k, v = (x.strip() for x in body.split("=", 1))
            assert k not in out, "duplicate key %s" % k
            out[k] = v
    return out


@pytest.mark.parametrize("strategy", ["corridor", "field_first"])
def test_run_cfg_equals_base_except_strategy_and_carrot_lead(tmp_path, strategy):
    si.write_run_inputs(SPEC, str(tmp_path), BASE_CFG, strategy)
    base = cfg_dict(open(BASE_CFG).read())
    run = cfg_dict(open(str(tmp_path / "corridor_nav.cfg")).read())
    assert set(run) == set(base)
    diff = {k for k in base if base[k] != run[k]}
    assert diff <= {"strategy", "carrot_lead_s"}
    assert run["strategy"] == strategy and run["carrot_lead_s"] == "1.2"
    assert {k: v for k, v in run.items() if k not in ("strategy", "carrot_lead_s")} == \
        {k: v for k, v in base.items() if k not in ("strategy", "carrot_lead_s")}


def test_run_cfg_errors():
    with pytest.raises(ValueError):
        si.cfg_overrides("zigzag")
    with pytest.raises(ValueError):
        si.run_cfg_text("z_scan = 15\n", {"strategy": "corridor"})                 # key missing
    with pytest.raises(ValueError):
        si.run_cfg_text("strategy = a\nstrategy = b\n", {"strategy": "corridor"})  # duplicate


def parse_field_yaml(text):
    """The subset of YAML field.yaml uses, parsed the way field_file.cpp reads it (flow lists of numbers)."""
    import ast
    out = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        k, v = (x.strip() for x in line.split(":", 1))
        out[k] = ast.literal_eval(v)
    return out


def test_field_yaml_has_exactly_the_field_file_schema(tmp_path):
    si.write_run_inputs(SPEC, str(tmp_path), BASE_CFG, "corridor")
    spec = json.load(open(SPEC))
    y = parse_field_yaml(open(str(tmp_path / "field.yaml")).read())
    assert tuple(y) == si.FIELD_KEYS                       # field_file.cpp rejects any other key
    assert len(y["polygon"]) >= 3 and all(len(p) == 2 for p in y["polygon"])
    assert y["polygon"] == [[float(x), float(v)] for x, v in spec["field"]["polygon"]]
    assert y["heading_rad"] == float(spec["rows"]["heading_rad"])
    assert y["home"] == [float(v) for v in spec["drone"]["home"]]
    try:
        import yaml                                         # the real YAML reader agrees when it is installed
        assert yaml.safe_load(open(str(tmp_path / "field.yaml"))) == y
    except ImportError:
        pass
    run = json.load(open(str(tmp_path / "run.json")))
    assert run["strategy"] == "corridor" and run["cfg_overrides"] == {"strategy": "corridor", "carrot_lead_s": "1.2"}
    assert json.load(open(str(tmp_path / "spec.json"))) == spec


def test_field_yaml_rejects_degenerate_fields():
    spec = json.load(open(SPEC))
    spec["field"]["polygon"] = [[0, 0], [1, 1]]
    with pytest.raises(ValueError):
        si.field_yaml_text(spec)
    spec["field"]["polygon"] = [[0, 0], [1, 1], [2, 2]]
    with pytest.raises(ValueError):
        si.field_yaml_text(spec)
