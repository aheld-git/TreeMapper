"""Print the README parameter table: every key of config/cand_obj.cfg with its sim and real-profile value.

    python3 -m treemapper_tools.param_table      (from tools/; reads ../config/cand_obj.cfg and ../config/cand_obj_real.cfg)

Groups come from the "# --- group" lines of cand_obj.cfg; descriptions and trade-offs are kept here. A key without a
description fails loudly, so a new tunable cannot be left out of the README table.
"""
from __future__ import annotations
import os, re, sys
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
def read(p):
    groups, vals, cur = [], {}, 'other'
    if not os.path.exists(p):
        return None, None
    for line in open(p):
        m = re.match(r'#\s*---\s*(.*)', line.strip())
        if m:
            cur = m.group(1).strip(); groups.append((cur, [])); continue
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        k, v = [x.strip() for x in line.split('=', 1)]
        vals[k] = v
        if not groups: groups.append((cur, []))
        groups[-1][1].append(k)
    return groups, vals
D = {
 'min_range': ('m, optical depth gate (near)', 'lower keeps very close points (low hover); very near stereo is unreliable'),
 'max_range': ('m, optical depth gate (far)', 'higher sees more of the swath edge but admits noisy far depth (sigma_Z ~ Z^2)'),
 'above_drone_margin': ('m, points above drone z - margin are dropped (impossible)', 'larger also drops real tops when the drone hovers low over a tall neighbour'),
 'max_frame_noise': ('m, a frame is dropped when 1.4826 x median per-cell MAD of z exceeds it (noise burst, turns)', 'lower rejects more blurred turn frames (fewer artefacts, slower evidence); higher admits bursts'),
 'height_sample_clearance': ('m, cells within this of the drone altitude give no height (the above-drone cap may have cut the top)', 'larger is safer against shoulder heights under a low hover, but low hovers then add no height'),
 'cell': ('m, world lattice cell (CHM and every evidence grid)', 'smaller = finer outlines but fewer points per cell (noisier medians, more fragments); larger merges gaps'),
 'min_cell_points': ('points for a cell to be known', 'higher = more robust medians, more unknown cells at range'),
 'max_grid': ('cells per side cap of the per-frame CHM window', 'memory/CPU bound; too small truncates the footprint at high altitude'),
 'z_ref': ('m, range of unit point weight, weight ~ (z_ref/Z)^4', 'sets how fast far points lose influence on heights and evidence'),
 'lateral_k': ('off-axis weight, weight /= 1 + k rho^2', 'higher trusts only near-nadir points (tilted real mount sees less weight)'),
 'rot_rate_ref': ('rad/s, frame quality = 1 / (1 + (rate/ref)^2)', 'lower discounts turns more (fewer turn-blur fragments, +0.1-0.2 s latency at 0.35)'),
 'quality_recover_s': ('s, frame quality recovers linearly after a fast rotation', 'longer = more caution after turns, later latching at row ends'),
 'ground_low_fraction': ('fraction of lowest cells the ground plane is fitted to', 'higher is stabler on open ground but lifts the plane under dense canopy'),
 'ground_inlier_band': ('m, refit band around the first plane', 'wider tolerates rough ground, narrower rejects low bushes'),
 'ground_max_slope': ('max |slope| of an accepted plane (else flat fallback, counted)', 'higher allows steep terrain but accepts fits tilted by canopy'),
 'min_above_ground': ('m, smoothed cell this far above the plane is canopy', 'higher rejects bushes/ground noise, loses low crown skirts'),
 'min_prominence': ('m, a basin peak this little above its saddle merges into the neighbour', 'the touching-tree knob: lower splits touching crowns (and over-fragments rough real crowns); higher merges neighbours'),
 'min_basin_cells': ('cells, smaller crowns ignored', 'higher suppresses noise blobs, delays small/edge crowns'),
 'box_dilate': ('fraction, boxes grown before the crown-top test', 'higher tolerates box jitter / pixel misalignment, admits more non-tree tops'),
 'peak_border_px': ('px, a crown top this close to the image edge is flagged truncated (its top may be outside the image)', 'with seed_truncated = false truncated crowns never seed; either way their height samples do not count toward the min_height_cells needed for publication (a tree seen only truncated never publishes)'),
 'seed_truncated': ('a crown whose top is flagged truncated may still seed an instance', 'true = earlier evidence (latency), slightly more neighbour spill at crown entry'),
 'window': ('cells per side of an instance evidence window', 'must cover the largest crown (28 x 0.4 = 11.2 m); memory ~14 kB per instance'),
 'hull_cell_corners': ('outline from latched cell corners (true) or centres (false)', 'corners = larger outlines (+spill), centres = tighter'),
 'match_radius': ('m, crown top this close to an instance centre is that instance', 'higher re-associates more (fewer duplicates), risks crediting a neighbour'),
 'match_overlap': ('fraction of an instance latched cells a crown must cover to be it, wherever its top is', 'lower = fewer transient duplicates, more neighbour takeover risk'),
 'match_reach': ('m, ... provided the top is within this of the centre', 'larger helps big crowns, credits neighbours at tight spacing (hard recall 0.78 at 4 m)'),
 'seed_overlap': ('fraction of a would-be seed on a published tree latched cells that credits it to that tree (seed guard)', 'lower prevents duplicates, can swallow a neighbour first slice'),
 'max_cell_radius': ('m, cells farther from the instance centre are not credited (spill guard)', 'must exceed the largest crown radius; higher allows spill'),
 'latch_rel_h': ('skirt trim: cell claimed only if its height >= this x crown peak (0 = off)', 'cuts spill, costs hard recall (0.82 -> 0.55 at 0.3, suite_v1)'),
 'latch_frames': ('quality-weighted positive frames before a cell latches', 'rule 2 knob: higher = more robust to bad frames, slower outlines / latency'),
 'latch_ratio': ('fraction of judged frames a cell must be positive in', 'higher = tighter outlines, less spill; too high loses recall under heavy noise'),
 'confirm_frames': ('frames with a matched crown before publication', 'higher = fewer false trees, later publication'),
 'min_height_cells': ('latched cells with a fused height (as many sampled from non-truncated crowns) before publication (rule 3)', 'higher = more robust first height, later publication'),
 'min_publish_area': ('m^2 of latched cells before publication', 'higher = fewer fragments / centre drift, later publication, misses small trees'),
 'min_tree_height': ('m above ground, lower crowns never seed', 'higher rejects bushes/clutter, misses young trees'),
 'tentative_timeout': ('s, unconfirmed instances unseen this long are deleted', 'longer keeps slow-building trees, keeps noise seeds alive'),
 'cell_weight_cap': ('saturation of a cell fused-height weight', 'lower adapts faster; higher averages more; the cap is what stops hover inflation'),
 'outlier_gate_m': ('m, frame height this far from an established cell is rejected', 'lower rejects more noise, slower to fix a wrong early value (3 rejections re-initialise)'),
 'gate_min_weight': ('weight after which the outlier gate applies', 'lower gates earlier'),
 'height_quantile': ('published height = this quantile of the latched fused surface', 'descent reference: higher = closer to the true top (safe direction), more upward noise'),
 'max_height_rate': ('m/s a published height may move', 'lower = no jumps, slower to converge after a merge'),
 'max_centre_rate': ('m/s a published centre may move (outline not delayed)', 'lower = smoother centre, lags the outline'),
 'centre_from_support': ('published centre = centroid of the cells with observation-weighted canopy support (true) or of the latched cells (false, M4)', 'true lets near views correct a centre a far edge view biased (latching is permanent); false keeps the centre inside the latched outline'),
 'centre_support_ratio': ('a cell supports the centre while its weighted canopy evidence is >= this share of its weighted evidence (and >= latch_frames weighted frames)', 'lower follows the newest good views sooner, admits more spill; higher (0.75) keeps more of a far view ghost band'),
 'merge_overlap': ('merge when latched overlap >= this x the smaller latched area', 'lower merges fragments readily, risks merging neighbours'),
 'merge_centre_dist': ('m, or merge when centres are closer', 'higher merges real-crown fragments, risks merging tight neighbours'),
 'post_spray_hold_radius': ('m, after SPRAYING non-targets stay frozen until the drone is this far from the hover point', 'larger = longer protection on the climb-out'),
}
sim_g, sim = read(ROOT + '/config/cand_obj.cfg')
real_g, real = read(ROOT + '/config/cand_obj_real.cfg')
miss = [k for k in sim if k not in D]
if miss: sys.exit('missing descriptions: %s' % miss)
out = ['| group | key | sim profile (`cand_obj.cfg`) | real profile (`cand_obj_real.cfg`) | meaning | trade-off |', '|---|---|---|---|---|---|']
for g, keys in sim_g:
    for k in keys:
        rv = 'n/a (pending)' if real is None else real.get(k, sim[k])
        if real is not None and rv != sim[k]: rv = '**%s**' % rv
        out.append('| %s | `%s` | %s | %s | %s | %s |' % (g, k, sim[k], rv, D[k][0], D[k][1]))
if real is not None:
    extra = [k for k in real if k not in sim]
    for k in extra: out.append('| (real only) | `%s` | - | %s | see config | |' % (k, real[k]))
print('\n'.join(out))
print('\n%d keys; %s' % (len(sim), 'real profile absent' if real is None else '%d keys differ in the real profile' % sum(1 for k in sim if real.get(k, sim[k]) != sim[k])))
