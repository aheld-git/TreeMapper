"""Isaac-only: build the orchard stage (ground, lights, referenced trees) and the canopy colliders.

Imports Isaac (pxr) and scipy; everything that can be tested without Isaac lives in layout.py / truth.py.
Stage conventions: z up, metres. The tree assets are Y-up and in centimetres, so every tree gets
``T(x, y, z_ground) Rz(yaw) Rx(90 deg) S(0.01 * scale)``.

Prim layout
    /World/Ground                          sloped ground mesh over ``field.polygon`` (grass texture with clutter.grass)
    /World/Orchard/tree_<id>               Xform with the placement
    /World/Orchard/tree_<id>/asset         instanceable reference to Orange_Tree.usd / Lemon_Tree.usd
    /World/Colliders/tree_<id>/hull        invisible convex-hull collider of the canopy (collision group ``canopy``)
"""
import math
import os
from typing import Any, Dict, List, Tuple

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

from isaac import ground_texture, layout, truth
from isaac.paths import COLLIDER_ROOT, COLLISION_GROUP, canopy_collider_path  # noqa: F401  (re-exported)

SPECIES_FILE = {"orange": "Orange_Tree.usd", "lemon": "Lemon_Tree.usd"}
ASSET_UP_AXIS_ROT = math.radians(90.0)   # asset Y-up -> world Z-up
ASSET_UNIT_TO_M = 0.01                   # the assets are in centimetres
CANOPY_BASE_FRACTION = 0.25              # no trunk-named mesh: canopy = points above 25 % of the tree height
GROUND_RGB = (0.30, 0.42, 0.18)


def placement_matrix(p: layout.Placement) -> np.ndarray:
    """Column-vector 4x4 asset -> world matrix of a placement."""
    c, s = math.cos(p.yaw), math.sin(p.yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    c, s = math.cos(ASSET_UP_AXIS_ROT), math.sin(ASSET_UP_AXIS_ROT)
    Rx = np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])
    M = np.eye(4)
    M[:3, :3] = Rz @ Rx * (ASSET_UNIT_TO_M * p.scale)
    M[:3, 3] = [p.x, p.y, p.z_ground]
    return M


def _gf_matrix(M: np.ndarray) -> Gf.Matrix4d:
    """Column-vector numpy 4x4 -> USD (row-vector) matrix."""
    return Gf.Matrix4d(*[float(v) for v in M.T.reshape(16)])


def _np_matrix(m: Gf.Matrix4d) -> np.ndarray:
    """USD (row-vector) matrix -> numpy array in the same row-vector layout (points @ M)."""
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=float)


def _material(stage: Usd.Stage, path: str, rgb: Tuple[float, float, float], roughness: float = 1.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _textured_material(stage: Usd.Stage, path: str, texture_file: str, roughness: float = 1.0):
    """UsdPreviewSurface whose diffuse colour is ``texture_file``, read through the mesh's ``st`` primvar (repeat)."""
    mat = UsdShade.Material.Define(stage, path)
    st = UsdShade.Shader.Define(stage, path + "/st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, path + "/diffuse_texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_file)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh = UsdShade.Shader.Define(stage, path + "/shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _build_ground(stage: Usd.Stage, spec: Dict[str, Any], texture_dir: str = "") -> None:
    poly = [(float(x), float(y)) for x, y in spec["field"]["polygon"]]
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    pts = [(cx, cy, layout.ground_z(spec, cx, cy))] + [(x, y, layout.ground_z(spec, x, y)) for x, y in poly]
    n = len(poly)
    idx = []
    for i in range(n):   # fan from the centroid (field polygons are convex)
        idx += [0, 1 + i, 1 + (i + 1) % n]
    mesh = UsdGeom.Mesh.Define(stage, "/World/Ground")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * n))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    rgb = tuple(float(c) for c in spec["terrain"].get("color", GROUND_RGB))   # optional spec key terrain.color
    mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*rgb)]))
    if (spec.get("clutter") or {}).get("grass"):
        # grass = a band-limited brightness texture on the terrain colour (ground_texture.py: why)
        if not texture_dir:
            raise ValueError("clutter.grass needs a directory for the ground texture")
        from PIL import Image
        tex_file = os.path.join(texture_dir, "ground_texture.png")
        Image.fromarray(ground_texture.grass_texture(rgb, seed=int(spec.get("seed", 0)))).save(tex_file)
        uv = ground_texture.ground_uv([p[:2] for p in pts])
        pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                               UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(float(u), float(v)) for u, v in uv]))
        mat = _textured_material(stage, "/World/Looks/ground", tex_file)
    else:
        mat = _material(stage, "/World/Looks/ground", rgb)
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)


def _build_lights(stage: Usd.Stage) -> None:
    sun = UsdLux.DistantLight.Define(stage, "/World/Lights/sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-55.0, 0.0, 30.0))
    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(700.0)


def _mesh_world_points(root: Usd.Prim) -> np.ndarray:
    """World-space points of every UsdGeom.Mesh below ``root`` (instance proxies included)."""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    chunks = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if pts is None or len(pts) == 0:
            continue
        M = _np_matrix(cache.GetLocalToWorldTransform(prim))
        p = np.asarray(pts, dtype=float).reshape(-1, 3)
        chunks.append(p @ M[:3, :3] + M[3, :3])
    if not chunks:
        raise RuntimeError("no mesh found below %s" % root.GetPath())
    return np.concatenate(chunks, axis=0)


def _canopy_points(root: Usd.Prim, z_ground: float) -> np.ndarray:
    """Canopy points in world space.

    Meshes whose name contains "trunk" (case-insensitive) are not canopy. The Orange / Lemon assets have
    no such split (bark and twigs are separate meshes but the foliage meshes overlap them), so the
    fallback applies: every point above ``z_ground + 0.25 * (top - z_ground)``.
    """
    names = [p.GetName().lower() for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
    if any("trunk" in n for n in names):
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        chunks = []
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if prim.IsA(UsdGeom.Mesh) and "trunk" not in prim.GetName().lower():
                M = _np_matrix(cache.GetLocalToWorldTransform(prim))
                p = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float).reshape(-1, 3)
                chunks.append(p @ M[:3, :3] + M[3, :3])
        return np.concatenate(chunks, axis=0)
    pts = _mesh_world_points(root)
    top = float(pts[:, 2].max())
    keep = pts[:, 2] >= z_ground + CANOPY_BASE_FRACTION * (top - z_ground)
    if keep.sum() < 4:
        raise RuntimeError("canopy extraction left %d points" % int(keep.sum()))
    return pts[keep]


def _add_collider(stage: Usd.Stage, tree_id: int, canopy_world: np.ndarray) -> None:
    """Invisible convex-hull collider of the canopy, in collision group ``canopy``."""
    from scipy.spatial import ConvexHull
    hull = ConvexHull(canopy_world)
    verts = hull.vertices
    remap = -np.ones(len(canopy_world), dtype=int)
    remap[verts] = np.arange(len(verts))
    faces = remap[hull.simplices]
    UsdGeom.Xform.Define(stage, "%s/tree_%d" % (COLLIDER_ROOT, tree_id))
    path = canopy_collider_path(tree_id)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, canopy_world[v])) for v in verts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for i in faces.reshape(-1)]))
    mesh.CreateSubdivisionSchemeAttr("none")
    prim = mesh.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    mca = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mca.CreateApproximationAttr("convexHull")
    hull_api = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
    hull_api.CreateHullVertexLimitAttr(255)
    UsdGeom.Imageable(prim).MakeInvisible()   # never rendered; PhysX scene queries ignore visibility
    UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)


def build_scene(stage: Usd.Stage, spec: Dict[str, Any], assets_dir: str,
                texture_dir: str = "") -> List[Dict[str, Any]]:
    """Build the whole orchard on ``stage`` and return the per-tree ground truth (``truth.tree_truth``).
    ``texture_dir`` receives the generated ground texture (needed when the spec asks for ``clutter.grass``).

    Raises on any problem (missing asset, unknown species, degenerate canopy): the caller must not run a
    half-built scene.
    """
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    placements = layout.plan_layout(spec)
    for sp in {p.species for p in placements}:
        if sp not in SPECIES_FILE:
            raise ValueError("unknown species %r (known: %s)" % (sp, sorted(SPECIES_FILE)))
        f = os.path.join(assets_dir, SPECIES_FILE[sp])
        if not os.path.isfile(f):
            raise FileNotFoundError("tree asset missing: %s" % f)

    _build_ground(stage, spec, texture_dir)
    _build_lights(stage)
    UsdGeom.Xform.Define(stage, "/World/Orchard")
    UsdGeom.Xform.Define(stage, COLLIDER_ROOT)

    group = UsdPhysics.CollisionGroup.Define(stage, COLLISION_GROUP)
    includes = group.GetCollidersCollectionAPI().CreateIncludesRel()

    trees = []   # type: List[Dict[str, Any]]
    for p in placements:
        tpath = "/World/Orchard/tree_%d" % p.id
        xf = UsdGeom.Xform.Define(stage, tpath)
        xf.AddTransformOp().Set(_gf_matrix(placement_matrix(p)))
        asset = UsdGeom.Xform.Define(stage, tpath + "/asset").GetPrim()
        asset.GetReferences().AddReference(os.path.join(assets_dir, SPECIES_FILE[p.species]))
        asset.SetInstanceable(True)
        canopy = _canopy_points(asset, p.z_ground)
        trees.append(truth.tree_truth(p, canopy))
        _add_collider(stage, p.id, canopy)
        includes.AddTarget(Sdf.Path(canopy_collider_path(p.id)))
    return trees
