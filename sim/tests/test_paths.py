from isaac import paths


def test_collider_path_roundtrip():
    for tid in (1, 5, 96, 12345):
        assert paths.tree_id_from_collider_path(paths.canopy_collider_path(tid)) == tid


def test_non_canopy_paths_map_to_none():
    for p in ("", None, "/World/Ground", "/World/Orchard/tree_3/asset/Meshes/bark",
              "/World/Colliders", "/World/Colliders/tree_x/hull", "/World/Collidersx/tree_3/hull",
              "/World/Colliders/tree_/hull"):
        assert paths.tree_id_from_collider_path(p) is None


def test_ids_are_not_prefix_confused():
    assert paths.tree_id_from_collider_path("/World/Colliders/tree_12/hull") == 12
    assert paths.tree_id_from_collider_path("/World/Colliders/tree_1/hull") == 1
