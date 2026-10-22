"""Robot description served to the Foxglove 3D panel."""

import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path
from urllib.request import urlopen

import pytest

from unitree_foxglove.assets import serve_directory
from unitree_foxglove.bridge import asset_handler, start_asset_server
from unitree_foxglove.robots import G1, GO2

RL_GYM_ROOT = Path(os.environ.get("UNITREE_RL_GYM_ROOT", "third_party/unitree_rl_gym"))


def test_g1_urdf_joints_match_the_joint_state_names() -> None:
    urdf = G1.description_path(RL_GYM_ROOT) / G1.urdf_file  # type: ignore[operator]
    if not urdf.is_file():
        pytest.skip("unitree_rl_gym submodule not checked out")
    root = ElementTree.parse(urdf).getroot()
    movable = [joint.get("name") for joint in root.iter("joint") if joint.get("type") != "fixed"]
    assert movable == list(G1.joint_names)
    # Every mesh is relative to the URDF, so one directory serves the whole model.
    meshes = {mesh.get("filename") for mesh in root.iter("mesh")}
    assert all(name.startswith("meshes/") for name in meshes if name)
    assert all((urdf.parent / name).is_file() for name in meshes if name)
    assert G1.base_frame == root.find("link").get("name")  # type: ignore[union-attr]


def test_gateway_serves_the_g1_model_as_the_remote_layout_names_it() -> None:
    directory = G1.description_path(RL_GYM_ROOT)
    if directory is None or not (directory / "g1_29dof.urdf").is_file():
        pytest.skip("unitree_rl_gym submodule not checked out")
    fetch = asset_handler(directory)
    urdf = fetch("package://g1_description/g1_29dof.urdf")
    assert urdf is not None
    mesh = ElementTree.fromstring(urdf).find(".//mesh").get("filename")  # type: ignore[union-attr]
    assert fetch(f"package://g1_description/{mesh}") is not None


def test_directory_is_served_with_cors_for_the_web_app(tmp_path: Path) -> None:
    (tmp_path / "robot.urdf").write_text("<robot name='x'/>")
    server = serve_directory(tmp_path, "127.0.0.1", 0)
    try:
        port = server.server_address[1]
        with urlopen(f"http://127.0.0.1:{port}/robot.urdf", timeout=5) as response:
            assert response.status == 200
            assert response.headers["Access-Control-Allow-Origin"] == "*"
            assert response.read() == b"<robot name='x'/>"
    finally:
        server.shutdown()


def test_robots_without_a_description_get_no_server(tmp_path: Path) -> None:
    assert start_asset_server(GO2, "127.0.0.1", 0, tmp_path) is None
    assert start_asset_server(G1, "127.0.0.1", 0, tmp_path) is None
