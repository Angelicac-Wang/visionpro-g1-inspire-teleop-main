#!/usr/bin/env python3
"""Build the SONIC G1 scene with pinned Unitree Inspire FTP hand assets."""

from __future__ import annotations

import copy
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

import mujoco

REVISION = "5994d4faef0a9cadd3287f8de0199a67eeb2a259"
SOURCE = f"https://raw.githubusercontent.com/unitreerobotics/unitree_ros/{REVISION}"
REPO = Path(__file__).resolve().parents[1]
CACHE = REPO / "assets/mujoco/inspire_ftp"
OUTPUT = REPO / "assets/mujoco/g1_runtime"


def fetch(relative: str, destination: Path) -> None:
    if destination.is_file():
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(f"{SOURCE}/{relative}", timeout=60) as response:
        content = response.read()
    temporary = destination.with_suffix(destination.suffix + ".part")
    temporary.write_bytes(content)
    temporary.replace(destination)


def setup() -> Path:
    groot = Path(os.environ["GR00T_ROOT"])
    original = groot / "gear_sonic/data/robot_model/model_data/g1"
    OUTPUT.mkdir(parents=True, exist_ok=True)
    CACHE.mkdir(parents=True, exist_ok=True)
    fetch("LICENSE", CACHE / "LICENSE")
    for side in ("left", "right"):
        fetch(f"robots/g1_description/inspire_hand/FTP_{side}_hand.urdf", CACHE / f"{side}.urdf")
    fetch("robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf", CACHE / "g1.urdf")
    mesh_names = set()
    for side in ("left", "right"):
        hand = ET.parse(CACHE / f"{side}.urdf").getroot()
        mesh_names.update(Path(mesh.get("filename")).name for mesh in hand.findall(".//mesh"))
    with ThreadPoolExecutor(max_workers=6) as pool:
        list(pool.map(lambda name: fetch(f"robots/g1_description/meshes/{name}", CACHE / "meshes" / name), sorted(mesh_names)))

    root = ET.parse(original / "g1_29dof_with_hand.xml").getroot()
    root.set("model", "g1_29dof_with_inspire_hand_ftp")
    root.find("compiler").set("meshdir", str(original / "meshes"))
    asset = root.find("asset")
    # Absolute mesh paths also work when a scene includes this generated model.
    for mesh in asset.findall("mesh"):
        mesh.set("file", str(original / "meshes" / mesh.get("file")))
    original_urdf = ET.parse(CACHE / "g1.urdf").getroot()
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")

    for side in ("left", "right"):
        hand = ET.parse(CACHE / f"{side}.urdf").getroot()
        compiler = hand.find("mujoco/compiler")
        compiler.set("meshdir", str(CACHE / "meshes"))
        compiler.set("fusestatic", "false")
        for mesh in hand.findall(".//mesh"):
            mesh.set("filename", Path(mesh.get("filename")).name)
        converted_input = CACHE / f"{side}_convert.urdf"
        ET.ElementTree(hand).write(converted_input)
        model = mujoco.MjModel.from_xml_path(str(converted_input))
        converted_path = CACHE / f"{side}.xml"
        mujoco.mj_saveLastXML(str(converted_path), model)
        converted = ET.parse(converted_path).getroot()
        for mesh in converted.findall("asset/mesh"):
            mesh.set("file", str(CACHE / "meshes" / Path(mesh.get("file")).name))
            asset.append(copy.deepcopy(mesh))

        wrist = root.find(f".//body[@name='{side}_wrist_yaw_link']")
        for child in list(wrist):
            if child.tag == "body" and child.get("name", "").startswith(f"{side}_hand_"):
                wrist.remove(child)
            elif child.tag == "geom" and "hand_palm" in child.get("mesh", ""):
                wrist.remove(child)
        # The old wrist inertia includes the Dex3 palm. Use the bare wrist
        # inertia from the official G1+Inspire URDF before adding the new palm.
        inertia = original_urdf.find(f"link[@name='{side}_wrist_yaw_link']/inertial")
        old_inertia = wrist.find("inertial")
        wrist.remove(old_inertia)
        tensor = inertia.find("inertia")
        ET.SubElement(wrist, "inertial", {
            "pos": inertia.find("origin").get("xyz"),
            "mass": inertia.find("mass").get("value"),
            "fullinertia": " ".join(tensor.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
        })
        imported_wrist = converted.find(f"worldbody/body[@name='{side}_wrist_yaw_link']")
        if imported_wrist is None:
            raise RuntimeError(f"Imported {side} hand lost its wrist mount")
        for body in imported_wrist.findall("body"):
            body = copy.deepcopy(body)
            for geom in body.findall(".//geom"):
                if geom.get("contype", "1") != "0":
                    geom.set("contype", "2" if side == "left" else "4")
                    geom.set("conaffinity", "5" if side == "left" else "3")
                    geom.set("group", "3")  # Hide duplicate collision meshes in the viewer.
            for joint in body.findall(".//joint"):
                joint.set("name", joint.get("name").replace(f"{side}_", f"{side}_hand_", 1))
                joint.set("damping", "0.03")
                joint.set("armature", "0.002")
                joint.set("frictionloss", "0.01")
            wrist.append(body)
        # URDF mimic relationships are not imported by MuJoCo automatically.
        for joint in hand.findall("joint"):
            mimic = joint.find("mimic")
            if mimic is not None:
                ET.SubElement(equality, "joint", {
                    "joint1": joint.get("name").replace(f"{side}_", f"{side}_hand_", 1),
                    "joint2": mimic.get("joint").replace(f"{side}_", f"{side}_hand_", 1),
                    "polycoef": f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
                })

    # Keep one torque actuator per hinge, in joint traversal order, as required
    # by SONIC's body torque/state indexing. Mimic joints get zero external torque.
    actuator = root.find("actuator")
    sensors = root.find("sensor")
    if sensors is not None:
        for sensor in list(sensors):
            if "_hand_" in sensor.get("joint", sensor.get("objname", "")):
                sensors.remove(sensor)
    body_motors = {motor.get("joint"): copy.deepcopy(motor) for motor in actuator if "hand" not in motor.get("joint", "")}
    actuator.clear()
    for joint in root.findall("worldbody/.//joint"):
        name = joint.get("name")
        if name in body_motors:
            actuator.append(body_motors[name])
        elif "_hand_" in name:
            ET.SubElement(actuator, "motor", name=name.removesuffix("_joint"), joint=name, ctrlrange="-1.4 1.4")
    model_path = OUTPUT / "g1_29dof_with_inspire_hand.xml"
    ET.indent(root)
    ET.ElementTree(root).write(model_path, encoding="unicode")
    scene = ET.parse(original / "scene_43dof.xml").getroot()
    scene.set("model", "g1_inspire_ftp_scene")
    scene.find("include").set("file", model_path.name)
    scene_path = OUTPUT / "scene_inspire_hand.xml"
    ET.indent(scene)
    ET.ElementTree(scene).write(scene_path, encoding="unicode")
    validated = mujoco.MjModel.from_xml_path(str(scene_path))
    if validated.nu != 53:
        raise RuntimeError(f"Expected 29 body + 24 hand actuators, got {validated.nu}")
    print(f"Inspire FTP scene ready: {scene_path} ({validated.nu} actuators)")
    return scene_path


if __name__ == "__main__":
    setup()
