#!/usr/bin/env python3
"""FK sanity check against the canonical URDF.

Reads the URDF and computes the world position of tool0 at q=0 plus a
few sample joint configurations. We use this to size the cabin mount
height for the hanging integration.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np


URDF = "/home/student/v33_ws/ros_ws/src/lexium_cobot_urdfbueno/urdf/lexium_cobot_l03s.urdf"


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(r, p, y):
    return Rz(y) @ Ry(p) @ Rx(r)


def parse_chain(urdf_path):
    t = ET.parse(urdf_path).getroot()
    joints = []
    for j in t.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(v) for v in (ax.get("xyz") if ax is not None else "0 0 0").split()]
        joints.append({
            "name": j.get("name"),
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
        })
    return joints


def fk_to_tool0(joints, q):
    """q is dict name->theta for revolute joints."""
    # Build link transforms by walking from base_link to tool0
    # The chain is linear: base_link -> link1_shoulder -> ... -> tool0
    T = np.eye(4)
    for j in joints:
        # transform parent->child
        R_origin = rpy_to_R(*j["rpy"])
        Tj = np.eye(4)
        Tj[:3, :3] = R_origin
        Tj[:3, 3] = j["xyz"]
        if j["type"] == "revolute":
            theta = q.get(j["name"], 0.0)
            axis = np.array(j["axis"], dtype=float)
            axis = axis / np.linalg.norm(axis)
            # Rodrigues
            K = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])
            R_rot = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)
            Tj[:3, :3] = Tj[:3, :3] @ R_rot
        T = T @ Tj
    return T


def main():
    joints = parse_chain(URDF)
    q0 = {j["name"]: 0.0 for j in joints}
    T0 = fk_to_tool0(joints, q0)
    print("tool0 @ q=0:")
    print("  pos =", T0[:3, 3])
    print("  R   =", T0[:3, :3].round(4).tolist())
    # furthest extent at random poses to estimate reach
    rng = np.random.default_rng(7)
    rs = []
    for _ in range(2000):
        q = {}
        for j in joints:
            if j["type"] != "revolute":
                continue
            # respect a soft +/-1 rad sample so all joints play
            q[j["name"]] = rng.uniform(-1.5, 1.5)
        T = fk_to_tool0(joints, q)
        rs.append(np.linalg.norm(T[:3, 3]))
    rs = np.array(rs)
    print(f"sampled tool0 distance from base_link: min={rs.min():.4f} m, max={rs.max():.4f} m")
    print(f"  median={np.median(rs):.4f} m, mean={rs.mean():.4f} m")


if __name__ == "__main__":
    main()
