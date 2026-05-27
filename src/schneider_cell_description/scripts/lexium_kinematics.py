"""V51 Lexium L03S kinematics (URDF debug as golden reference).

V51 deltas vs V50:
  * The lateral grasp is no longer described by a hand-tuned scalar
    `LATERAL_GRASP_OFFSET` and a `PICK_CONVEYOR_TARGET_DX_WORLD`
    correction; both were the result of geometric estimation that
    fought RViz reality.  V51 takes the relationship directly from
    the user-supplied URDF debug ("golden reference"):

        gripper_base -> tcp_link              = (+0.000250, +0.060250, +0.076750)
        gripper_base -> cafi_lateral_target   = (+0.000000, +0.016600, +0.076750)
        gripper_base -> fixed_jaw_inner       = (+0.000000, -0.027050, +0.076750)
        gripper_base -> appendage_inner       = (+0.000250, +0.060250, +0.076750) at q=0

    From these (all in gripper_base local frame), the "lateral grasp
    delta" between TCP and CAFI center is:

        LATERAL_GRASP_DELTA = TCP - CAFI_target
                            = (+0.000250, +0.043650, +0.000000)

    i.e. TCP sits +43.65 mm along gripper local +Y from the CAFI
    centre, with a negligible +0.25 mm in gripper local +X.  The
    IK target for any lateral grasp pose is therefore

        TCP_world = CAFI_world + R_gripper_world * LATERAL_GRASP_DELTA

    where R_gripper_world is the gripper's world rotation at the IK
    solution (top-down for all current poses).  This replaces the
    old `p_center - L * y_world` formulation and removes the need
    for an ad-hoc PICK_CONVEYOR DX correction (set to 0 in V51).

  * The cobot URDF (`lexium_cobot_with_final_gripper.urdf`) gains the
    debug frames (gripper_grasp_center_frame, fixed_jaw_inner_contact
    _frame, appendage_inner_contact_frame, cafi_lateral_target_frame,
    grasp_volume_frame, debug_cafi_in_gripper_link) so RViz shows the
    golden reference.  NO change to the active gripper mechanism:
    appendage_link, appendage_prismatic_joint (axis 0 1 0, limit
    [0, 0.028]), tcp_link offset, tool0->gripper_base, mesh files
    all byte-for-byte from V48/V49/V50.
  * Cobot URDF (`lexium_cobot_with_final_gripper`) preserved
    byte-for-byte from V48.  `appendage_prismatic_joint` axis (0 1 0),
    limit [0, 0.028], tcp_link offset (0.00025, 0.06025, 0.07675),
    tool0->gripper_base (0, -0.07, 0.015) all untouched.

V48 deltas vs V46:
  * Gripper upper limit raised from 0.015 m to 0.028 m at the user's
    request.  GRIPPER_JAW_STROKE updated to 0.028 m to match.
  * LATERAL_GRASP_OFFSET raised from 0.082 m to 0.087 m so the moving
    appendage_link contacts the CAFI's lateral face from its INTERNAL
    side (the -Y face of the appendage in its own frame) WITHOUT
    penetrating the CAFI.  V46 had a 3.75 mm penetration: at +0.082 m
    TCP offset the appendage internal face (Y=+0.036 in appendage
    frame) projected to +0.05775 m from CAFI centre, 3.75 mm INSIDE
    the +0.0615 m east face of the CAFI.  V48 with +0.087 m places
    the internal face at +0.06275 m, 1.25 mm OUTSIDE the east face
    (touching, not penetrating).  External face at +0.08275 m, well
    clear.
  * spawn_q kept rpy(0, pi, pi) byte-for-byte from V46 (user rule:
    "no cambies orientacion del CAFI").
  * Cobot URDF (lexium_cobot_with_final_gripper) integrated as in V43.
    Joint kinematics, JOINT2_RPY=(0,0,0.05), mount Rx(+pi/2) all
    preserved.  The only joint-limit change is the prismatic gripper.
  * GRASP_CENTER_OFFSET (in tool0 frame, at jaw closed q=0):
        tool0 -> gripper_base: T(0, -0.07, 0.015)
        gripper_base -> appendage: T(0, 0, 0) at q=0
        appendage -> tcp_link:  T(0.000250, 0.060250, 0.076750)
        => TCP in tool0 = (0.000250, -0.009750, 0.091750)
    This is the user-defined working centre of the gripper (~92 mm along
    tool0 +Z, near-axial with a tiny -Y skew).  GRIPPER_YAW_MOUNT = 0.

Joint chain (byte-for-byte from the user-supplied URDF):

  base_link
     |  joint_1   rev Y    xyz (0.1623, 0.0867, 0.0645)    rpy 0
  link1_shoulder
     |  joint_2   rev Z    xyz (-0.0115, 0.0639, 0.0000)   rpy (0,0,0.05)
  link2_upper_arm
     |  joint_3   rev Z    xyz (-0.0015, 0.2450, 0.2258)   rpy 0
  link3_forearm
     |  joint_4   rev Z    xyz (-0.0060, 0.2295, 0.1244)   rpy 0
  link4_wrist1
     |  joint_5   rev Y    xyz (-0.0010, 0.0465, -0.2300)  rpy 0
  link5_wrist2
     |  joint_6   rev Z    xyz (-0.0040, 0.0720, 0.0898)   rpy 0
  link6_wrist3
     |  joint_tool0  fixed xyz (0, 0.068, 0)
  tool0
     |  tool0_to_gripper_base fixed xyz (0, -0.07, 0.015)
  gripper_base
     |  appendage_prismatic_joint  axis (0,1,0), q [0, 0.015]
  appendage_link
     |  tcp_fixed_joint            xyz (0.000250, 0.060250, 0.076750)
  tcp_link
     |  tcp_grasp_center_fixed_joint (identity alias)
  tcp_grasp_center      <-- THE WORKING CENTRE OF THE GRIPPER
"""
import math
import numpy as np

# ============================================================
# V43 CHAIN CONSTANTS (per the user-supplied URDF, byte-for-byte)
# ============================================================
JOINT1_XYZ = (0.1623, 0.0867, 0.0645)
JOINT2_XYZ = (-0.0115, 0.0639, 0.0000)
JOINT2_RPY = (0.0, 0.0, 0.05)  # V43: restored from the user URDF
# V61: URDF DEFINITIVO extracts link_elbow_connector as an independent
# link between link2_upper_arm and link3_forearm.  The fixed
# joint_elbow_connector carries the (0, pi, 0) Y-flip of the original
# pivot.  joint_3 then hangs off link_elbow_connector and its origin
# is expressed in the elbow_connector frame (post Ry(pi)).
ELBOW_CONNECTOR_XYZ = (0.001060, 0.243625, 0.119967)
ELBOW_CONNECTOR_RPY = (0.0, math.pi, 0.0)
JOINT3_XYZ = (0.002560, 0.001375, 0.113592)
JOINT4_XYZ = (-0.0060, 0.2295, 0.1244)
JOINT5_XYZ = (-0.0010, 0.0465, -0.2300)
JOINT6_XYZ = (-0.0040, 0.0720, 0.0898)
TOOL0_XYZ  = (0.0000, 0.0680, 0.0000)

# Joint hard limits (canonical URDF; <limit lower upper>)
JOINT_LIMITS = [
    (-3.14159, +3.14159),  # J1  +/-180 deg
    (-2.61799, +2.61799),  # J2  +/-150 deg
    (-2.61799, +2.61799),  # J3  +/-150 deg
    (-3.14159, +3.14159),  # J4  +/-180 deg
    (-2.09440, +2.09440),  # J5  +/-120 deg
    (-3.14159, +3.14159),  # J6  +/-180 deg
]

# ============================================================
# V43 GRIPPER (integrated, tcp_link alias = tcp_grasp_center)
# ============================================================
# tool0 -> tcp_grasp_center is a PURE translation in tool0 frame.
#   tool0 -> gripper_base: T(0, -0.07, 0.015)
#   gripper_base -> appendage_link: T(0, 0, 0) at q=0 (jaw closed)
#   appendage_link -> tcp_link: T(0.000250, 0.060250, 0.076750)
# Sum: (0.000250, -0.009750, 0.091750).  No yaw, no roll, no pitch.
GRIPPER_YAW_MOUNT  = 0.0
GRASP_CENTER_OFFSET = (0.000250, -0.009750, 0.091750)

# Reference helper: if the planning pipeline ever needs to account for an
# open jaw, the prismatic q just adds to the Y component of the TCP
# offset (axis is +Y).  At runtime the IK seeds use q=0 (closed); jaw
# motion only happens during the close/open beats while the cobot is at
# PICK/PLACE.
#
# V48: stroke raised to 0.028 m (URDF upper limit) so the open jaw
# leaves 28 mm clearance from the CAFI east face instead of 15 mm —
# more margin during approach without changing the closed grasp.
GRIPPER_JAW_STROKE = 0.028  # full open stroke, m (V48: was 0.015 in V46)

# V48 lateral grasp offset — INTERNAL-face contact, no penetration.
#
#   appendage.stl bbox in appendage_link frame (after URDF Rx(-pi/2)
#   and scale 0.001):
#       Y in [+0.0503, +0.0702]  (gripper closing axis +Y)
#       X in [-0.0095, +0.010]
#       Z in [+0.012,  +0.0767]
#   TCP rigidly attached at appendage frame (+0.00025, +0.06025,
#   +0.07675) — the TCP sits at Y=+0.06025, INSIDE the appendage Y
#   span.
#   IK convention (resolve_poses.py): TCP_world = CAFI_world -
#   LATERAL_GRASP_OFFSET * y_world.  At PICK_CONVEYOR y_world points
#   along world -X, so TCP ends up on the world +X (east) side of
#   the CAFI by +LATERAL_GRASP_OFFSET m.  Because gripper +Y in
#   world == -X here, the CAFI is at LARGER appendage Y than TCP.
#   The face of the appendage CLOSEST to the CAFI is therefore the
#   Y_MAX face (+0.0702) — THIS is the "internal" face that should
#   touch the CAFI.
#
#   For the Y_MAX face to land on the CAFI east face (at +0.0615 m
#   from CAFI centre along gripper +Y):
#       CAFI_Y_in_app = Y_MAX + 0.0615 = +0.1317
#       TCP_Y_in_app  = +0.06025
#       (TCP - CAFI) . gripper_Y = TCP_Y - CAFI_Y = -0.07145 m
#       LATERAL_GRASP_OFFSET = -(TCP - CAFI) . gripper_Y = +0.07145 m
#   Add a 1.25 mm cosmetic clearance so the contact is "touching"
#   rather than coincident (avoids RViz Z-fight + sim contact jitter):
# V48 legacy scalar -- kept for any module that still imports it.
# In V51 the lateral grasp is described by LATERAL_GRASP_DELTA below.
LATERAL_GRASP_OFFSET = 0.0727  # V48 legacy -- not used by V51 IK
# Sanity:
#   At L = 0.0727, IK target TCP = CAFI + 0.0727 along world +X.
#       Appendage Y_MAX face world X = TCP_x - 0.00995 = CAFI_x +
#           0.0727 - 0.00995 = CAFI_x + 0.06275.
#       CAFI east face = CAFI_x + 0.0615.
#       Gap (appendage internal face to CAFI east face) = 0.06275 -
#           0.0615 = +0.00125 m = +1.25 mm OUTSIDE -- CONTACT, NO
#           PENETRATION.
#   At q=0.028 open, the appendage retracts +28 mm along +Y, so the
#   internal face sits at CAFI_x + 0.06275 + 0.028 = CAFI_x +
#   0.09075 m (≈ +29 mm clear of the east face) — ample margin
#   during approach.

# V51: PICK_CONVEYOR target X correction set back to ZERO.
# The new LATERAL_GRASP_DELTA captures the gripper-to-CAFI offset
# directly from the URDF debug; no ad-hoc world-X shift is needed.
PICK_CONVEYOR_TARGET_DX_WORLD = 0.0  # V51: superseded by LATERAL_GRASP_DELTA

# V51 golden reference, extracted byte-for-byte from the user-supplied
# URDF debug (gripper_base local frame, q=0 / jaw closed):
TCP_IN_GRIPPER_BASE             = (0.000250, 0.060250, 0.076750)
CAFI_LATERAL_TARGET_IN_GRIPPER  = (0.000000, 0.016600, 0.076750)
GRIPPER_GRASP_CENTER_IN_GRIPPER = (0.000000, 0.016600, 0.076750)  # alias
FIXED_JAW_INNER_IN_GRIPPER      = (0.000000, -0.027050, 0.076750)
APPENDAGE_INNER_IN_GRIPPER      = (0.000250, 0.060250, 0.076750)  # at q=0

# LATERAL_GRASP_DELTA = TCP - CAFI_lateral_target in gripper_base
# local frame.  This is the vector that, projected to world through
# R_gripper_world, places the runtime CAFI centre at the URDF-debug
# cafi_lateral_target_frame relative to the gripper.
LATERAL_GRASP_DELTA = (
    TCP_IN_GRIPPER_BASE[0] - CAFI_LATERAL_TARGET_IN_GRIPPER[0],  # +0.000250
    TCP_IN_GRIPPER_BASE[1] - CAFI_LATERAL_TARGET_IN_GRIPPER[1],  # +0.043650
    TCP_IN_GRIPPER_BASE[2] - CAFI_LATERAL_TARGET_IN_GRIPPER[2],  # +0.000000
)
# Applied AFTER the lateral-shift computation (p_center -
# LATERAL_GRASP_OFFSET * y_world) as an additive world-frame
# correction.  See resolve_poses.py for the call site.  Applies to
# APPROACH_CONVEYOR, PICK_CONVEYOR, LIFT_CONVEYOR -- all three stack
# above the same CAFI XY centre.  No other pose is affected.


# ============================================================
# V38 WORLD MOUNT (cobot bolted directly to the mesa)
# V44: layout unchanged for the cobot anchor; only bins / vision / cabin
# shifted (see schneider_cell.urdf.xacro V44 header).
# ============================================================
WORLD_COBOT_XY  = (1.152, 1.049)
WORLD_COBOT_Z   = 1.000
WORLD_COBOT_RPY = (+math.pi / 2.0, 0.0, 0.0)

# Real HOME of the planta (J4=+90, J5=-90; all others 0).
POSE_HOME_Q = [0.0, 0.0, 0.0, +math.pi / 2.0, -math.pi / 2.0, 0.0]


# ============================================================
# Math helpers
# ============================================================
def Rx(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[1,1]=c; M[1,2]=-s; M[2,1]=s; M[2,2]=c
    return M

def Ry(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[0,0]=c; M[0,2]=s; M[2,0]=-s; M[2,2]=c
    return M

def Rz(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4); M[0,0]=c; M[0,1]=-s; M[1,0]=s; M[1,1]=c
    return M

def T(x, y, z):
    M = np.eye(4); M[0,3]=x; M[1,3]=y; M[2,3]=z
    return M

def Trpy(xyz, rpy):
    """URDF convention rpy: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
    r, p, y = rpy
    M = T(*xyz) @ Rz(y) @ Ry(p) @ Rx(r)
    return M


# ============================================================
# FK
# ============================================================
def fk_base_to_tool0(q):
    """FK from base_link frame to tool0.  Returns 4x4 transform.

    V61: inserts the fixed joint_elbow_connector (Ry(pi) at
    ELBOW_CONNECTOR_XYZ) between Rz(q[1]) and the joint_3 translation,
    matching the URDF DEFINITIVO chain.  Downstream chain (joint_4 ...
    tool0) is byte-for-byte the old chain.
    """
    M = np.eye(4)
    M = M @ T(*JOINT1_XYZ) @ Ry(q[0])
    M = M @ Trpy(JOINT2_XYZ, JOINT2_RPY) @ Rz(q[1])
    M = M @ Trpy(ELBOW_CONNECTOR_XYZ, ELBOW_CONNECTOR_RPY)
    M = M @ T(*JOINT3_XYZ) @ Rz(q[2])
    M = M @ T(*JOINT4_XYZ) @ Rz(q[3])
    M = M @ T(*JOINT5_XYZ) @ Ry(q[4])
    M = M @ T(*JOINT6_XYZ) @ Rz(q[5])
    M = M @ T(*TOOL0_XYZ)
    return M


def base_to_world():
    """Mount transform: base_link in world."""
    return Trpy((WORLD_COBOT_XY[0], WORLD_COBOT_XY[1], WORLD_COBOT_Z),
                WORLD_COBOT_RPY)


def fk_world_to_tool0(q):
    return base_to_world() @ fk_base_to_tool0(q)


def fk_world_to_grasp_center(q):
    """Compose to tcp_grasp_center (the working centre of the new gripper).

    V43: pure translation in tool0 frame, no extra yaw — the gripper
    joints between tool0 and tcp_link are all identity rotation.
    """
    M = fk_world_to_tool0(q)
    if GRIPPER_YAW_MOUNT != 0.0:
        M = M @ Rz(GRIPPER_YAW_MOUNT)
    M = M @ T(*GRASP_CENTER_OFFSET)
    return M


# Friendly alias so consumers can spell out the V43 frame name.
def fk_world_to_tcp_grasp_center(q):
    return fk_world_to_grasp_center(q)


def fk_joint_origins_world(q):
    """V44: return the world position of every joint origin in the cobot
    chain.  Used by the per-joint mesa clearance check (no joint may sit
    below mesa_top + safety margin).

    Returns a dict {joint_name: (x, y, z)}.
    """
    Tb = np.eye(4)
    Tj1 = Tb @ T(*JOINT1_XYZ) @ Ry(q[0])
    Tj2 = Tj1 @ Trpy(JOINT2_XYZ, JOINT2_RPY) @ Rz(q[1])
    Telbow = Tj2 @ Trpy(ELBOW_CONNECTOR_XYZ, ELBOW_CONNECTOR_RPY)
    Tj3 = Telbow @ T(*JOINT3_XYZ) @ Rz(q[2])
    Tj4 = Tj3 @ T(*JOINT4_XYZ) @ Rz(q[3])
    Tj5 = Tj4 @ T(*JOINT5_XYZ) @ Ry(q[4])
    Tj6 = Tj5 @ T(*JOINT6_XYZ) @ Rz(q[5])
    Ttool = Tj6 @ T(*TOOL0_XYZ)
    Tw = base_to_world()
    out = {}
    for name, T_local in [
            ("base",   Tb), ("joint_1", Tj1), ("joint_2", Tj2),
            ("elbow",  Telbow),
            ("joint_3", Tj3), ("joint_4", Tj4), ("joint_5", Tj5),
            ("joint_6", Tj6), ("tool0",   Ttool)]:
        v = Tw @ T_local @ np.array([0.0, 0.0, 0.0, 1.0])
        out[name] = (float(v[0]), float(v[1]), float(v[2]))
    return out


def gripper_local_axis_world(q, axis_local):
    """V44 helper for the lateral grasp.  Return the unit vector in
    WORLD frame corresponding to `axis_local` (3-tuple) in the
    tcp_grasp_center frame.  The closing axis of the gripper is
    (0, 1, 0) in tcp_grasp_center's frame (it inherits the orientation
    of tool0 since GRIPPER_YAW_MOUNT=0 and the TCP joints are identity).
    """
    M = fk_world_to_grasp_center(q)
    R = M[:3, :3]
    a = np.array(axis_local, dtype=float)
    w = R @ a
    n = np.linalg.norm(w)
    if n < 1e-9:
        return np.array([0.0, 0.0, 0.0])
    return w / n


def lateral_grasp_delta_world(q_seed):
    """V51 helper: given a seed joint config, return the LATERAL_GRASP
    _DELTA vector expressed in WORLD frame.  This is the world-frame
    translation from CAFI centre to TCP for a lateral grasp at the
    URDF-debug golden reference pose.
    """
    x_world = gripper_local_axis_world(q_seed, (1.0, 0.0, 0.0))
    y_world = gripper_local_axis_world(q_seed, (0.0, 1.0, 0.0))
    z_world = gripper_local_axis_world(q_seed, (0.0, 0.0, 1.0))
    return (LATERAL_GRASP_DELTA[0] * x_world +
            LATERAL_GRASP_DELTA[1] * y_world +
            LATERAL_GRASP_DELTA[2] * z_world)


def lateral_target_for_pick(q_seed, p_center):
    """V51: given a seed q and a desired CAFI-centre world target,
    return the SHIFTED TCP target so the runtime CAFI centre lands at
    the URDF-debug cafi_lateral_target_frame relative to the gripper.
    Replaces the V44 scalar formulation.
    """
    delta_w = lateral_grasp_delta_world(q_seed)
    p_center = np.asarray(p_center, dtype=float)
    return p_center + delta_w


def pos(M):
    return np.array([M[0,3], M[1,3], M[2,3]])

def rot_mat(M):
    return M[:3, :3].copy()

def mat_to_rpy(R):
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        roll  = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2(R[1,0], R[0,0])
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0.0
    return (roll, pitch, yaw)


# ============================================================
# Jacobian / damped LS IK (position + orientation)
# ============================================================
def jacobian_world_grasp(q, eps=1e-6):
    M0 = fk_world_to_grasp_center(q)
    p0 = pos(M0); R0 = rot_mat(M0)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = list(q); dq[i] += eps
        M = fk_world_to_grasp_center(dq)
        dp = (pos(M) - p0) / eps
        dR = rot_mat(M) @ R0.T
        omega = np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0],
                          dR[1,0]-dR[0,1]]) / 2.0 / eps
        J[0:3, i] = dp
        J[3:6, i] = omega
    return J


def damped_ls_ik(q_init, p_target, R_target=None,
                 max_iter=400, pos_tol=1e-4, rot_tol=3e-2,
                 damping=0.05, step_clip=0.20):
    """Damped-LS IK with periodic perturbation to escape local minima."""
    import random as _r
    rng = _r.Random(hash(tuple(q_init)) & 0xFFFFFFFF)
    q = np.array(q_init, dtype=float)
    last_pos_err = float('inf')
    no_progress_count = 0
    prev_err = float('inf')
    for it in range(max_iter):
        M = fk_world_to_grasp_center(q)
        e_pos = p_target - pos(M)
        if R_target is None:
            err = e_pos
            J = jacobian_world_grasp(q)[0:3, :]
            converged = np.linalg.norm(e_pos) < pos_tol
        else:
            dR = R_target @ rot_mat(M).T
            e_rot = np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0],
                              dR[1,0]-dR[0,1]]) / 2.0
            err = np.concatenate([e_pos, e_rot])
            J = jacobian_world_grasp(q)
            converged = (np.linalg.norm(e_pos) < pos_tol and
                         np.linalg.norm(e_rot) < rot_tol)
        last_pos_err = float(np.linalg.norm(e_pos))
        if converged:
            return q, True, it, last_pos_err
        if abs(prev_err - last_pos_err) < 1e-5:
            no_progress_count += 1
        else:
            no_progress_count = 0
        prev_err = last_pos_err
        if no_progress_count >= 15:
            for i in range(6):
                lo, hi = JOINT_LIMITS[i]
                amt = 0.6 if i >= 3 else 0.3
                q[i] += rng.uniform(-amt, amt)
                q[i] = min(hi, max(lo, q[i]))
            no_progress_count = 0
            continue
        JTJ = J.T @ J + (damping**2) * np.eye(6)
        try:
            dq = np.linalg.solve(JTJ, J.T @ err)
        except np.linalg.LinAlgError:
            break
        nrm = np.linalg.norm(dq)
        if nrm > step_clip:
            dq *= step_clip / nrm
        q = q + dq
        for i in range(6):
            lo, hi = JOINT_LIMITS[i]
            q[i] = min(hi, max(lo, q[i]))
    return q, False, max_iter, last_pos_err


# ============================================================
# Self-test
# ============================================================
def selftest():
    print("=" * 78)
    print("V43 Lexium kinematics SELF-TEST  (new URDF + integrated final gripper)")
    print("=" * 78)
    print(f"WORLD_COBOT_XY = {WORLD_COBOT_XY}")
    print(f"WORLD_COBOT_Z  = {WORLD_COBOT_Z} m")
    print(f"WORLD_COBOT_RPY= {WORLD_COBOT_RPY}  (floor mount Rx(+pi/2))")
    print(f"JOINT2_RPY     = {JOINT2_RPY}  (user URDF original CAD)")
    print(f"GRASP_CENTER_OFFSET (tool0->tcp) = {GRASP_CENTER_OFFSET}")
    print()
    q0 = [0.0] * 6
    Mb = fk_base_to_tool0(q0)
    pb = pos(Mb)
    rpy_b = mat_to_rpy(rot_mat(Mb))
    print(f"base->tool0 (q=0)  xyz=({pb[0]:+.4f}, {pb[1]:+.4f}, {pb[2]:+.4f})  "
          f"rpy=({rpy_b[0]:+.3f}, {rpy_b[1]:+.3f}, {rpy_b[2]:+.3f})")

    Mw = fk_world_to_tool0(q0)
    pw = pos(Mw)
    rpy_w = mat_to_rpy(rot_mat(Mw))
    print(f"world->tool0 (q=0) xyz=({pw[0]:+.4f}, {pw[1]:+.4f}, {pw[2]:+.4f})  "
          f"rpy=({rpy_w[0]:+.3f}, {rpy_w[1]:+.3f}, {rpy_w[2]:+.3f})")

    Mg = fk_world_to_grasp_center(q0)
    pg = pos(Mg)
    print(f"world->tcp_grasp_center (q=0) xyz=({pg[0]:+.4f}, {pg[1]:+.4f}, {pg[2]:+.4f})")
    print()

    print("=== V43 REAL HOME (J1=0,J2=0,J3=0,J4=+90,J5=-90,J6=0) ===")
    Mh_base = fk_base_to_tool0(POSE_HOME_Q)
    ph_base = pos(Mh_base)
    rpy_h_base = mat_to_rpy(rot_mat(Mh_base))
    print(f"base->tool0 (HOME) xyz=({ph_base[0]:+.4f}, {ph_base[1]:+.4f}, "
          f"{ph_base[2]:+.4f})  rpy=({rpy_h_base[0]:+.3f}, "
          f"{rpy_h_base[1]:+.3f}, {rpy_h_base[2]:+.3f})")
    Mh = fk_world_to_tool0(POSE_HOME_Q)
    ph = pos(Mh)
    rpy_h = mat_to_rpy(rot_mat(Mh))
    print(f"world->tool0 (HOME) xyz=({ph[0]:+.4f}, {ph[1]:+.4f}, "
          f"{ph[2]:+.4f})  rpy=({rpy_h[0]:+.3f}, {rpy_h[1]:+.3f}, "
          f"{rpy_h[2]:+.3f})")
    Mg_h = fk_world_to_grasp_center(POSE_HOME_Q)
    pg_h = pos(Mg_h)
    print(f"world->tcp (HOME) xyz=({pg_h[0]:+.4f}, {pg_h[1]:+.4f}, "
          f"{pg_h[2]:+.4f})")
    print()

    test_target = (1.235, 1.365, 1.0825)  # CONV_PICK CAFI centre
    q_sol, ok, iters, err = damped_ls_ik(POSE_HOME_Q, test_target,
                                          max_iter=400)
    Mt = fk_world_to_grasp_center(q_sol)
    print(f"IK test target = {test_target}  (PICK conveyor)")
    print(f"  q_sol = {[round(x,3) for x in q_sol]}")
    print(f"  achieved = ({Mt[0,3]:.4f}, {Mt[1,3]:.4f}, {Mt[2,3]:.4f})")
    print(f"  err = {err*1000:.2f} mm  ({'CONVERGED' if ok else 'NOT CONV'} "
          f"in {iters} iters)")


if __name__ == "__main__":
    selftest()
