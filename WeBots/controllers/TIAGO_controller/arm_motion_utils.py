

from coord_utils import world_to_robot
import numpy as np
import math
import config as cfg
import scene_objects as scene

my_chain = None
arm_sensors = None
names = None
robot_parts = None
robot = None
time_step = None# Workaround for ikpy on newer NumPy versions
if not hasattr(np, "float"):
    np.float = float

def init_ik(chain, arm_sensors_map, names_list, robot_parts_list, robot_obj, ts):
    global my_chain, arm_sensors, names, robot_parts, robot, time_step
    my_chain = chain
    arm_sensors = arm_sensors_map
    names = names_list
    robot_parts = robot_parts_list
    robot = robot_obj
    time_step = ts

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
# Workaround for ikpy on newer NumPy versions
if not hasattr(np, "float"):
    np.float = float

def distance_to_target(target_xyz):
    pos, _ = get_end_effector_pose_from_sensors()
    dx = pos[0] - target_xyz[0]
    dy = pos[1] - target_xyz[1]
    dz = pos[2] - target_xyz[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def get_current_chain_positions():
    """
    Build a list of joint positions for all links in my_chain.
    Disabled links get 0. Active links map to the Webots motors.
    Uses motor target positions only, no sensors.
    """
    current = [0.0] * len(my_chain.links)

    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name

        if link_name in names:
            motor_index = names.index(link_name)
            motor = robot_parts[motor_index]
            # Use last commanded position; good enough for IK initial guess
            current[i] = motor.getTargetPosition()
        else:
            current[i] = 0.0

    return current

def get_end_effector_pose():
    # 1) get current joint configuration
    joints = get_joint_positions_from_sensors()

    # 2) run forward kinematics
    T = my_chain.forward_kinematics(joints)  # 4x4 homogeneous matrix

    # 3) extract position
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    # 4) extract orientation (rotation matrix)
    R = T[:3, :3]

    return (x, y, z), R

def get_joint_positions_from_sensors():
    joints = [0.0] * len(my_chain.links)
    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name
        if link_name in arm_sensors:
            joints[i] = arm_sensors[link_name].getValue()
        else:
            joints[i] = 0.0

    return joints


def get_end_effector_pose_from_sensors():
    joints = get_joint_positions_from_sensors()
    T = my_chain.forward_kinematics(joints)
    pos = (T[0, 3], T[1, 3], T[2, 3])
    R = T[:3, :3]
    return pos, R

# ---------------------------------------------------------------------
# IKPY-based arm motion
# ---------------------------------------------------------------------
def move_arm_to_position(target_xyz, orientation=None, orientation_mode=None,
                         error_tolerance=0.005):
    """
    Use IKPY to move the TIAGo arm so that the end-effector reaches `target_xyz`
    in robot base coordinates.

    Parameters
    ----------
    target_xyz : list/tuple of length 3
        [x, y, z] in the same base frame as the IK chain (usually base_link).
    orientation : list/tuple of length 3 or 4, optional
        Optional target orientation vector, passed to IKPY.
        For example [0, 0, 1] to keep the tool roughly vertical.
    orientation_mode : str, optional
        IKPY orientation mode, e.g. "Y" or "all". None means no orientation.
    error_tolerance : float
        If the distance error after solution is greater than this, you can
        decide to re-run IK or log a warning.
    """
    # 1) Get current joint configuration as initial guess
    initial_position = get_joint_positions_from_sensors()

    # 2) Run IK
    if orientation is not None and orientation_mode is not None:
        ik_results = my_chain.inverse_kinematics(
            target_xyz,
            initial_position=initial_position,
            target_orientation=orientation,
            orientation_mode=orientation_mode,
        )
    else:
        ik_results = my_chain.inverse_kinematics(
            target_xyz,
            initial_position=initial_position,
        )

    # 3) Optional: compute position error in task space (rough check)
    #    (Using the requested target and forward kinematics on ik_results)
    fk_result = my_chain.forward_kinematics(ik_results)
    end_eff_pos = fk_result[:3, 3]
    err = math.sqrt(
        (end_eff_pos[0] - target_xyz[0]) ** 2 +
        (end_eff_pos[1] - target_xyz[1]) ** 2 +
        (end_eff_pos[2] - target_xyz[2]) ** 2
    )

    if err > error_tolerance:
        print(f"[IK] Warning: end-effector error {err:.4f} > tolerance {error_tolerance:.4f}")

    # 4) Apply joint angles to Webots motors
    for i, link in enumerate(my_chain.links):
        if not my_chain.active_links_mask[i]:
            continue

        link_name = link.name

        # Only send commands to joints that correspond to Webots motors
        if link_name in names:
            motor_index = names.index(link_name)
            motor = robot_parts[motor_index]
            target_angle = ik_results[i]
            motor.setPosition(target_angle)
            # You can log if needed:
            # print(f"Setting {link_name} to {target_angle:.3f}")

    return err

def move_arm_to_position_blocking(target_xyz,
                                  orientation=None,
                                  orientation_mode=None,
                                  pos_tolerance=0.005,
                                  max_steps=1000,
                                  min_z=0.52):
    # Command the move
    err_ik = move_arm_to_position(
        target_xyz,
        orientation,
        orientation_mode,
        error_tolerance=pos_tolerance
    )
    
    if err_ik > pos_tolerance:
        print("[IK] Target likely unreachable, aborting blocking wait")
        return

    steps = 0
    while robot.step(time_step) != -1:
        # SAFETY CHECK: make sure end-effector doesn't go below min_z
        pos, _ = get_end_effector_pose_from_sensors()
        current_z = pos[2]   # assuming chain frame shares z with world (usual TIAGo case)

        if current_z < min_z:
            print(f"[IK] SAFETY: end-effector z={current_z:.3f} < min_z={min_z:.3f}, aborting move.")
            # Optionally try to move straight up to safety:
            safe_target = [pos[0], pos[1], min_z]
            move_arm_to_position(safe_target, orientation, orientation_mode)
            break

        # Normal convergence check
        err = distance_to_target(target_xyz)
        if err < pos_tolerance:
            print(f"[IK] Reached target within tolerance {pos_tolerance:.4f}")
            break

        steps += 1
        if steps > max_steps:
            print("[IK] Warning: timeout waiting for target", target_xyz)
            break


def motor_blocking(target_angle):
    sensor = arm_sensors["arm_4_joint"]   # not "..._sensor"
    motor  = robot_parts[names.index("arm_4_joint")]

    while robot.step(time_step) != -1:
        curr_rotation = sensor.getValue()
        err = abs(curr_rotation - target_angle)
        if err < 0.01:
            break
    
def arm_movement(targetObject, lift, tableHeight, robotNode):
    """
    Move the arm to a given targetObject in three phases:
      1) go up to a safe height (>= min_z),
      2) move horizontally above the target at that safe height,
      3) move down to the target height, but never below min_z.

    `tableHeight` is the height of the table in WORLD coords; we enforce that
    no commanded end-effector position ever goes below max(tableHeight, 0.52).
    """
    if targetObject is None:
        print("[arm_movement] ERROR: targetObject is None")
        return

    # Safety height in world coordinates
    MIN_Z_WORLD = max(tableHeight, 0.40)

    # 1) Get target position in WORLD coordinates
    targetPosWC = list(targetObject.getField("translation").getSFVec3f())
    # Clamp target z so we never aim below the allowed height
    if targetPosWC[2] < MIN_Z_WORLD:
        print(f"[arm_movement] Clamping target Z from {targetPosWC[2]:.3f} to {MIN_Z_WORLD:.3f}")
        targetPosWC[2] = MIN_Z_WORLD

    # 2) Convert clamped world coordinates to ROBOT coordinates
    targetPosRC = world_to_robot(targetPosWC, robotNode)
    target_x, target_y, target_z = targetPosRC

    # 3) Get current end-effector pose (in the same frame as IK chain)
    pos, _ = get_end_effector_pose_from_sensors()
    current_x, current_y, current_z = pos

    # Safety height in robot coordinates:
    # we assume robot z-axis is aligned with world z, so we can use the same threshold.
    MIN_Z = MIN_Z_WORLD

    # -----------------------------------------------
    # Phase 0: pre-lift joint to avoid sweeping low
    # -----------------------------------------------
    # lift_rotation = 0.8
    # robot_parts[names.index("arm_4_joint")].setPosition(lift_rotation)
    # motor_blocking(lift_rotation)

    # -----------------------------------------------
    # Phase 1: go straight up from current position
    # -----------------------------------------------
    phase1_z = max(current_z, MIN_Z + lift)
    phase1_target = [current_x, current_y, phase1_z]

    move_arm_to_position_blocking(
        phase1_target,
        orientation=[0, 0, 1],
        min_z=MIN_Z,
    )

    # -----------------------------------------------
    # Phase 2: move horizontally above the target
    # -----------------------------------------------
    phase2_z = max(target_z + lift, MIN_Z + lift)
    phase2_target = [target_x, target_y, phase2_z]

    move_arm_to_position_blocking(
        phase2_target,
        # orientation=[0, 0, 1],
        # orientation_mode="Z",
        pos_tolerance=0.2,
        min_z=MIN_Z,
    )

    # -----------------------------------------------
    # Phase 3: descend to the clamped target height
    # -----------------------------------------------
    final_z = max(target_z, MIN_Z)
    phase3_target = [target_x, target_y, final_z]

    move_arm_to_position_blocking(
        phase3_target,
        pos_tolerance=0.01,
        min_z=MIN_Z,
        orientation=[0, 0, 1],
        orientation_mode="Z",
    )
