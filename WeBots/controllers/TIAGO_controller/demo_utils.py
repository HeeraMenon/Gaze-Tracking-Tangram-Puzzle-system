# e.g. in keyboard_utils.py or demo_utils.py

from arm_motion_utils import arm_movement
import scene_objects as scene


def _move_arm_over_group(piece_dict, x_offset, y_offset, robot_node, group_name):
    """
    Helper: loops over all nodes in piece_dict and calls arm_movement on each.

    piece_dict: dict[str, Node] from scene_objects (red_objects, blue_objects, target_objects)
    x_offset, y_offset: desired grip offset in robot frame
    robot_node: Node of the robot (Supervisor.getFromDef("TIAGO") or similar)
    group_name: string label for debug prints ("red", "blue", "target")
    """
    if not piece_dict:
        print(f"[arm_demo] WARNING: No pieces in {group_name} group.")
        return

    # Sort keys for deterministic order
    for name in sorted(piece_dict.keys()):
        node = piece_dict[name]
        if node is None:
            print(f"[arm_demo] WARNING: Node '{name}' in {group_name} group is None, skipping.")
            continue

        print(f"[arm_demo] Moving arm to {group_name} piece '{name}'")
        arm_movement(node, x_offset, y_offset, robot_node)


def move_arm_to_all_pieces(robot_node, x_offset=0.3, y_offset=0.40):
    """
    Move the arm sequentially to:
      1. all red pieces
      2. all blue pieces
      3. all target pieces

    Uses the dictionaries initialized in scene_objects.init_scene_objects(robot).
    """
    # Make sure scene objects were initialized
    # (optional but helpful)
    if scene.red_objects is None or scene.blue_objects is None:
        print("[arm_demo] ERROR: scene_objects.init_scene_objects(robot) was not called.")
        return

    print("[arm_demo] Visiting all RED pieces...")
    _move_arm_over_group(scene.red_objects, x_offset, y_offset, robot_node, "red")

    print("[arm_demo] Visiting all BLUE pieces...")
    _move_arm_over_group(scene.blue_objects, x_offset, y_offset, robot_node, "blue")

    print("[arm_demo] Visiting all TARGET pieces...")
    _move_arm_over_group(scene.target_objects, x_offset, y_offset, robot_node, "target")
