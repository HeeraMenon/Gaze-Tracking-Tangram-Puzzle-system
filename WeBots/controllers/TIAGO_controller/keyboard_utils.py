from head_motion_utils import make_head_look_at_target
from arm_motion_utils import arm_movement
from controller import Keyboard
import scene_objects as scene
import config as cfg
from demo_utils import move_arm_to_all_pieces

def check_keyboard(robot_parts, keyboard, robot_node):
    global last_space_down, robot_view_target  
    global init_viewpoint, init_rotation

    key = keyboard.getKey()
    speeds_left = 0.0
    speeds_right = 0.0

    if key == Keyboard.UP:
        speeds_left = cfg.MAX_SPEED
        speeds_right = cfg.MAX_SPEED

    elif key == Keyboard.DOWN:
        speeds_left = -cfg.MAX_SPEED
        speeds_right = -cfg.MAX_SPEED

    elif key == Keyboard.RIGHT:
        speeds_left = cfg.MAX_SPEED
        speeds_right = -cfg.MAX_SPEED

    elif key == Keyboard.LEFT:
        speeds_left = -cfg.MAX_SPEED
        speeds_right = cfg.MAX_SPEED

    elif key == ord('0'):
        vp_pos_field = scene.viewpoint.getField("position")
        vp_rot_field = scene.viewpoint.getField("orientation")
        vp_pos_field.setSFVec3f(cfg.init_viewpoint_coord)
        vp_rot_field.setSFRotation(cfg.init_viewpoint_rotation)

    elif key == ord('1'):
        print("Moving arm to puzzle outline")
        arm_movement(scene.puzzle_outline, robot_node)
        
    elif key == ord('2'):
        print("Moving arm to small triangle target piece")
        arm_movement(scene.small_triangle_target, robot_node) 

    elif key == ord('3'):
        print("Moving arm to big triangle 1 target piece")
        arm_movement(scene.big_triangle_1_target, robot_node)
    
    elif key == ord('4'):
        print("Moving arm to big triangle 2 target piece")
        arm_movement(scene.big_triangle_2_target, robot_node)
    
    elif key == ord('5'):
        print("Moving arm to square target piece")
        arm_movement(scene.square_target, robot_node)

    elif key == ord('6'):
        print("Moving arm to para 1 target piece")
        arm_movement(scene.para_1_target, robot_node)

    elif key == ord('7'):
        print("Moving arm to para 2 target piece")
        arm_movement(scene.para_2_target, robot_node)
    elif key == ord('9'):
        move_arm_to_all_pieces(robot_node)
        
        
    space_now = (key == ord(' '))
    if space_now and not last_space_down:
        print("SPACE pressed: making head look at camera")
        if robot_node is not None and scene.viewpoint is not None:
            viewpoint_pos_field = scene.viewpoint.getField("position")
            cam_pos  = viewpoint_pos_field.getSFVec3f() 
            robot_view_target = cam_pos
            make_head_look_at_target(robot_parts, robot_node, robot_view_target)
        else:
            print("Cannot look at camera: robot_node or viewpoint is None.")

    # update the state for the next timestep
    last_space_down = space_now

    robot_parts[cfg.MOTOR_LEFT].setVelocity(speeds_left)
    robot_parts[cfg.MOTOR_RIGHT].setVelocity(speeds_right)