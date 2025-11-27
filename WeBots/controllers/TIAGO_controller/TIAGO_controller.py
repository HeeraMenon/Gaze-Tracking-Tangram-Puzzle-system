from controller import Supervisor, Keyboard, Camera, RangeFinder
import math

MAX_SPEED = 7.0

MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

robot_view_target = None
vp_pos_field = None
vp_rot_field = None

init_viewpoint = [0, -2.05, 2.6]
init_rotation = [-0.37570441069953675, 0.3756483724608402, 0.8471921246378744, 1.748990849844424]


# ---------------------------------------------------------------------
# Helper: make head look at the Viewpoint
# ---------------------------------------------------------------------
def normalize_angle(a):
    # keep angle in [-pi, pi]
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def make_head_look_at_target(robot_parts, robot_node, target_pos):
    # Fields on robot and viewpoint
    base_translation_field = robot_node.getField("translation")
    base_rotation_field    = robot_node.getField("rotation")
    

    if base_translation_field is None or base_rotation_field is None or target_pos is None:
        print("Could not get translation/rotation/position fields.")
        return

    # Robot base pose
    base_pos = base_translation_field.getSFVec3f()   # [x, y, z]
    base_rot = base_rotation_field.getSFRotation()   # [ax, ay, az, angle]
          # [x, y, z]

    head_pos = [
    base_pos[0],
    base_pos[1],
    base_pos[2] + 0.8
]

    # --- Extract body yaw from axis-angle (assuming planar, axis ~ [0,0,±1]) ---
    ax, ay, az, angle = base_rot
    # default if axis is weird: assume yaw = 0
    if abs(az) > 1e-3:
        body_yaw = angle * (1.0 if az >= 0.0 else -1.0)
    else:
        body_yaw = 0.0

 
    # Vector from base to camera (world frame)
    dx = target_pos[0] - head_pos[0]
    dy = -target_pos[1] - head_pos[1]
    dz = target_pos[2] - head_pos[2]

    # Guard against degenerate case 
    if dx == 0 and dy == 0:
        print("Head target direction is undefined (camera exactly above/below in XY).")
        return

    # --- World yaw to camera and relative yaw for the head ---
    world_yaw = math.atan2(dy, dx)           # yaw of vector base→camera in world frame
    rel_yaw   = normalize_angle(world_yaw + body_yaw)  # yaw relative to body orientation


    # vertical angle
    dist_xy = math.sqrt(dx * dx + dy * dy)
    pitch   = math.atan2(dz, dist_xy)        # positive if camera is above

    # Desired joint angles (head joints)
    # pan = relative yaw, tilt = opposite sign of pitch (depends on joint convention)
    desired_pan  = rel_yaw
    desired_tilt = -pitch

    head_tilt = robot_parts[0]  # "head_2_joint"
    head_pan  = robot_parts[1]  # "head_1_joint"

    # --- Get joint limits from Webots ---
    pan_min  = head_pan.getMinPosition()
    pan_max  = head_pan.getMaxPosition()
    tilt_min = head_tilt.getMinPosition()
    tilt_max = head_tilt.getMaxPosition()

    BIG = 1e6
    if abs(pan_min) > BIG:  pan_min = -BIG
    if abs(pan_max) > BIG:  pan_max = BIG
    if abs(tilt_min) > BIG: tilt_min = -BIG
    if abs(tilt_max) > BIG: tilt_max = BIG

    # --- Clamp to limits ---
    clamped_yaw  = max(pan_min,  min(pan_max,  desired_pan))
    clamped_tilt = max(tilt_min, min(tilt_max, desired_tilt))

    if abs(clamped_yaw - desired_pan) > 1e-3 or abs(clamped_tilt - desired_tilt) > 1e-3:
        print(
            "Head target out of range. "
            f"Requested pan={desired_pan:.2f}, tilt={desired_tilt:.2f} "
            f"→ clamped to pan={clamped_yaw:.2f}, tilt={clamped_tilt:.2f}"
        )

    # Apply the (possibly clamped) positions
    head_pan.setPosition(-clamped_yaw)
    head_tilt.setPosition(-clamped_tilt)



# ---------------------------------------------------------------------
# Helper: handles keyboard input
# ---------------------------------------------------------------------
def check_keyboard(robot_parts, keyboard, robot_node, head_pan_sensor, head_tilt_sensor):
    global last_space_down, robot_view_target  
    global init_viewpoint, init_rotation, vp_pos_field, vp_rot_field        

    key = keyboard.getKey()
    speeds_left = 0.0
    speeds_right = 0.0

    if key == Keyboard.UP:
        speeds_left = MAX_SPEED
        speeds_right = MAX_SPEED

    elif key == Keyboard.DOWN:
        speeds_left = -MAX_SPEED
        speeds_right = -MAX_SPEED

    elif key == Keyboard.RIGHT:
        speeds_left = MAX_SPEED
        speeds_right = -MAX_SPEED

    elif key == Keyboard.LEFT:
        speeds_left = -MAX_SPEED
        speeds_right = MAX_SPEED

    elif key == ord('0'):
        vp_pos_field.setSFVec3f(init_viewpoint)
        vp_rot_field.setSFRotation(init_rotation)
        

    space_now = (key == ord(' '))
    if space_now and not last_space_down:
        print("SPACE pressed: making head look at camera")
        if robot_node is not None and viewpoint is not None:
            viewpoint_pos_field = viewpoint.getField("position")
            cam_pos  = viewpoint_pos_field.getSFVec3f() 
            robot_view_target = cam_pos
            make_head_look_at_target(robot_parts, robot_node, robot_view_target)
        else:
            print("Cannot look at camera: robot_node or viewpoint is None.")

    # update the state for the next timestep
    last_space_down = space_now

    robot_parts[MOTOR_LEFT].setVelocity(speeds_left)
    robot_parts[MOTOR_RIGHT].setVelocity(speeds_right)



# ---------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------
robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

# Enable RGB + depth cameras (devices, not supervisor nodes)
rgb = robot.getDevice("Astra rgb")
rgb.enable(time_step)

depth = robot.getDevice("Astra depth")
depth.enable(time_step)

# Robot part names (same order as C code)
names = [
    "head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
    "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint",
    "arm_6_joint", "arm_7_joint", "wheel_left_joint", "wheel_right_joint"
]

# Target positions for initialization
target_pos = [0.24, -0.67, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, float('inf'), float('inf')]

robot_parts = []

# Configure motors
for i in range(N_PARTS):
    motor = robot.getDevice(names[i])
    robot_parts.append(motor)
    motor.setVelocity(motor.getMaxVelocity() / 2.0)
    motor.setPosition(target_pos[i])

# Position sensors for pan/tilt
head_pan_sensor  = robot.getDevice("head_1_joint_sensor")
head_tilt_sensor = robot.getDevice("head_2_joint_sensor")

head_pan_sensor.enable(time_step)
head_tilt_sensor.enable(time_step)


# Get the robot node (this robot, as a Pose node)
robot_node = robot.getSelf()
if robot_node is None:
    print("Error: getSelf() returned None, something is wrong with the controller binding.")


viewpoint = robot.getFromDef("USER_CAMERA")
if viewpoint is None:
    print("Error: could not find USER_CAMERA Viewpoint. Check the DEF name in the world file.")
else:
    vp_pos_field = viewpoint.getField("position")
    vp_rot_field = viewpoint.getField("orientation")


vp_pos_field.setSFVec3f(init_viewpoint)
vp_rot_field.setSFRotation(init_rotation) 

# Enable keyboard
keyboard = Keyboard()
keyboard.enable(time_step) 


initial_time = robot.getTime()

print("Use arrow keys to drive. Press SPACE to turn head toward the camera.")

# initial view direction of robot
right1_node = robot.getFromDef("PARA_PIECE_1_A")
if right1_node is None:
    print("Could not find PARA_PIECE_1_A node")

piece_pos = 0
last_robot_view_target = 0

if right1_node is not None:
    translation_field = right1_node.getField("translation")



# Main loop
while robot.step(time_step) != -1:
    # piece_pos = translation_field.getSFVec3f()  # [x, y, z]
    # print("PARA_PIECE_1_A position:", piece_pos)
    # robot_view_target = piece_pos
    check_keyboard(robot_parts, keyboard, robot_node, head_pan_sensor, head_tilt_sensor)
    # if robot_view_target and not last_robot_view_target:
    #     make_head_look_at_target(robot_parts, robot_node, robot_view_target)
    #     last_robot_view_target = robot_view_target

    # "Hello" waving movement (arm joint 8)
    t = robot.getTime() - initial_time
    robot_parts[8].setPosition(0.3 * math.sin(5.0 * t) - 0.3)
