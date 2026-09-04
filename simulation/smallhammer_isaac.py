from isaacsim import SimulationApp


# START ISAAC SIM
simulation_app = SimulationApp({
    "headless": False
})

from isaacsim.core.api import World
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

import socket
import threading
import queue
import json
import math
import numpy as np


# path for urdf 
URDF_PATH = (
    "/home/ali/Downloads/"
    "smallhammer_ros2-fixed/"
    "smallhammer_ros2-main/"
    "ros2_ws/src/"
    "smallhammer_description/urdf"
)

URDF_FILE = "smallhammer.urdf"

TCP_PORT = 9000 # ESP32 connects to Isaac Sim on TCP port 9000.

command_queue = queue.Queue()


#tcp server
def tcp_server():

    server = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM
    )

    server.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )

    server.bind(
        ("0.0.0.0", TCP_PORT)
    )

    server.listen(5)

    print(" SMALLHAMMER ISAAC SIM SERVER")
    print(f"Listening on TCP port {TCP_PORT}")
    print("Waiting for ESP32...")
    print()

    while True:

        client, address = server.accept()

        print(
            f"ESP32 connected from {address}"
        )

        buffer = ""

        try:

            while True:

                data = client.recv(1024)

                if not data:
                    break

                buffer += data.decode(
                    "utf-8"
                )

                while "\n" in buffer:

                    line, buffer = buffer.split(
                        "\n",
                        1
                    )

                    line = line.strip()

                    if not line:
                        continue

                    try:

                        command = json.loads(line)

                        print(
                            "RECEIVED:",
                            command
                        )

                        command_queue.put(
                            command
                        )

                    except json.JSONDecodeError:

                        print(
                            "Invalid JSON:",
                            line
                        )

        except Exception as e:

            print(
                "ESP32 connection error:",
                e
            )

        finally:

            client.close()

            print(
                "ESP32 disconnected"
            )


# START TCP THREAD
server_thread = threading.Thread(
    target=tcp_server,
    daemon=True
)

server_thread.start()



# URDF IMPORT
print("Importing Small Hammer URDF...")
print()


urdf_interface = (
    _urdf.acquire_urdf_interface()
)


import_config = _urdf.ImportConfig()

import_config.merge_fixed_joints = False

import_config.fix_base = True

import_config.self_collision = False

import_config.default_drive_type = (
    _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
)


robot_model = urdf_interface.parse_urdf(
    URDF_PATH,
    URDF_FILE,
    import_config
)



#robot import
robot_prim_path = urdf_interface.import_robot(
    URDF_PATH,
    URDF_FILE,
    robot_model,
    import_config,
    "/World/smallhammer",
    True
)

print(
    "Robot imported at:",
    robot_prim_path
)



#bare-bone world environment
world = World()

world.reset()

for _ in range(10):

    world.step(
        render=True
    )


#articulation
robot = SingleArticulation(
    prim_path=robot_prim_path,
    name="smallhammer"
)

robot.initialize()


#show joints
print(" ISAAC JOINTS")


for i, name in enumerate(
    robot.dof_names
):

    print(
        f"{i}: {name}"
    )

print()



#joint index-lookup
JOINT_INDEX = {}

for i, name in enumerate(
    robot.dof_names
):

    JOINT_INDEX[name] = i


# ============================================================
# PHYSICAL SERVO CHANNEL -> URDF JOINT
#
# UNCHANGED.
#
# ESP32:
#
# CH6 = J1 Base
# CH1 = J2 Shoulder
# CH2 = J3 Elbow
# CH3 = J4 Wrist Pitch
# CH4 = J5 Wrist Roll
# CH5 = J6 Gripper
# ============================================================

CHANNEL_TO_JOINT = {

    6: "base_joint",

    1: "shoulder_joint",

    2: "elbow_joint",

    3: "wrist_pitch_joint",

    4: "wrist_roll_joint",
}


# ============================================================
# SERVO DIRECTION
#
# CORRECTED:
#
# J1 BASE       = KEEP
# J2 SHOULDER   = KEEP
# J3 ELBOW      = INVERT
# J4 WRIST      = KEEP EXISTING INVERSION
# J5 WRIST ROLL = KEEP
#
# ============================================================

SERVO_DIRECTION = {

    # J1 BASE
    6: 1.0,

    # J2 SHOULDER
    1: 1.0,

    # J3 ELBOW
    #
    # Physical elbow UP must make
    # simulated elbow go UP.
    #
    # Previous mapping was inverted.
    #
    2: -1.0,

    # J4 WRIST PITCH
    #
    # Existing tested direction.
    #
    3: -1.0,

    # J5 WRIST ROLL
    4: 1.0,
}


# ============================================================
# SERVO CENTER
#
# Physical servo:
#
# 90 degrees = neutral
#
# Isaac:
#
# 0 radians = neutral/home
#
# ============================================================

SERVO_CENTER = {

    1: 90.0,   # J2

    2: 90.0,   # J3

    3: 90.0,   # J4

    4: 90.0,   # J5

    6: 90.0,   # J1
}


# ============================================================
# HOME OFFSETS
#
# THESE ARE THE CORRECT ISAAC HOME POSITIONS.
#
# J1 = 0
# J2 = 0
# J3 = 0
# J4 = 0
# J5 = 0
#
# ============================================================

HOME_OFFSET = {

    # J1
    "base_joint": 0.0,

    # J2
    "shoulder_joint": 0.0,

    # J3
    "elbow_joint": 0.0,

    # J4
    "wrist_pitch_joint": 0.0,

    # J5
    "wrist_roll_joint": 0.0,
}


# ============================================================
# GRIPPER
#
# URDF gripper_joint is PRISMATIC.
#
# Range:
#
# 0.00 m = closed
# 0.02 m = open
#
# Physical servo:
#
# 90 -> higher = CLOSE
# 90 -> lower  = OPEN
# ============================================================

GRIPPER_MIN = 0.0

GRIPPER_MAX = 0.02


# ============================================================
# PERSISTENT JOINT TARGETS
# When ONE servo moves:
#
# only its target changes
#
# but ALL joint targets remain actively commanded.
#
# Therefore:
#
# moving BASE does not release shoulder/elbow/wrist
# moving WRIST does not release shoulder/elbow
# moving GRIPPER does not release the arm
#
# ============================================================

JOINT_TARGETS = {}



# INITIALIZE TARGETS

current_positions = (
    robot.get_joint_positions()
)


for name in robot.dof_names:

    index = JOINT_INDEX[name]

    JOINT_TARGETS[name] = float(
        current_positions[index]
    )



# FORCE ARM TARGETS TO CORRECT HOME 90 degree servo angle


for joint_name in HOME_OFFSET:

    if joint_name in JOINT_INDEX:

        JOINT_TARGETS[
            joint_name
        ] = HOME_OFFSET[
            joint_name
        ]


# ============================================================
# INITIAL J6 POSITION
#
# Physical 90-degree neutral corresponds to the middle
# of the 0.00 - 0.02 m gripper travel.
# ============================================================

if "gripper_joint" in JOINT_INDEX:

    JOINT_TARGETS[
        "gripper_joint"
    ] = 0.01


# ============================================================
# APPLY ALL STORED TARGETS
#
# We explicitly provide joint_indices.
#
# This is important because a target of 0 is a REAL target
# for our arm home position.
# ============================================================

def apply_all_targets():

    controlled_names = []

    # --------------------------------------------------------
    # J1
    # --------------------------------------------------------

    if "base_joint" in JOINT_INDEX:

        controlled_names.append(
            "base_joint"
        )

    # --------------------------------------------------------
    # J2
    # --------------------------------------------------------

    if "shoulder_joint" in JOINT_INDEX:

        controlled_names.append(
            "shoulder_joint"
        )

    # --------------------------------------------------------
    # J3
    # --------------------------------------------------------

    if "elbow_joint" in JOINT_INDEX:

        controlled_names.append(
            "elbow_joint"
        )

    # --------------------------------------------------------
    # J4
    # --------------------------------------------------------

    if "wrist_pitch_joint" in JOINT_INDEX:

        controlled_names.append(
            "wrist_pitch_joint"
        )

    # --------------------------------------------------------
    # J5
    # --------------------------------------------------------

    if "wrist_roll_joint" in JOINT_INDEX:

        controlled_names.append(
            "wrist_roll_joint"
        )

    # --------------------------------------------------------
    # J6
    # --------------------------------------------------------

    if "gripper_joint" in JOINT_INDEX:

        controlled_names.append(
            "gripper_joint"
        )


    # --------------------------------------------------------
    # BUILD EXPLICIT JOINT INDEX LIST
    # --------------------------------------------------------

    indices = np.array(
        [
            JOINT_INDEX[name]
            for name in controlled_names
        ],
        dtype=np.int32
    )


    # --------------------------------------------------------
    # BUILD TARGET VECTOR
    #
    # Revolute values are radians.
    #
    # Prismatic gripper value is meters.
    # --------------------------------------------------------

    targets = np.array(
        [
            JOINT_TARGETS[name]
            for name in controlled_names
        ],
        dtype=np.float32
    )


    # --------------------------------------------------------
    # APPLY POSITION TARGETS
    # --------------------------------------------------------

    action = ArticulationAction(
        joint_positions=targets,
        joint_indices=indices
    )


    robot.apply_action(
        action
    )


# SERVO -> ISAAC RADIANS

def servo_to_radians(
    channel,
    servo_angle
):

    center = SERVO_CENTER[
        channel
    ]

    direction = SERVO_DIRECTION[
        channel
    ]

    joint_name = CHANNEL_TO_JOINT[
        channel
    ]


    # Difference from physical 90-degree center
    relative_angle = (
        servo_angle - center
    )


    # Apply experimentally determined direction
    relative_angle *= direction


    # Convert degrees -> radians
    radians = math.radians(
        relative_angle
    )


    # Apply Isaac home offset
    radians += HOME_OFFSET[
        joint_name
    ]


    return radians


# ============================================================
# SET ONLY ONE JOINT TARGET
#
# We change only the requested TARGET and then reapply all
# stored targets so every other joint remains actively held.
# ============================================================

def set_single_joint(
    joint_name,
    radians
):

    if joint_name not in JOINT_INDEX:

        print(
            "ERROR: Joint not found:",
            joint_name
        )

        return


   
    # MODIFY ONLY THIS JOINT'S TARGET
   

    JOINT_TARGETS[
        joint_name
    ] = radians


    # KEEP EVERY OTHER JOINT HELD


    apply_all_targets()

# ARM SERVO COMMAND


def handle_servo(
    channel,
    servo_angle
):

    joint_name = (
        CHANNEL_TO_JOINT[channel]
    )


    radians = servo_to_radians(
        channel,
        servo_angle
    )


    print()

    print(
        "----------------------------------------"
    )

    print(
        f"CH {channel}"
    )

    print(
        f"Joint: {joint_name}"
    )

    print(
        f"Servo angle: {servo_angle:.1f}°"
    )

    print(
        f"Isaac target: "
        f"{math.degrees(radians):.1f}°"
    )

    print(
        "----------------------------------------"
    )


    set_single_joint(
        joint_name,
        radians
    )


# ============================================================
# GRIPPER SERVO -> PRISMATIC POSITION
#
# Physical behavior:
#
# lower servo angle = OPEN
# higher servo angle = CLOSE
#
#
# Servo:
#
#   0 degrees   -> 0.020 m OPEN
#   90 degrees  -> 0.010 m CENTER
#   180 degrees -> 0.000 m CLOSED
#
#
# This preserves:
#
# 90 -> 120 = closes
# 90 -> lower = opens
# ============================================================

def gripper_servo_to_position(
    servo_angle
):

    servo_angle = max(
        0.0,
        min(
            180.0,
            servo_angle
        )
    )


    opening = (
        GRIPPER_MAX
        *
        (
            1.0
            -
            servo_angle / 180.0
        )
    )


    return opening


# ============================================================
# GRIPPER
#
#
# The current URDF contains ONE controllable J6:
#
#     gripper_joint
#
# We therefore control ONLY gripper_joint.
#
# We do NOT look for:
#
#     finger_left_joint
#     finger_right_joint
#
# because those are not the controllable J6 DOF in this URDF.
#
# Most importantly:
#
# opening/closing J6 changes ONLY its stored target.
#
# J1-J5 continue receiving their existing targets.
# ============================================================

def handle_gripper(
    servo_angle
):

    if "gripper_joint" not in JOINT_INDEX:

        print(
            "ERROR: gripper_joint not found."
        )

        return


    opening = gripper_servo_to_position(
        servo_angle
    )


    # CHANGE ONLY J6 TARGET
    

    JOINT_TARGETS[
        "gripper_joint"
    ] = opening

    # REAPPLY ALL TARGETS
    #J1-J5 remain actively held.
  

    apply_all_targets()


    print()

    print(
        "----------------------------------------"
    )

    print(
        "CH 5"
    )

    print(
        "Joint: gripper_joint"
    )

    print(
        f"Servo angle: {servo_angle:.1f}°"
    )

    print(
        f"Gripper opening: {opening:.4f} m"
    )

    print(
        "----------------------------------------"
    )



# CENTER / HOME
def center_robot():

    print()

    print(
        "========================================"
    )

    print(
        " HOME POSITION"
    )

    print(
        "========================================"
    )



    # J1 BASE
    if "base_joint" in JOINT_INDEX:

        JOINT_TARGETS[
            "base_joint"
        ] = HOME_OFFSET[
            "base_joint"
        ]


    # J2 SHOULDER

    if "shoulder_joint" in JOINT_INDEX:

        JOINT_TARGETS[
            "shoulder_joint"
        ] = HOME_OFFSET[
            "shoulder_joint"
        ]

    # J3 ELBOW

    if "elbow_joint" in JOINT_INDEX:

        JOINT_TARGETS[
            "elbow_joint"
        ] = HOME_OFFSET[
            "elbow_joint"
        ]

    # J4 WRIST PITCH
    if "wrist_pitch_joint" in JOINT_INDEX:

        JOINT_TARGETS[
            "wrist_pitch_joint"
        ] = HOME_OFFSET[
            "wrist_pitch_joint"
        ]


    # J5 WRIST ROLL

    if "wrist_roll_joint" in JOINT_INDEX:

        JOINT_TARGETS[
            "wrist_roll_joint"
        ] = HOME_OFFSET[
            "wrist_roll_joint"
        ]


 
    # J6 GRIPPER
    # Physical center command = servo 90 degrees.

    if "gripper_joint" in JOINT_INDEX:

        JOINT_TARGETS[
            "gripper_joint"
        ] = gripper_servo_to_position(
            90.0
        )


    # COMMAND ALL HOME TARGETS TOGETHER


    apply_all_targets()


    print(
        "J1 Base        -> 0°"
    )

    print(
        "J2 Shoulder    -> 0°"
    )

    print(
        "J3 Elbow       -> 0°"
    )

    print(
        "J4 Wrist Pitch -> 0°"
    )

    print(
        "J5 Wrist Roll  -> 0°"
    )

    print(
        "J6 Gripper     -> servo 90° / 0.010 m"
    )

    print()


# ============================================================
# PROCESS COMMAND
#
# JSON FORMAT:
# CENTER:
#
# {
#     "cmd": "center"
# }
#
# SERVO:
#
# {
#     "ch": 2,
#     "angle": 120
# }
#
# ============================================================

def process_command(
    command
):

    # CENTER

    if command.get("cmd") == "center":

        center_robot()

        return
    # CHANNEL


    channel = command.get(
        "ch",
        -1
    )


    #angle
    angle = command.get(
        "angle",
        -1
    )

    # VALIDATE
 

    if not isinstance(
        channel,
        (int, float)
    ):

        print(
            "Invalid channel"
        )

        return


    if not isinstance(
        angle,
        (int, float)
    ):

        print(
            "Invalid angle"
        )

        return


    channel = int(
        channel
    )

    angle = float(
        angle
    )


    if angle < 0:

        angle = 0


    if angle > 180:

        angle = 180


    # J6 GRIPPER


    if channel == 5:

        handle_gripper(
            angle
        )

        return


    # J1-J5 ARM

    if channel in CHANNEL_TO_JOINT:

        handle_servo(
            channel,
            angle
        )

        return


    print(
        "Unknown channel:",
        channel
    )


# INITIAL HOME
center_robot()


# ALLOW HOME TARGETS TO SETTLE


for _ in range(60):

    apply_all_targets()

    world.step(
        render=True
    )


# Start
print()

print(
    "========================================"
)

print(
    " SMALLHAMMER DIGITAL TWIN READY"
)

print(
    "========================================"
)

print()

print(
    "Physical arm ↔ ESP32 ↔ Isaac Sim"
)

print()

print(
    "MAPPING:"
)

print()

print(
    "CH6 → J1 → BASE"
)

print(
    "CH1 → J2 → SHOULDER"
)

print(
    "CH2 → J3 → ELBOW [INVERTED]"
)

print(
    "CH3 → J4 → WRIST PITCH [INVERTED]"
)

print(
    "CH4 → J5 → WRIST ROLL"
)

print(
    "CH5 → J6 → GRIPPER"
)

print()

print(
    "CONTROL:"
)

print()

print(
    "Persistent position targets enabled."
)

print(
    "Uncommanded joints remain held at their targets."
)

print()

print(
    "Waiting for ESP32..."
)

print()


# MAIN SIMULATION LOOP


while simulation_app.is_running():

    
    # PROCESS EVERY ESP32 COMMAND

    while not command_queue.empty():

        command = (
            command_queue.get()
        )

        process_command(
            command
        )


    # --------------------------------------------------------
    # KEEP ALL JOINTS ACTIVELY TARGETED
    #
    # This is intentional.
    #
    # Even if no new ESP32 message arrives:
    #
    # J1-J6 retain their previous target positions.
    #
    # Therefore gravity should not make unrelated joints fall
    # when another joint is being controlled.
    # --------------------------------------------------------

    apply_all_targets()



    # RUN SIMULATION
  
    world.step(
        render=True
    )


# SHUTDOWN

simulation_app.close()