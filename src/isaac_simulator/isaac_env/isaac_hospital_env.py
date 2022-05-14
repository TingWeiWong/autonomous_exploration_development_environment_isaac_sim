import carb
from omni.isaac.kit import SimulationApp
import numpy as np
import sys
import argparse
import os


# We have to start simulation app first to import those packages
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

import omni
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from pxr import Gf
from omni.isaac.core.robots import Robot

# enable ROS bridge extension
extensions.enable_extension("omni.isaac.ros_bridge")

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose



CARTER_STAGE_PATH = "/World/Carter"
BACKGROUND_STAGE_PATH = "/World/hospital"
LIDAR_STAGE_PATH = "/World/Local_lidar"
CLOCK_PATH = "/World/ROS_Clock"
HEIGHT = 30

class HospitalScene:
    def __init__(self, RootPath):
        ROBOT_USD_PATH = os.path.join(RootPath, "../world/Carter_ROS_template.usd")
        BACKGROUND_USD_PATH = "/Isaac/Environments/Hospital/hospital.usd"
        LIDAR_USD_PATH = os.path.join(RootPath, "../world/Lidar.usd")
        print("*****"*8)
        print(ROBOT_USD_PATH)
        print(LIDAR_USD_PATH)
        SCALE_FACTOR = 0.75
        print("*****"*8)
        self.world = World(stage_units_in_meters=0.01)
        # Locate /Isaac folder on nucleus server to load environment and robot stages
        result, _nucleus_path = nucleus.find_nucleus_server()
        if result is False:
            carb.log_error("Could not find nucleus server with /Isaac folder, exiting")
            simulation_app.close()
            sys.exit()

        # Preparing stage
        self._robot_position = np.array([0, 0, HEIGHT])
        self._robot_orientation = rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 0))

        self._lidar_position = np.array([0, 0, 50])
        self._lidar_orientation = rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 0))
        viewports.set_camera_view(eye=np.array([120, 120, 80]), target=np.array([0, 0, 50]))

        # Loading the simple_room environment
        stage.add_reference_to_stage(_nucleus_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH)
        
        # Loading the franka robot USD
        prims.create_prim(
            CARTER_STAGE_PATH,
            "Xform",
            position=self._robot_position,
            orientation=self._robot_orientation,
            usd_path=ROBOT_USD_PATH,
            # scale should be numpy array, not as showed in document
            scale = np.array([SCALE_FACTOR, SCALE_FACTOR, SCALE_FACTOR]),
        )

        simulation_app.update()

        prims.create_prim(
            LIDAR_STAGE_PATH,
            "Xform",
            position=self._lidar_position,
            orientation=self._lidar_orientation,
            usd_path=LIDAR_USD_PATH,
        )
        simulation_app.update()
        

        # Load ROS Clock
        omni.kit.commands.execute("ROSBridgeCreateClock", path="/World/ROS_Clock", enabled=False)
        rospy.init_node('navmsg_sub', anonymous=True)
        self.ros_sub = rospy.Subscriber("/nav_path", Pose, self.move_robot_callback, queue_size=10)
        self.world.reset()

        simulation_app.update()

    def move_robot_callback(self, data):
        # callback function to set the cube position to a new one upon receiving a (empty) ros message
        if self.world.is_playing():
            x = data.position.x*80
            y = data.position.y*80
            z_robot = self._robot_position[-1]
            z_lidar = self._lidar_position[-1]
            yaw = data.orientation.x

            self._robot_orientation = rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw*180/np.pi))
            self._robot_position = np.array([x, y, z_robot])
            self._lidar_position = np.array([x, y, z_lidar])


    def run_simulator(self):
        self.world.scene.add(Robot(prim_path=CARTER_STAGE_PATH, name="Carter"))
        self.world.scene.add(Robot(prim_path=LIDAR_STAGE_PATH, name="Local_lidar"))
        omni.kit.commands.execute("RosBridgeTickComponent", path=CLOCK_PATH)

        self.world.start_simulation()
        self.world.play()
        frame = 0
        while simulation_app.is_running():
            # Run with a fixed step size
            self.world.step(render=True)

            # move the robot, avoid fall through
            if frame % 2  == 0:
                self._robot_position[2] = HEIGHT + 1e-3
            else:
                self._robot_position[2] = HEIGHT - 1e-3
            frame += 1
            
            self.world.scene.get_object("Carter").set_world_pose(position = self._robot_position, orientation = self._robot_orientation)
            self.world.scene.get_object("Local_lidar").set_world_pose(position = self._lidar_position, orientation = self._lidar_orientation)
            omni.kit.commands.execute("RosBridgeTickComponent", path=CLOCK_PATH)

        self.ros_sub.unregister()
        self.world.stop()
        simulation_app.close()

if __name__ == "__main__":
    ROOTPATH = os.path.dirname(os.path.realpath(__file__))

    scene_test = HospitalScene(ROOTPATH)
    scene_test.run_simulator()
