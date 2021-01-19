#!/usr/bin/python3
#Control the kuka iiwa end effector position through sliders, while printing  the end effector position
from pydrake.manipulation.simple_ui import JointSliders
from pydrake.systems.framework import DiagramBuilder, LeafSystem, BasicVector, PublishEvent, TriggerType
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import FirstOrderLowPassFilter
from iiwa_hardware_interface import IiwaHardwareInterface
import numpy as np
import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz, plot_graphviz
from pydrake.manipulation.planner import (
    DifferentialInverseKinematicsParameters)
from pydrake.geometry import ConnectDrakeVisualizer, SceneGraph,DrakeVisualizer
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.systems.planar_scenegraph_visualizer import (
    PlanarSceneGraphVisualizer
)
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.all import Parser, FindResourceOrThrow, RigidTransform,RollPitchYaw,ConnectMeshcatVisualizer
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from differential_ik import DifferentialIK

import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 1200    
import time
import lcm
from drake import lcmt_iiwa_status
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
lc = lcm.LCM()

#this is used to subscribe to LCM mesages
class lcm_subscriptor(object):
    def __init__(self, channel, lcm_type, lc):
        self.subscriptor = lc.subscribe(channel,self._msg_handle)
        self.lcm_type = lcm_type
        self.msg = lcm_type()
    def _msg_handle(self, channel, message):
        self.msg = self.lcm_type.decode(message)

#we subscribe to "IIWA_STATUS" LCM message to set the initial values of the 
# drake systems, so that it matches the hardware values        
subscription = lcm_subscriptor("IIWA_STATUS",lcmt_iiwa_status,lc)
lc.handle()

class EndEffectorTeleop(LeafSystem):
    def __init__(self, planar=False):
        """
        @param planar if True, restricts the GUI and the output to have y=0,
                      roll=0, yaw=0.
        """

        LeafSystem.__init__(self)
        self.DeclareVectorOutputPort("rpy_xyz", BasicVector(6),
                                     self.DoCalcOutput)

        # Note: This timing affects the keyboard teleop performance. A larger
        #       time step causes more lag in the response.
        self.DeclarePeriodicPublish(0.01, 0.0)
        self.planar = planar

        self.window = tk.Tk()
        self.window.title("End-Effector TeleOp")

        self.roll = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                             resolution=-1,
                             label="roll (keys: ctrl-right, ctrl-left)",
                             length=800,
                             orient=tk.HORIZONTAL)
        self.roll.pack()
        self.roll.set(0)
        self.pitch = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                              resolution=-1,
                              label="pitch (keys: ctrl-d, ctrl-a)",
                              length=800,
                              orient=tk.HORIZONTAL)
        if not planar:
            self.pitch.pack()
        self.pitch.set(0)
        self.yaw = tk.Scale(self.window, from_=-2 * np.pi, to=2 * np.pi,
                            resolution=-1,
                            label="yaw (keys: ctrl-up, ctrl-down)",
                            length=800,
                            orient=tk.HORIZONTAL)
        if not planar:
            self.yaw.pack()
        self.yaw.set(1.57)
        self.x = tk.Scale(self.window, from_=-0.6, to=0.8,
                          resolution=-1,
                          label="x (keys: right, left)",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.x.pack()
        self.x.set(0)
        self.y = tk.Scale(self.window, from_=-0.8, to=0.3,
                          resolution=-1,
                          label="y (keys: d, a)",
                          length=800,
                          orient=tk.HORIZONTAL)
        if not planar:
            self.y.pack()
        self.y.set(0)
        self.z = tk.Scale(self.window, from_=0, to=1.1,
                          resolution=-1,
                          label="z (keys: up, down)",
                          length=800,
                          orient=tk.HORIZONTAL)
        self.z.pack()
        self.z.set(0)

        # The key bindings below provide teleop functionality via the
        # keyboard, and are somewhat arbitrary (inspired by gaming
        # conventions). Note that in order for the keyboard bindings to
        # be active, the teleop slider window must be the active window.

        def update(scale, value):
            return lambda event: scale.set(scale.get() + value)

        # Delta displacements for motion via keyboard teleop.
        rotation_delta = 0.05  # rad
        position_delta = 0.01  # m

        # Linear motion key bindings.
        self.window.bind("<Up>", update(self.z, +position_delta))
        self.window.bind("<Down>", update(self.z, -position_delta))
        if (not planar):
            self.window.bind("<d>", update(self.y, +position_delta))
            self.window.bind("<a>", update(self.y, -position_delta))
        self.window.bind("<Right>", update(self.x, +position_delta))
        self.window.bind("<Left>", update(self.x, -position_delta))

        # Rotational motion key bindings.
        self.window.bind("<Control-Right>", update(self.roll, +rotation_delta))
        self.window.bind("<Control-Left>", update(self.roll, -rotation_delta))
        if (not planar):
            self.window.bind("<Control-d>",
                             update(self.pitch, +rotation_delta))
            self.window.bind("<Control-a>",
                             update(self.pitch, -rotation_delta))
            self.window.bind("<Control-Up>",
                             update(self.yaw, +rotation_delta))
            self.window.bind("<Control-Down>",
                             update(self.yaw, -rotation_delta))

    def SetPose(self, pose):
        """
        @param pose is an Isometry3.
        """
        tf = RigidTransform(pose)
        self.SetRPY(RollPitchYaw(tf.rotation()))
        self.SetXYZ(tf.translation())

    def SetRPY(self, rpy):
        """
        @param rpy is a RollPitchYaw object
        """
        self.roll.set(rpy.roll_angle())
        if not self.planar:
            self.pitch.set(rpy.pitch_angle())
            self.yaw.set(rpy.yaw_angle())

    def SetXYZ(self, xyz):
        """
        @param xyz is a 3 element vector of x, y, z.
        """
        self.x.set(xyz[0])
        if not self.planar:
            self.y.set(xyz[1])
        self.z.set(xyz[2])

    def DoPublish(self, context, event):
        self.window.update_idletasks()
        self.window.update()

    def DoCalcOutput(self, context, output):
        output.SetAtIndex(0, self.roll.get())
        output.SetAtIndex(1, self.pitch.get())
        output.SetAtIndex(2, self.yaw.get())
        output.SetAtIndex(3, self.x.get())
        output.SetAtIndex(4, self.y.get())
        output.SetAtIndex(5, self.z.get())

def main():
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    station = builder.AddSystem(IiwaHardwareInterface(scene_graph))
    robot = station.get_controller_plant()
    finger = Parser(robot).AddModelFromFile("models/onefinger.urdf", "simplefinger")
    X_7G = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, 0.045])
    robot.WeldFrames(robot.GetFrameByName("iiwa_link_7"),robot.GetFrameByName("finger_base", finger), X_7G)
    station.Finalize()
    
    station.Connect()

    
    params =  DifferentialInverseKinematicsParameters(7,7
                                                     )

    time_step = 0.005
    params.set_timestep(time_step)
    #True velocity limits for IIWA14 in radians
    iiwa14_velocity_limits =np.array([1.4, 1.4, 1.7, 1.3, 2.2, 2.3, 2.3])
    planar = False
    if(planar):
        iiwa14_velocity_limits = iiwa14_velocity_limits[1:6:2]
        params.set_end_effector_velocity_gain([1, 0, 0, 0, 1, 1])   

    factor =1.0 #velocity limit factor
    params.set_joint_velocity_limits((-factor*iiwa14_velocity_limits,
                                         factor*iiwa14_velocity_limits))
    differential_ik = builder.AddSystem(DifferentialIK(robot,
                robot.GetFrameByName("tip"), params, time_step))

    builder.Connect(differential_ik.GetOutputPort("joint_position_desired"),
                    station.GetInputPort("iiwa_position"))

    teleop = builder.AddSystem(EndEffectorTeleop(False))
    filter = builder.AddSystem(
        FirstOrderLowPassFilter(time_constant=2.0, size=6))
    builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
    builder.Connect(filter.get_output_port(0),
                    differential_ik.GetInputPort("rpy_xyz_desired"))


    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(robot))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"), to_pose.get_input_port())
    
    print("GET SOURCE ID ", robot.get_source_id())
    builder.Connect(
        to_pose.get_output_port(),
        scene_graph.get_source_pose_port(robot.get_source_id()))

   

    DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)
    #frames_td= {"iiwa": {"iiwa_link_1", "iiwa_link_2", "iiwa_link_3", "iiwa_link_4", "iiwa_link_5", "iiwa_link_6", "iiwa_link_7"}}
    meshcat_viz = builder.AddSystem(
            MeshcatVisualizer(scene_graph))
    builder.Connect(
            scene_graph.get_query_output_port(),
            meshcat_viz.get_geometry_query_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    
    plot_diagram = False
    if(plot_diagram ==True):
        img = plot_system_graphviz(diagram)
        plt.savefig("system.svg")
        plt.savefig("system.png")
        plt.show()
    

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(7))


    #get the initial values from he hardware
    lc.handle()
    initPos= list(subscription.msg.joint_position_measured)
    print("InitPos ", initPos)

    differential_ik.parameters.set_nominal_joint_position(initPos)
    teleop.SetPose(differential_ik.ForwardKinematics(initPos))
    #set the initial values of the filter output
    filter.set_initial_output_value(
        diagram.GetMutableSubsystemContext(
            filter, simulator.get_mutable_context()),
                teleop.get_output_port(0).Eval(diagram.GetMutableSubsystemContext(
                      teleop, simulator.get_mutable_context())))
    differential_ik.SetPositions(diagram.GetMutableSubsystemContext(
        differential_ik, simulator.get_mutable_context()), initPos)                  
    
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(np.inf)


if __name__ == '__main__':
    main()
