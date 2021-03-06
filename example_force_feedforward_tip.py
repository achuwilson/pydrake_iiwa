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

from pydrake.all import SystemSliders, PortDataType,JacobianWrtVariable,Parser
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

class ffcontrol(LeafSystem):
    def __init__(self, plant):
        LeafSystem.__init__(self)
        self._plant =  plant
        self.plant_context = plant.CreateDefaultContext()
        #self._iiwa = plant.GetModelInstanceByName("iiwa")
        self._G = plant.GetBodyByName("tip")#.body_frame()
        self._W = plant.world_frame()
        #print("Plant ", help(plant))

        self.input_port = self.DeclareVectorInputPort("iiwa_position_in", BasicVector(7))
        self.input_port2 = self.DeclareVectorInputPort("cartforce_in", BasicVector(6))
        self.output_port =self.DeclareVectorOutputPort("torque_commanded", BasicVector(7), self.CopyStateOut1)
        print("FFCONTROL INIT")

    def CopyStateOut1(self,context,output):
        pos_ = self.input_port.Eval(context)
        force_ =self.input_port2.Eval(context)
        plant_context = self._plant.CreateDefaultContext()
        self._plant.SetPositions(plant_context, pos_)
        J = self._plant.CalcJacobianSpatialVelocity(plant_context,JacobianWrtVariable.kQDot,self._G.body_frame(),[0,0,0],self._W, self._W)
        
        torques  = np.dot(J.T,force_)
        print("torques ", torques)
        output.SetFromVector(torques)

#define a plant which will just print the data coming into the input port
class PrintPlant(LeafSystem):
    def __init__(self, num_input = 1):
        LeafSystem.__init__(self)
        self.set_name('PrintPlant')
        self.input_port = self.DeclareInputPort(
            "print_in", PortDataType.kVectorValued, num_input)

        #The periodic event starts at a small offset, so that we already have a couple of message already at the input port 
        self.DeclarePeriodicEvent(period_sec =1.0/100,offset_sec=0.00,event=PublishEvent(trigger_type=TriggerType.kPeriodic,callback=self._periodic_update))
    def _periodic_update(self, context, event):
        #read data from the input port
        msg = self.input_port.Eval(context)
        print(msg)

def main():
    builder = DiagramBuilder()
    station = builder.AddSystem(IiwaHardwareInterface())
    robot = station.get_controller_plant()
    finger = Parser(robot).AddModelFromFile("models/onefinger.urdf", "simplefinger")
    X_7G = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, 0.045])
    robot.WeldFrames(robot.GetFrameByName("iiwa_link_7"),robot.GetFrameByName("finger_base", finger), X_7G)
    station.Finalize()
    station.Connect()

    robot = station.get_controller_plant()
    params =  DifferentialInverseKinematicsParameters(7,7)

    sliders = SystemSliders(port_size=6,
                    slider_names=["r", "p", "y","x", "y", "z"], lower_limit=-10,
                    upper_limit=10, resolution=0.001,
                    update_period_sec=0.005, title='Feed Forward Force',
                    length=800)
    slider_sys = builder.AddSystem(sliders)
    #print_sys = builder.AddSystem(PrintPlant(num_input = 6))
    print("DONE SLIDES")
    ff_sys = builder.AddSystem(ffcontrol(station.get_controller_plant())) 
    print("FFCONTROL DONE")
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

    builder.Connect(slider_sys.get_output_port(0),ff_sys.GetInputPort("cartforce_in"))  
    builder.Connect(station.GetOutputPort("iiwa_position_measured"),ff_sys.GetInputPort("iiwa_position_in")) 
    builder.Connect(ff_sys.GetOutputPort("torque_commanded"), station.GetInputPort("iiwa_feedforward_torque"))     
    #builder.Connect(slider_sys.get_output_port(0),print_sys.GetInputPort("print_in"))          

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    #station.GetInputPort("iiwa_feedforward_torque").FixValue(
    #    station_context, np.zeros(7))

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
