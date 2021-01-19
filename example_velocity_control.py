#!/usr/bin/python3
#Control the kuka iiwa end effector velocity through sliders
#NOTE : be extremely carefull when running this code
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
from pydrake.all import JacobianWrtVariable, SystemSliders, Integrator
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from differential_ik import DifferentialIK

import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 1200    
import time
import lcm
from drake import lcmt_iiwa_status

#initiate the LCM instance
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

class PsuedoInverseVelocityController(LeafSystem):
    ''' A Jacobian Psuedo Inverse based velocity controller

        Initialize with the multibodyplant, provide the current joint position
        and the desired end effector velocity 

    ''' 
    def __init__(self,plant):
        LeafSystem.__init__(self)
        self._plant =  plant
        self.plant_context = plant.CreateDefaultContext()
        
        #The world frame and the end-effector frame
        self._G = plant.GetBodyByName("iiwa_link_7").body_frame()
        self._W = plant.world_frame()
        
        #Declare the input and output ports
        self.input_port = self.DeclareVectorInputPort("iiwa_position_in",
                                                        BasicVector(7))
        self.input_port2 = self.DeclareVectorInputPort("velocity_desired",
                                                        BasicVector(6))
        self.output_port =self.DeclareVectorOutputPort("velocity_commanded", 
                                                        BasicVector(7),
                                                        self.CopyStateOut)

    def CopyStateOut(self,context,output):
        pos_ = self.input_port.Eval(context)
        v_des =self.input_port2.Eval(context)
        plant_context = self._plant.CreateDefaultContext()
        #update the joinbt positions of the plant with current values
        self._plant.SetPositions(plant_context, pos_)
        #calculate the Jacobian
        J = self._plant.CalcJacobianSpatialVelocity(plant_context,
                                                    JacobianWrtVariable.kQDot,
                                                    self._G,
                                                    [0,0,0],
                                                    self._W,
                                                    self._W)
        # Muliply the Jacobian Pseudoinverse with the desired cartesian 
        # velocities to get the joint velocities 
        v_q = np.linalg.pinv(J).dot(v_des)
        output.SetFromVector(v_q)


def main():
    builder = DiagramBuilder()

    ######## ADD SYSTEMS #############
    station = builder.AddSystem(IiwaHardwareInterface())
    station.Finalize()
    station.Connect()

    #get the multibodyplant inside the station
    robot = station.get_controller_plant()
    
    sliders = SystemSliders(port_size=6,
                    slider_names=["r", "p", "y","x", "y", "z"],
                    lower_limit=-0.025,
                    upper_limit=0.025,
                    resolution=0.001,
                    update_period_sec=0.005,
                    title='Cartesian velocities',
                    length=800)
    slider_sys = builder.AddSystem(sliders)
    
    controller = builder.AddSystem(PsuedoInverseVelocityController(robot)) 
    #an integrator to convert velocity commands to position commands 
    integrator = builder.AddSystem(Integrator(7))   

    ######### MAKE CONNECTIONS ############
    builder.Connect(station.GetOutputPort("iiwa_position_measured"),
                        controller.GetInputPort("iiwa_position_in"))
    builder.Connect(slider_sys.get_output_port(0),
                        controller.GetInputPort("velocity_desired"))    
    builder.Connect(controller.GetOutputPort("velocity_commanded"),
                        integrator.get_input_port())
    builder.Connect(integrator.get_output_port(),
                        station.GetInputPort("iiwa_position"))

    ######### BUILD ############
    diagram = builder.Build()
    simulator = Simulator(diagram)

    ######### SET INTIAL CONDITIONS/PARAMETERS ###########
    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(7))

    #get the initial values from he hardware
    lc.handle()
    initPos= list(subscription.msg.joint_position_measured)
    
    #THIS IS IMPORTANT
    #Set the initial value of the integrator.
    #The values computed by the PseudoInVerseVelocityController 
    #will be added to the initial value of the integrator.So set 
    # it as the initial pose of the controller 
    integrator.GetMyContextFromRoot(simulator.get_mutable_context())
                                .get_mutable_continuous_state_vector()
                                    .SetFromVector(initPos)

    simulator.set_target_realtime_rate(1.0)

    ############ RUN ##############
    simulator.AdvanceTo(np.inf)

if __name__ == '__main__':
    main()
