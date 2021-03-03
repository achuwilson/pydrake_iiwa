#!/usr/bin/python3
# Control the kuka iiwa joints through sliders, 
# while printing out the end effector position
# using fo![](images/fk_system.png)rward kinematics
from pydrake.manipulation.simple_ui import JointSliders
from pydrake.systems.framework import (DiagramBuilder, 
    LeafSystem, BasicVector, PublishEvent, TriggerType)
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import FirstOrderLowPassFilter
from iiwa_manipulation_station import IiwaManipulationStation
import numpy as np
import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz, plot_graphviz
import matplotlib as mpl
from pydrake.all import GenerateHtml, RollPitchYaw
mpl.rcParams['figure.dpi'] = 1200    
import time
import lcm
from drake import lcmt_iiwa_status

#initiate the LCM instance
lc = lcm.LCM()

#this is used to subscribe to LCM mesages
#so as to 
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

class forwardkin(LeafSystem):
    # This system has to be initialized with a multibodyplant
    # it has a single input and prints the End Effector XYZ 
    # position into terminal
    def __init__(self,plant):
        LeafSystem.__init__(self)
        self.set_name('FKSystem')
        self._plant =  plant
        self.plant_context = plant.CreateDefaultContext()
        #Get the handler for the last link. Replace with gripper/custom 
        # end effector if required
        self._G = plant.GetBodyByName("iiwa_link_7")
        self._W = plant.world_frame()
        
        self.input_port = self.DeclareVectorInputPort("iiwa_position_in", 
                                                    BasicVector(7))

        #declare a periodic update event. This will call the function 
        # _periodic_update at 200 Hz
        self.DeclarePeriodicEvent(period_sec =1.0/200,
                                    offset_sec=0.010,
                                    event=PublishEvent(
                                        trigger_type=TriggerType.kPeriodic,
                                    callback=self._periodic_update))

    def _periodic_update(self, context, event):
        # This function will be called at the rat6e specified in
        # DeclarePeriodicEvent
        #
        # evaluate the input_port and get the joint position values
        msg = self.input_port.Eval(context)
        # update the joint position values of the multibodyplant
        self._plant.SetPositions(self.plant_context, msg)
        # evaluate the end effector position
        eepos = self._plant.EvalBodyPoseInWorld(self.plant_context, self._G)
        # print the XYZ translation estimated. The rotation is available 
        # using eepos.rotation() - as rotation matrix
        print("Gripper Pos ", eepos.translation(), RollPitchYaw(eepos.rotation()).roll_angle(), RollPitchYaw(eepos.rotation()).pitch_angle(), RollPitchYaw(eepos.rotation()).yaw_angle())



def main():
    builder = DiagramBuilder()
    ########### ADD SYSTEMS ############
    station = builder.AddSystem(IiwaManipulationStation())  
    station.Finalize()
    station.Connect()

    teleop = builder.AddSystem(JointSliders(station.get_controller_plant(),
                length=800))

    num_iiwa_joints = station.num_iiwa_joints()
    filter = builder.AddSystem(FirstOrderLowPassFilter(
        time_constant=1.00, size=num_iiwa_joints))

    fkinsys = builder.AddSystem(forwardkin(station.get_controller_plant())) 

    ########### CONNECT THE PORTS ###########
    builder.Connect(teleop.get_output_port(0), 
                            filter.get_input_port(0))
    builder.Connect(filter.get_output_port(0),
                            station.GetInputPort("iiwa_position"))
    builder.Connect(station.GetOutputPort("iiwa_position_measured"), 
                            fkinsys.GetInputPort("iiwa_position_in"))                        

    ########### BUILD ##############
    diagram = builder.Build()

    ########### PLOT #############
    plot_diagram = True
    if(plot_diagram ==True):
        img = plot_system_graphviz(diagram)
        plt.savefig("images/fk_system.png")
        plt.show()

    ######### SET INITIAL CONDITIONS ##########
    simulator = Simulator(diagram)

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(num_iiwa_joints))

    #query the initial position from hardware
    lc.handle()
    initPos= subscription.msg.joint_position_measured
  
    #set the initial values of the filter output
    filter.set_initial_output_value(
        diagram.GetMutableSubsystemContext(
            filter, simulator.get_mutable_context()), list(initPos))
    #set the slider positions such that it matches the hardware        
    teleop.set_position(list(initPos))

    simulator.set_target_realtime_rate(1.0)

    ######## SIMULATE/RUN ################
    simulator.AdvanceTo(np.inf)

if __name__ == '__main__':
    main()
