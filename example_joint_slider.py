#!/usr/bin/python3
#Control the kuka iiwa joints through sliders
from pydrake.manipulation.simple_ui import JointSliders
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import FirstOrderLowPassFilter
from iiwa_manipulation_station import IiwaManipulationStation
import numpy as np

import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz, plot_graphviz
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 1200    
import time
import lcm
from drake import lcmt_iiwa_status

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

def main():
    builder = DiagramBuilder()

    ########### ADD SYSTEMS ############
    station = builder.AddSystem(IiwaHardwareInterface())
    station.Finalize()
    station.Connect()
    
    teleop = builder.AddSystem(JointSliders(station.get_controller_plant(),length=800))

    num_iiwa_joints = station.num_iiwa_joints()
    filter = builder.AddSystem(FirstOrderLowPassFilter(
        time_constant=1.00, size=num_iiwa_joints))

    ########### CONNECT THE PORTS and BUILD ###########
    builder.Connect(teleop.get_output_port(0), 
                            filter.get_input_port(0))
    builder.Connect(filter.get_output_port(0),
                            station.GetInputPort("iiwa_position"))
    
    diagram = builder.Build()
    simulator = Simulator(diagram)

    ########### PLOT #############
    plot_diagram = False
    if(plot_diagram ==True):
        img = plot_system_graphviz(diagram)
        plt.savefig("images/jointcontrol.png")
        plt.show()

    ######### SET INITIAL CONDITIONS ##########
    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(num_iiwa_joints))

    #query the initial position from hardware
    lc.handle()
    initPos= list(subscription.msg.joint_position_measured)

    #set the initial values of the filter output
    filter.set_initial_output_value(
        diagram.GetMutableSubsystemContext(
            filter, simulator.get_mutable_context()), initPos)
    #set the slider positions such that it matches the hardware        
    teleop.set_position(initPos)

    ######## SIMULATE/RUN ################
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(np.inf)

if __name__ == '__main__':
    main()
