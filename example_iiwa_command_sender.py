from pydrake.lcm import DrakeLcm
from pydrake.systems.all import LcmPublisherSystem, LcmInterfaceSystem

from drake import lcmt_iiwa_command
from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector,PublishEvent,DiagramBuilder
from pydrake.systems.all import  AbstractValue
from pydrake.systems.analysis import Simulator

from iiwa_command_sender import IiwaCommandSender


import numpy as np
import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz, plot_graphviz
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 600


lcm = DrakeLcm()
builder = DiagramBuilder()

robot_command_publisher = builder.AddSystem(LcmPublisherSystem.Make(channel = 'IIWA_COMMAND', lcm_type = lcmt_iiwa_command, lcm = lcm,publish_period = 0.005))
robot_command_publisher.set_name('robot_command_publisher')
#robot_command_publisher.set_publish_period(0.005) #corresponding to 200 Hz for Kuka_driver

#lcm message parser
robot_command_parser = builder.AddSystem(IiwaCommandSender())

builder.Connect(robot_command_parser.GetOutputPort("lcmt_iiwa_command"),robot_command_publisher.get_input_port(0))

#################### BUILD ################################
diagram = builder.Build()

q0 = [0,0,0,0,0,0,0]
t0 = [0,0,0,0,0,0,0]
#robot_command_parser.GetInputPort("position").FixValue(q0)
#robot_command_parser.GetInputPort("torque").FixValue(t0)


################### PLOT DIAGRAM ########################
plot_diagram = True
if(plot_diagram ==True):
    img = plot_system_graphviz(diagram)
    plt.show()

diagram_context = diagram.CreateDefaultContext()
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1)

context = robot_command_parser.GetMyContextFromRoot(simulator.get_mutable_context())
q0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
t0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
robot_command_parser.GetInputPort("position").FixValue(context,q0)
#robot_command_parser.GetInputPort("torque").FixValue(context,t0)

# This is important to avoid duplicate publishes to the hardware interface:
simulator.set_publish_every_time_step(False)
simulator.Initialize()

simulation_time = np.inf
simulator.AdvanceTo(simulation_time)