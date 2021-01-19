from pydrake.lcm import DrakeLcm
from pydrake.systems.all import LcmSubscriberSystem, LcmInterfaceSystem,AbstractValue

from drake import lcmt_iiwa_status
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector
from pydrake.systems.analysis import Simulator

from pydrake.systems.framework import PublishEvent
from pydrake.systems.framework import TriggerType

from iiwa_status_receiver import IiwaStatusReceiver

import numpy as np
import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz, plot_graphviz
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 1200


#define a plant which will just print the data coming into the input port
class PrintPlant(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.set_name('PrintPlant')
        self.input_port = self.DeclareInputPort(
            "print_in", PortDataType.kVectorValued, 7)

        #The periodic event starts at a small offset, so that we already have a couple of message already at the input port 
        self.DeclarePeriodicEvent(period_sec =1.0/2000,offset_sec=0.010,event=PublishEvent(trigger_type=TriggerType.kPeriodic,callback=self._periodic_update))
    def _periodic_update(self, context, event):
        #read data from the input port
        msg = self.input_port.Eval(context)
        print(msg)

lcm = DrakeLcm()
builder = DiagramBuilder()

#################### ADD SYSTEMS ############################
#The LcmInterfaceSystem has no inputs nor outputs nor state nor parameters; 
# it declares only an update event that pumps LCM messages into 
# their subscribers if the LCM stack has message(s) waiting. 
# The subscribers will then update their outputs using their
#  own declared events
lcm_sys = builder.AddSystem(LcmInterfaceSystem(lcm=lcm))

#This LcmSubscriberSystem subscribes to the LCM data stream and outputs
# the recived data through a single output port 
subscriber_sys = builder.AddSystem(LcmSubscriberSystem.Make('IIWA_STATUS', lcmt_iiwa_status,lcm))

#WaitForMessage helps us to wait until a message arrives
value = AbstractValue.Make(lcmt_iiwa_status())
old_message_count = 10
print("Waiting for IIWA_STATUS ...")
while True:
    new_count = subscriber_sys.WaitForMessage(old_message_count, value, timeout=0.2)
    if new_count > old_message_count:
        break
    lcm.HandleSubscriptions(0)
#received message    
iiwa_status_parser_sys = builder.AddSystem(IiwaStatusReceiver() )
print_sys = builder.AddSystem(PrintPlant())


################## MAKE CONNECTIONS #########################
builder.Connect(subscriber_sys.get_output_port(0),iiwa_status_parser_sys.GetInputPort('lcmt_iiwa_status'))

# connect any of the following output ports to printer
# "position_commanded"
# "position_measured"
# "velocity_estimated"
# "torque_commanded"
# "torque_measured"
# "torque_external"
builder.Connect(iiwa_status_parser_sys.GetOutputPort("position_measured"),print_sys.GetInputPort("print_in"))

#################### BUILD ################################
diagram = builder.Build()

################### PLOT DIAGRAM ########################
plot_diagram = True
if(plot_diagram ==True):
    img = plot_system_graphviz(diagram)
    plt.show()

diagram_context = diagram.CreateDefaultContext()
simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1)

station_context = diagram.GetMutableSubsystemContext(iiwa_status_parser_sys, simulator.get_mutable_context())
# This is important to avoid duplicate publishes to the hardware interface:
simulator.set_publish_every_time_step(False)
#simulator.Initialize()
print("starting")
simulator.AdvancePendingEvents()
for i in range(100):
    print("####", iiwa_status_parser_sys.GetOutputPort("position_measured").Eval(station_context))

simulation_time = np.inf
#simulator.AdvanceTo(simulation_time)
