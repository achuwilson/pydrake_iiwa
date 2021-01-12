from pydrake.systems.framework import Diagram,DiagramBuilder
from pydrake.lcm import DrakeLcm
from pydrake.all import MultibodyPlant, Parser,FindResourceOrThrow
from pydrake.systems.all import LcmSubscriberSystem, LcmInterfaceSystem, LcmPublisherSystem,  AbstractValue

from drake import lcmt_iiwa_status, lcmt_iiwa_command

from iiwa_status_receiver import IiwaStatusReceiver
from iiwa_command_sender import IiwaCommandSender
from pydrake.systems.framework import BasicVector, LeafSystem
import pydrake.all

import numpy as np
import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz, plot_graphviz
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 1200

class IiwaHardwareInterface(Diagram):
    def __init__(self):
        Diagram.__init__(self)
        self.set_name("IiwaHardwareInterface")
        self.lcm = DrakeLcm()
        self.plant = MultibodyPlant(time_step=0.0)
        self.builder = DiagramBuilder()
        self.num_joints=7

        self.lcm_sys = self.builder.AddSystem(LcmInterfaceSystem(lcm=self.lcm))

        #subscribe to data from hardware
        self.iiwa_status_subscriber = self.builder.AddSystem(LcmSubscriberSystem.Make('IIWA_STATUS', lcmt_iiwa_status,self.lcm))
        self.iiwa_status_parser = self.builder.AddSystem(IiwaStatusReceiver())
        #connect the LCM subscriber
        self.builder.Connect(self.iiwa_status_subscriber.get_output_port(0),self.iiwa_status_parser.GetInputPort('lcmt_iiwa_status'))
        #bring out all the other ports
        self.builder.ExportOutput(self.iiwa_status_parser.GetOutputPort("position_measured"), "iiwa_position_measured")
        self.builder.ExportOutput(self.iiwa_status_parser.GetOutputPort("position_commanded"),"iiwa_position_commanded")
        self.builder.ExportOutput(self.iiwa_status_parser.GetOutputPort("velocity_estimated"),"iiwa_velocity_estimated")
        self.builder.ExportOutput(self.iiwa_status_parser.GetOutputPort("torque_commanded"),"iiwa_torque_commanded")
        self.builder.ExportOutput(self.iiwa_status_parser.GetOutputPort("torque_measured"),"iiwa_torque_measured")
        self.builder.ExportOutput(self.iiwa_status_parser.GetOutputPort("torque_external"),"iiwa_torque_external")
       
        self.iiwa_command_publisher = self.builder.AddSystem(LcmPublisherSystem.Make(channel = 'IIWA_COMMAND', lcm_type = lcmt_iiwa_command, lcm = self.lcm,publish_period = 0.005))
        self.iiwa_command_parser = self.builder.AddSystem(IiwaCommandSender())
        #connect the publisher to LCM
        self.builder.Connect(self.iiwa_command_parser.GetOutputPort("lcmt_iiwa_command"),self.iiwa_command_publisher.get_input_port(0))
        #export inputs to outside
        self.builder.ExportInput(self.iiwa_command_parser.GetInputPort("position"), "iiwa_position")
        self.builder.ExportInput(self.iiwa_command_parser.GetInputPort("torque"), "iiwa_feedforward_torque")

        self.builder.BuildInto(self)

        Parser(self.plant).AddModelFromFile(FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"))
        self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("iiwa_link_0"))
        self.plant.Finalize()
    
    def Connect(self):
        value = AbstractValue.Make(lcmt_iiwa_status())
        old_message_count = 10 # wait until 10 messages received
        print("Waiting for IIWA_STATUS ...")
        while True:
            new_count = self.iiwa_status_subscriber.WaitForMessage(old_message_count, value, timeout=0.2)
            if new_count > old_message_count:
                break
            self.lcm.HandleSubscriptions(0)
        print("Received IIWA_STATUS...")
      
    def num_iiwa_joints(self):
        return self.num_joints  
    
    def get_controller_plant(self):
        return self.plant     


