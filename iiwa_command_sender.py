from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector,PublishEvent
from pydrake.systems.framework import TriggerType
from pydrake.systems.all import  AbstractValue
from drake import lcmt_iiwa_command
import time

class IiwaCommandSender(LeafSystem):
    #   encodes the input into IIWA_COMMAND
    #   LCM message which is pubblished by the LcmSubscriberSystem
    def __init__(self ):
        LeafSystem.__init__(self)
        self.set_name('IiwaCommandSender')
        self.num_joints=7

        #Declare input ports
        self.input_port1 = self.DeclareInputPort("position", 
                                                PortDataType.kVectorValued, 
                                                self.num_joints)
        self.input_port2 = self.DeclareInputPort("torque",
                                                PortDataType.kVectorValued, 
                                                self.num_joints)
        #Declare output port
        self.output_port = self.DeclareAbstractOutputPort("lcmt_iiwa_command",
                                lambda: AbstractValue.Make(lcmt_iiwa_command),
                                self.CalcOutput)


    def CalcOutput(self,context,output):
        msg_cmd = lcmt_iiwa_command() 
        position = self.input_port1.Eval(context)
        msg_cmd.utime = int(time.time()*1000000)
        msg_cmd.num_joints = self.num_joints
        msg_cmd.joint_position = position
        #evaluate the optional torque
        isTorque  = self.input_port2.HasValue(context)
        if(isTorque):
            torque = self.input_port2.Eval(context)
            msg_cmd.num_torques = self.num_joints
            msg_cmd.joint_torque = torque
        else:
            msg_cmd.num_torques =0               
        output.set_value(msg_cmd)
        