#A system to parse the IIWA status into various data
from pydrake.systems.all import(AbstractValue,TriggerType,LeafSystem, 
    PortDataType, BasicVector,PublishEvent)

#import the LCM message definition
from drake import lcmt_iiwa_status

class IiwaStatusReceiver(LeafSystem):
    # parses the IIWA_STATUS LCM message into the following vector valued outputs:
    # - position_commanded
    # - position_measured
    # - velocity_estimated
    # - torque_commanded
    # - torque_measured
    # - torque_external
    def __init__(self ):
        LeafSystem.__init__(self)
        self.set_name('IiwaStatusReceiver')

        #Declare the input port
        #this is actually an abstractinput port, to which the output of 
        # LcmSubscriberSystem is connected
        # It is the LcmSubscriberSystem that actually subscribes or reads
        # the LCM data
        self.input_port = self.DeclareAbstractInputPort('lcmt_iiwa_status',
                                        AbstractValue.Make(lcmt_iiwa_status))

        #define the output portsS
        self.output_port1 = self.DeclareVectorOutputPort("position_commanded",
                                                        BasicVector(7),
                                                        self.CopyStateOut1)
        self.output_port2 = self.DeclareVectorOutputPort("position_measured",
                                                        BasicVector(7),
                                                        self.CopyStateOut2)
        self.output_port3 = self.DeclareVectorOutputPort("velocity_estimated", 
                                                        BasicVector(7),
                                                        self.CopyStateOut3)
        self.output_port4 = self.DeclareVectorOutputPort("torque_commanded",
                                                        BasicVector(7),
                                                        self.CopyStateOut4)
        self.output_port5 = self.DeclareVectorOutputPort("torque_measured", 
                                                        BasicVector(7),
                                                        self.CopyStateOut5)
        self.output_port6 = self.DeclareVectorOutputPort("torque_external",
                                                        BasicVector(7),
                                                        self.CopyStateOut6)

    def CopyStateOut1(self, context, output):
        #get the input port
        #msg = self.EvalAbstractInput(context,0).get_value()
        msg = self.input_port.Eval(context)
        if(msg.num_joints==0):
            output.SetZero()
        else:
            #kuka_driver outputs data as a tuple, convert it to list
            out = list(msg.joint_position_commanded)
            #set the output port
            output.SetFromVector(out)

    def CopyStateOut2(self, context, output):
        #get the input port
        #msg = self.EvalAbstractInput(context,0).get_value()
        msg = self.input_port.Eval(context)
        if(msg.num_joints==0):
            output.SetZero()
            #print("setting zero")
            #msg = self.input_port.Eval(context)
        else:
            #print("MSG##", msg.num_joints)
            #kuka_driver outputs data as a tuple, convert it to list
            out = list(msg.joint_position_measured)
            #set the output 
            output.SetFromVector(out)    

    def CopyStateOut3(self, context, output):
        #get the input port
        #msg = self.EvalAbstractInput(context,0).get_value()
        msg = self.input_port.Eval(context)
        if(msg.num_joints==0):
            output.SetZero()
        else:    
            #kuka_driver outputs data as a tuple, convert it to list
            out = list(msg.joint_velocity_estimated)
            #set the output port
            output.SetFromVector(out)

    def CopyStateOut4(self, context, output):
        #get the input port
        #msg = self.EvalAbstractInput(context,0).get_value()
        msg = self.input_port.Eval(context)
        if(msg.num_joints==0):
            output.SetZero()
        else:
            #kuka_driver outputs data as a tuple, convert it to list
            out = list(msg.joint_torque_commanded)
            #set the output port
            output.SetFromVector(out)

    def CopyStateOut5(self, context, output):
        #get the input port
        #msg = self.EvalAbstractInput(context,0).get_value()
        msg = self.input_port.Eval(context)
        if(msg.num_joints==0):
            output.SetZero()
        else:
            #kuka_driver outputs data as a tuple, convert it to list
            out = list(msg.joint_torque_measured)
            #set the output port
            output.SetFromVector(out)

    def CopyStateOut6(self, context, output):
        #get the input port
        #msg = self.EvalAbstractInput(context,0).get_value()
        msg = self.input_port.Eval(context)
        if(msg.num_joints==0):
            output.SetZero()
        else:
            #kuka_driver outputs data as a tuple, convert it to list
            out = list(msg.joint_torque_external)
            #set the output port
            output.SetFromVector(out)