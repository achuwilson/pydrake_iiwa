from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector,PublishEvent
from pydrake.systems.framework import TriggerType
from pydrake.systems.all import  AbstractValue

#import the LCM message definition
from drake import lcmt_iiwa_status

class IiwaStatusReceiver(LeafSystem):
    def __init__(self ):
        LeafSystem.__init__(self)
        self.set_name('IiwaStatusReceiver')

        #Declare the input port
        #this is actually an abstractinput port, to which the output of LcmSubscriberSystem is connected
        #Is is the LcmSubscriberSystem that actually subscribes or reads the LCM data
        self.input_port = self.DeclareAbstractInputPort('lcmt_iiwa_status', AbstractValue.Make(lcmt_iiwa_status))

        #define the output portsS
        self.output_port1 = self.DeclareVectorOutputPort("position_commanded", BasicVector(7), self.CopyStateOut1)
        self.output_port2 = self.DeclareVectorOutputPort("position_measured", BasicVector(7), self.CopyStateOut2)
        self.output_port3 = self.DeclareVectorOutputPort("velocity_estimated", BasicVector(7), self.CopyStateOut3)
        self.output_port4 = self.DeclareVectorOutputPort("torque_commanded", BasicVector(7), self.CopyStateOut4)
        self.output_port5 = self.DeclareVectorOutputPort("torque_measured", BasicVector(7), self.CopyStateOut5)
        self.output_port6 = self.DeclareVectorOutputPort("torque_external", BasicVector(7), self.CopyStateOut6)

        #declare a periodic event to update the data (at 200 Hz - defualt data rate from kuka_driver)
        #self.DeclarePeriodicEvent(period_sec =1.0/200,offset_sec=0.00,event=PublishEvent(trigger_type=TriggerType.kPeriodic,callback=self._periodic_update))

    def _periodic_update(self, context, event):
        #read data from the input port
        msg = self.input_port.Eval(context)
        #check for valid data, somehow the initial 2-3 data packets may be 0
        if(msg.num_joints ==0):
            pass
        else:
            self.output_port1.Eval(context) 
            self.output_port2.Eval(context)
            self.output_port3.Eval(context)
            self.output_port4.Eval(context)
            self.output_port5.Eval(context)
            self.output_port6.Eval(context) 



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