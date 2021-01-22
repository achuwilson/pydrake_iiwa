#!/usr/bin/python3
#example of a simple diagram with two systems
# A slider system whose output is connected to a printsystem
# which prints the inputs to it on terminal
from pydrake.all import ( DiagramBuilder, LeafSystem, BasicVector, Simulator,
    plot_system_graphviz, plot_graphviz, SystemSliders, PublishEvent,
    TriggerType, PortDataType)
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 1600 

#define a plant which will just print the data coming into the input port
class PrintSystem(LeafSystem):
    def __init__(self, num_input = 1):
        LeafSystem.__init__(self)
        self.set_name('PrintSystem')
        self.input_port = self.DeclareInputPort(
            "data_in", PortDataType.kVectorValued, num_input)

        #The periodic event starts at a small offset, so that we already have a couple of message already at the input port 
        self.DeclarePeriodicEvent(period_sec =1.0/100,offset_sec=0.00,event=PublishEvent(trigger_type=TriggerType.kPeriodic,callback=self._periodic_update))
    def _periodic_update(self, context, event):
        #read data from the input port
        msg = self.input_port.Eval(context)
        print(msg)

def main():
    #initialize the diagram builder
    builder = DiagramBuilder()

    ########### ADD SYSTEMS ############
    #configure slider parameters
    sliders = SystemSliders(port_size=3,
                    slider_names=["a", "b", "c"], lower_limit=-10,
                    upper_limit=10, resolution=0.001,
                    update_period_sec=0.005, title='test',
                    length=800)
    #add the slider system to diagram
    slider_sys = builder.AddSystem(sliders)
    #add the print system to diagram
    printer = builder.AddSystem(PrintSystem(num_input=3))

    ########### CONNECT THE  PORTS & BUILD ###############
    builder.Connect(slider_sys.get_output_port(0), printer.GetInputPort("data_in"))
    # Build the system
    diagram = builder.Build()
    simulator = Simulator(diagram)

    ########### PLOT #############
    plot_diagram = True
    if(plot_diagram ==True):
        img = plot_system_graphviz(diagram)
        plt.savefig("images/simple_system.png")
        plt.show()

    ######## SIMULATE/RUN ################
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(np.inf)


if __name__ == '__main__':
    main()