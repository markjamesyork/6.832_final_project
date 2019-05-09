'''This script loads the diff_drive plant and dynamics, and implements
a basic controller

This is a 2D controller
'''
import inspect
import argparse
import math
import numpy as np
import os.path
from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyPlant,
                         RigidBodyTree, Simulator, VectorSystem,
                         ConstantVectorSource, CompliantMaterial,
                         CompliantContactModelParameters, DrakeVisualizer,
                         AddFlatTerrainToWorld, LogOutput)

from pydrake.multibody.rigid_body_tree import (FloatingBaseType,
                                               RigidBodyFrame, RigidBodyTree)
from underactuated import (FindResource)
from pydrake.lcm import DrakeLcm
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer

#Settings
duration = 3.0

#Create Plant
tree = RigidBodyTree(FindResource("diff_drive.urdf"),
                     FloatingBaseType.kRollPitchYaw) #Use same floating base as 2D quadrotor, or perhaps 3D, kRollPitchYaw will likely be best
AddFlatTerrainToWorld(tree) #see simulate.py from rimless wheel example for sloped ground
tree.compile()
plant = RigidBodyPlant(tree)
context = plant.CreateDefaultContext()
#context = simulator.get_mutable_context()
builder = DiagramBuilder()
diff_drive = builder.AddSystem(plant)

#context.SetContinuousState(x0) ###COMMENTED OUT DUE TO RUNTIME ERROR; MAY NEED TO RE-ADD

#Setup Drake visualizer
lcm = DrakeLcm()
visualizer = builder.AddSystem(DrakeVisualizer(tree=tree, lcm=lcm,
                                               enable_playback=True))
builder.Connect(diff_drive.get_output_port(0), visualizer.get_input_port(0))

# Zero inputs -- passive forward simulation
u0 = ConstantVectorSource(np.zeros(tree.get_num_actuators()))
x0 = np.asarray([0.0]*tree.get_num_positions()*2) #np.zeros(tree.get_num_positions()*2) #initializes zeros for all states and velocities
x0[2] = 0.5 #set z above 0
x0[3] = 0.1
#print('n_actuators',tree.get_num_actuators())
#print('inspect.getmembers(tree)',inspect.getmembers(tree))
#print('n_states',tree.get_num_positions())
null_controller = builder.AddSystem(u0)
builder.Connect(null_controller.get_output_port(0), diff_drive.get_input_port(0))

#Data Logger
logger = LogOutput(diff_drive.get_output_port(0), builder)

#Run Simulation
diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
#simulator.Initialize()
#diff_drive.set_state_vector(simulator.get_mutable_context(), x0) #Old failed attempt to set initial state
#print('x0',x0)
state = simulator.get_mutable_context().\
    get_mutable_continuous_state_vector()
state.SetFromVector(x0)

simulator.StepTo(duration)

print('sample_times',logger.sample_times()) #nx1 array
print('Final State: ',logger.data()[:,-1]) #nxm array
print('z evolution: ',logger.data()[2,:]) #nxm array
print('roll evolution: ',logger.data()[3,:])
print('pitch evolution: ',logger.data()[4,:])
print('yaw evolution: ',logger.data()[5,:])

def diff_drive_pd_controller(x):
    #Get statte derivatives from pydrake
    theta = 0.
    theta_dot = 0.

    #Create control Inputs
    kp = 1.0
    kd = kp / 10.
    upright_state = [0., 0.] #[theta, theta_dot]
    u = kp * (upright_state[0] - theta + kd * (upright_state[1] - theta_dot))

    return u

def lqr_controller(x):

    return u

'''
To dos: 5/8/19
X Figure out how to set initial state
X Read documentation / see all class examples for visualizer
X Make attempt(s) at visualizer
X Set flat ground
- Figure out how to implement more creative controller
X Test different initial states and figure out what each state variable is doing
NEXT STEPS
- Implement PD controller
- Implement LQR controller
- Examine ROA for PD and LQR controllers
- Test various swing-up controllers
- Implement trajectory tracking (e.g. to do the limbo)
- Test controller on varying slopes
- Test controller on rough terrain


Define upright state
#State = q = [x, theta, psi, x_dot, theta_dot, psi_dot]
def UprightState():
    state = (*, 0, *, 0, 0, 0)
    return state
'''
'''
#Questions 5/6/19
- How to set initial state?
    - Look at Cassie python example
- With Meshcat, how do we register geometry and connect ports?
    - Use Drake Visualizer; follow Cassie example
- Why is visualizer online not pulling anything up? Is it possible to visualize on local machine?
    - With Drake Visualizer, you have to run a separate process from the controlling script that runs the visualizer
- How do you set input and output ports?
    -
- How to connect controller?
    - Could use function, but would need to define a leaf system (read up, Mark will try slacking)
    - Drake has primitive blocks for basic controllers available
- Where to put data logger in code and view results?


#Questions 4/28/19
- How to build plant of 3D system? X Same as 2D plant
- How to visualize a URDF? X Use visualizer script
- Does URDF need masses? X Yes - follow format from Cassie urdf
- Does Drake automatically find dynamics, or should we program a function to do so? X Dynamics come from simulator automatically, based on URDF
- How to integrate control inputs? X In full Cassie Simulation - change input from zero to not-zero
- How to print data from the simulation? X Signal logger - Mark sent link
- Should "floating base type" change for wheeled robot? X Yes (?) - Look at Cassie
'''
