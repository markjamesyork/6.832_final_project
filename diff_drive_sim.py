'''This script loads the diff_drive plant and dynamics, and implements
a basic controller

This is a 2D controller

QUESTIONS:
1. Why is meshcat breaking at "RegisterGeometry"?
2. Why can't we wire up controller?
3. How to set controller in system?
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
                         AddFlatTerrainToWorld, LogOutput, MultibodyPlant,
                         Parser, UniformGravityFieldElement)

from pydrake.multibody.rigid_body_tree import (FloatingBaseType,
                                               RigidBodyFrame, RigidBodyTree)
from underactuated import (FindResource)
#Visualtion imports
from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.geometry import SceneGraph
from underactuated.planar_multibody_visualizer import PlanarMultibodyVisualizer

class Diff_Drive_Controller(VectorSystem):
    def __init__(self, plant, print_period = 0.0):
        VectorSystem.__init__(self, plant.num_positions() + plant.num_velocities(), plant.num_actuators())
        print("n_inputs", plant.num_positions() + plant.num_velocities())
        print('n_actuators',plant.num_actuators())
        # 6 inputs (from the 6 outputs of the plant)
        # 3 outputs (for the three actuators on the plant)
        self.plant = plant
        self.print_period = print_period
        self.last_print_time = -print_period
        u0 = [0.]
        print('init')
        print('VectorSystem',VectorSystem)

    def _DoCalcVectorOutput(self, context, plant_state_vec, controller_state_vec, output_vec):
        print('_DoCalcVectorOutput')
        if (self.print_period and
            context.get_time() - self.last_print_time >= self.print_period):
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()

        q = plant_state_vec[:] # subtract of fixed values
        v = plant_state_vec[:]

        output_vec[:] = np.zeros(self.plant.get_input_size())
        output_vec[0] = [1.] # add constant torque of 1
        u0 = [0.]
#This is running in RigidBodyTree; we need to run in MultibodyPlant
#Use 2d planar hopper 20Model, lines 188-204

#Settings
duration = 1.0

#Setup simulator elements
builder = DiagramBuilder()
plant = builder.AddSystem(MultibodyPlant(0.0005))
scene_graph = builder.AddSystem(SceneGraph())
plant.RegisterAsSourceForSceneGraph(scene_graph)
builder.Connect(plant.get_geometry_poses_output_port(),
                scene_graph.get_source_pose_port(
                    plant.get_source_id()))
builder.Connect(scene_graph.get_query_output_port(),
                plant.get_geometry_query_input_port())

#Setup plant
parser = Parser(plant)
parser.AddModelFromFile("diff_drive.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"))
plant.AddForceElement(UniformGravityFieldElement())
plant.Finalize()

#context = plant.CreateDefaultContext()
#diff_drive = builder.AddSystem(plant)

# Set up visualization in MeshCat
'''
#Meshcat Visualization
scene_graph = builder.AddSystem(SceneGraph())
plant.RegisterGeometry(scene_graph) #Commented out due to runtime error; may end up needing this
builder.Connect(plant.get_geometry_pose_output_port(),
                scene_graph.get_source_pose_port(plant.source_id()))
meshcat = builder.AddSystem(MeshcatVisualizer(
    scene_graph, zmq_url=None,
    open_browser=False))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                meshcat.get_input_port(0))
'''
visualizer = builder.AddSystem(PlanarMultibodyVisualizer(scene_graph,
                                                      xlim=[-1., 1.],
                                                      ylim=[-1., 1.]))
builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))


# Zero inputs -- passive forward simulation
#print('inspect.getmembers(plant)',inspect.getmembers(plant))
#u0 = ConstantVectorSource(np.zeros(plant.num_actuators()))
x0 = np.asarray([0.0]*plant.num_positions()*2) #np.zeros(tree.get_num_positions()*2) #initializes zeros for all states and velocities
x0[2] = 0.5 #set z above 0
x0[3] = 0.1
print('n_actuators',plant.num_actuators())
print('n_states',plant.num_positions())

#null_controller = builder.AddSystem(u0)
#builder.Connect(null_controller.get_output_port(0), plant.get_input_port(0))
#Controller as in lqr.py for 3d quadrotor
controller = builder.AddSystem(Diff_Drive_Controller(plant))
builder.Connect(plant.get_continuous_state_output_port(), controller.get_input_port(0))
builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

#Data Logger
logger = LogOutput(plant.get_continuous_state_output_port(), builder)

#Run Simulation
diagram = builder.Build()
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.Initialize()
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

def diff_drive_pd(x, target_state):
    #Create control Inputs
    kp = 1.0
    kd = kp / 10.
    theta = x[3] #Must be verified
    theta_dot = x[12] #Must be verified
    upright_state = [0., 0.] #[theta, theta_dot]
    u = kp * (upright_state[0] - theta + kd * (upright_state[1] - theta_dot))

    return u

def lqr_controller(x):
    actuator_limit: 100. #must determine limits
    A = np.zeros((2,2))
    B = np.zeros((1,1))
    Q = np.asarray([[10.,0.],[0.,1.]])
    R = np.asarray([0.]) #0 => costless control
    K, S = LinearQuadraticRegulator(A,B,Q,R)
    u = np.matmul(-K,x)
    if u[0] = np.clip(u[0], -actuator_limit, actuator_limit)
    return u

'''
To dos: 5/13/19
- Write LQR controller
X Edit PD controller
- Edit urdf
- Make list of other behaviors and improvements to make
- Get controller to work (waiting for Piazza / OH)


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
