'''This script loads the diff_drive plant and dynamics, and implements
a basic controller

This is a 2D controller

QUESTIONS:
1. Why is meshcat breaking at "RegisterGeometry"?
2. Why can't we wire up controller?
3. How to set controller in system?
'''
import inspect
import time
import argparse
import math
import numpy as np
import pandas as pd
import os.path
from pydrake.all import (DiagramBuilder, FloatingBaseType, RigidBodyPlant,
                         RigidBodyTree, Simulator, VectorSystem,
                         ConstantVectorSource, CompliantMaterial,
                         CompliantContactModelParameters, DrakeVisualizer,
                         AddFlatTerrainToWorld, LogOutput, MultibodyPlant,
                         Parser, UniformGravityFieldElement, RollPitchYaw,
                         Quaternion, LinearQuadraticRegulator,
                         MathematicalProgram, Solve)
'''
from pydrake.multibody.rigid_body_tree import (FloatingBaseType,
                                               RigidBodyFrame, RigidBodyTree)
'''
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

    def diff_drive_pd(self, x, x_des): # x_des = [x, theta, yaw, x_dot, theta_dot, yaw_dot]
        kp = 1.
        kd = kp / 30.
        ky = .05
        actuator_limit = .05 #must determine limits
        u = np.zeros((self.plant.num_actuators()))
        theta = math.asin(2*(x[0]*x[2] - x[1]*x[3])) #pitch
        phi = np.arctan2(2*(x[0]*x[1] + x[2]*x[3]), 1-2*(x[1]**2 + x[2]**2))#yaw
        theta_dot = x[10] #Shown to be ~1.5% below true theta_dot on average in experiments
        u[0] = kp * (x_des[1] - theta + kd * (x_des[4] - theta_dot)) + ky * (x_des[2] - phi)
        u[0] = np.clip(u[0],-actuator_limit,actuator_limit)
        u[1] = -u[0]
        #print('u',u)
        return u

    def diff_drive_pd_mp(self, x, x_des): # x_des = [x, theta, yaw, x_dot, theta_dot, yaw_dot]
        m_s = 0.2 #kg
        d = 0.085 #m
        m_c = 0.056
        I_3 = 0.000228373 #.0000989844 #kg*m^2
        I_2 = .0000989844
        R = 0.0333375
        g = -9.81 #may need to be set as -9.81; test to see
        L = 0.03
        A_val_theta = (m_s*d*g*(3*m_c + m_s)) / (3*m_c*I_3 + 3*m_c*m_s*d**2 + m_s*I_3)
        B_val_theta = (-m_s*d/R -3*m_c*m_s) / (3*m_c*I_3 + 3*m_c*m_s*d**2 +m_s*I_3)*math.pi/180 #multiplicand for u to change in theta_dot
        B_val_phi = -(2*L/R) / (6*m_c*L**2 + m_c*R**2 + 2*I_2)
        mp = MathematicalProgram()
        k = mp.NewContinuousVariables(3, 'k_val') # [kp, kd, ky]
        u = mp.NewContinuousVariables(2,'u_val')
        theta_dot_post = mp.NewContinuousVariables(1,'theta_dot_post_val') #estimated value of theta dot after control
        phi_dot_post = mp.NewContinuousVariables(1,'phi_dot_post_val') #estimated value of theta dot after control
        '''
        kp = 1. #regulate theta (pitch) position
        kd = kp / 20. #regulate theta (pitch) velocity
        ky = 0.5 #regulate phi (yaw) position
        '''
        actuator_limit = .1 #estimate; 0.1 is probably high
        #mp.AddConstraint(theta[0] == np.arcsin(2*(x[0]*x[2] - x[1]*x[3]))) #pitch
        theta = np.arcsin(2*(x[0]*x[2] - x[1]*x[3]))
        phi = np.arctan2(2*(x[0]*x[1] + x[2]*x[3]), 1-2*(x[1]**2 + x[2]**2))#yaw
        theta_dot = x[10] #Shown to be ~1.5% below true theta_dot on average in experiments
        mp.AddConstraint(k[0] >= 0.)
        mp.AddConstraint(k[1] >= 0.)
        mp.AddConstraint(k[2] >= 0.)
        mp.AddConstraint(u[0] == k[0] * (x_des[1] - theta + k[1] * (x_des[4] - theta_dot)) + k[2] * (x_des[2] - phi))
        mp.AddConstraint(u[0] <= -actuator_limit)
        mp.AddConstraint(u[0] >= -actuator_limit)
        mp.AddConstraint(u[1] == -u[0])
        mp.AddConstraint(theta_dot_post[0] == theta_dot + B_val_theta*u[0]*0.0005 + A_val_theta*theta*.0005)
        mp.AddConstraint(phi_dot_post[0] == x[11] + B_val_phi*u[0]*.0005)
        theta_dot_des = -(theta - x_des[1])/(2*.0005)
        mp.AddQuadraticCost((theta_dot_post[0] - theta_dot_des)**2)
        phi_dot_des = -(phi - x_des[2])/2
        print('theta_dot_des',theta_dot_des)
        print('theta',theta)
        mp.AddQuadraticCost(0.001*(phi_dot_post[0] - phi_dot_des)**2)

        result = Solve(mp)
        print('Success: ',result.is_success())
        print('Solver id: ',result.get_solver_id().name())
        print('k: ',result.GetSolution(k))
        print('u: ',result.GetSolution(u))
        return result.GetSolution(u)

    def lqr_controller(self, x, x_des):
        #Robot parameters manually set according to actual measurements on 5/13/19
        m_s = 0.2 #kg
        d = 0.085 #m
        m_c = 0.056
        I_3 = 0.000228373 #.0000989844 #kg*m^2
        R = 0.0333375
        g = -9.81 #may need to be set as -9.81; test to see

        u = np.zeros((self.plant.num_actuators()))
        theta = math.asin(2*(x[0]*x[2] - x[1]*x[3]))

        theta_dot = x[10] #Shown to be ~1.5% below true theta_dot on average in experiments

        x_mod = np.asarray([x[4]-x_des[0],theta-x_des[1],x[12]-x_des[2],theta_dot-x_des[3]]) #[x, theta, x_dot, theta_dot] - need to verify positions of theta and theta_dot
        #Assumes that all zeros is optimal state; may add optimal_state variable and subtract from current state to change
        actuator_limit = .05 #must determine limits
        A = np.zeros((4,4))
        A[0,2] = 1.
        A[1,3] = 1.
        A[2,1] = (m_s**2 * d**2 * g) / (3*m_c*I_3 + 3*m_c*m_s*d**2 + m_s*I_3) * 180 / math.pi
        A[3,1] = (m_s*d*g*(3*m_c + m_s)) / (3*m_c*I_3 + 3*m_c*m_s*d**2 + m_s*I_3)
        B = np.zeros((4,1))
        B[2,0] = (-(m_s*d**2 + I_3)/R - m_s*d) / (3*m_c*I_3 + 3*m_c*m_s*d**2 +m_s*I_3)
        B[3,0] = (-m_s*d/R -3*m_c*m_s) / (3*m_c*I_3 + 3*m_c*m_s*d**2 +m_s*I_3)*math.pi/180
        Q = np.zeros((4,4))
        Q[0,0] = 3.
        Q[1,1] = .1
        Q[2,2] = .1
        Q[3,3] = 4.
        R = np.asarray([1.]) #0 => costless control
        K, S = LinearQuadraticRegulator(A,B,Q,R)
        #print('A',A)
        #print('B',B)
        #print('K',K)
        u[0] = np.matmul(K,x_mod)
        u[0] = np.clip(u[0], -actuator_limit, actuator_limit)
        u[1] = -u[0]
        #print('u',u)
        return u

    def DoCalcVectorOutput(self, context, plant_state_vec, controller_state_vec, output_vec):
        if (self.print_period and
            context.get_time() - self.last_print_time >= self.print_period):
            print "t: ", context.get_time()
            self.last_print_time = context.get_time()

        x = plant_state_vec[:] # subtract of fixed values
        x_des = np.zeros((6,)) #[x, theta, yaw, x_dot, theta_dot, yaw_dot]
        x_des[3] = 0.

        output_vec[:] = np.zeros(self.plant.num_actuators())
        #control = np.zeros((2)) #null controller
        #control = self.diff_drive_pd(x, x_des)
        control = self.lqr_controller(x, x_des)
        output_vec[0] = control[0] #.00005 # add constant torque in newton meters
        output_vec[1] = control[1]

#Settings
duration = 3.

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
parser = Parser(plant, scene_graph)
parser.AddModelFromFile("diff_drive_real.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("ground"))
'''#Uncomment these three lines if using a urdf with obstacles
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("obstacle_1"))
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("obstacle_2"))
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("obstacle_3"))
'''
plant.AddForceElement(UniformGravityFieldElement())
plant.Finalize()

#context = plant.CreateDefaultContext()
#diff_drive = builder.AddSystem(plant)

# Set up visualization in MeshCat
#Meshcat Visualization
#scene_graph = builder.AddSystem(SceneGraph())
#plant.RegisterGeometry(scene_graph) #Commented out due to runtime error; may end up needing this
#builder.Connect(plant.get_continuous_state_output_port(),
#                scene_graph.get_pose_bundle_output_port())
#builder.Connect(plant.get_geometry_pose_output_port(),
#                scene_graph.get_source_pose_port(plant.source_id()))
meshcat = builder.AddSystem(MeshcatVisualizer(scene_graph))
builder.Connect(scene_graph.get_pose_bundle_output_port(),
                meshcat.get_input_port(0))
'''

visualizer = builder.AddSystem(PlanarMultibodyVisualizer(scene_graph,
                                                      xlim=[-1., 1.],
                                                      ylim=[-1., 1.],
                                                      facecolor='blue',
                                                      Tview=np.array([[0., 1., 0., 0.],
                                                                      [0., 0., 1., 0.],
                                                                      [0., 0., 0., 1.]])))
builder.Connect(scene_graph.get_pose_bundle_output_port(), visualizer.get_input_port(0))
'''

# Zero inputs -- passive forward simulation
#print('inspect.getmembers(plant)',inspect.getmembers(plant))
#u0 = ConstantVectorSource(np.zeros(plant.num_actuators()))
theta0 = math.pi/12.
x0 = np.zeros((plant.num_positions()*2-1)) #np.zeros(tree.get_num_positions()*2) #initializes zeros for all states and velocities
x0[0] = math.cos(theta0/2) # q0 or qw
x0[1] = 0 # q1 or qx
x0[2] = math.sin(theta0/2) # q2 or qy
x0[3] = 0. # q3 or qz
x0[4] = 0. # x
x0[5] = 0. # y
x0[6] = .0433371122 # z on ground = .0433371122
x0[7] = 0 # right wheel
x0[8] = 0 # left wheel
x0[12] = 0. #x_dot

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
simulator.set_target_realtime_rate(1.)

plant_context = diagram.GetMutableSubsystemContext(
    plant, simulator.get_mutable_context())
print plant_context.get_mutable_discrete_state_vector()
plant_context.get_mutable_discrete_state_vector().SetFromVector(x0)

simulator.Initialize()
simulator.StepTo(duration)

#print('sample_times',logger.sample_times()) #nx1 array
print('Final State: ',logger.data()[:,-1]) #nxm array
x_log = logger.data()[4,:]
data = logger.data()
theta_log = np.asarray([math.asin(2*(data[0,i]*data[2,i] - data[1,i]*data[3,i])) for i in range(data.shape[1])])
#theta = math.asin(2*(x[0]*x[2] - x[1]*x[3]))
print('Cost: ',np.matmul(theta_log,theta_log.T))

'''
#print('x evolution: ',logger.data()[1,:]) #nxm array
#print('y evolution: ',logger.data()[2,:]) #nxm array
#print('z evolution: ',logger.data()[3,:]) #nxm array
#print('roll evolution: ',logger.data()[4,:])
#print('pitch evolution: ',logger.data()[5,:])
#print('yaw evolution: ',logger.data()[6,:])
'''

#Printing logger data to file
df = pd.DataFrame(logger.data())
df.to_csv('mbp_logger.csv')

'''
Questions 5/14/19:
X What is wrong with urdf?
X What is wrong with visualizer?
X How best to determine which state is which?
X How best to set ground?
- Will controller(s) work?


Define upright state
#State = q = [x, theta, psi, x_dot, theta_dot, psi_dot]
def UprightState():
    state = (*, 0, *, 0, 0, 0)
    return state
'''
'''
#Questions 5/13/19
- How to set I3?
- How to get derivatives from Drake to build A and B matrices?
- Where in controller is input set?


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
