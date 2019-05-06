'''This script loads the diff_drive plant and dynamics, and implements
a basic controller

This is a 2D controller
'''

import math
import numpy as np
import matplotlib.pyplot as plt

from pydrake.all import (DirectCollocation, FloatingBaseType,
                         PiecewisePolynomial, RigidBodyTree, RigidBodyPlant,
                         SolutionResult)

#The following imports come from Mark Petersen's quadrotor example
from pydrake.all import (ConstantVectorSource, DiagramBuilder, FramePoseVector,
                         MeshcatVisualizer, SceneGraph, Simulator)
from pydrake.all import (MultibodyPlant, Parser)


from underactuated import (FindResource, PlanarRigidBodyVisualizer)

#Create Plant
tree = RigidBodyTree(FindResource("diff_drive/diff_drive.urdf"),
                     FloatingBaseType.kRollPitchYaw) #Use same floating base as 2D quadrotor, or perhaps 3D, kRollPitchYaw will likely be best
plant = RigidBodyPlant(tree)
context = plant.CreateDefaultContext()

builder = DiagramBuilder()
diff_drive = builder.AddSystem(RigidBodyPlant(tree))
diagram = builder.Build()

context = simulator.get_mutable_context()
x0 = np.zeros((4,1))
context.SetContinuousState(x0)

#Setup Controller

#Setup Visualization

#Run Simulation
simulator = Simulator(diagram)
simulator.set_target_realtime_rate(1.0)
simulator.Initialize()
simulator.StepTo(1.0)



'''
Define upright state
#State = q = [x, theta, psi, x_dot, theta_dot, psi_dot]
def UprightState():
    state = (*, 0, *, 0, 0, 0)
    return state
'''


'''
#Steps = 5/6/19
1. Add masses to URDF (following Cassie) X
2. Write simulator code (leveraging Mark's version)
3. Run simulator & fix bugs
4. Add signal logger to grab data from simulator

#Questions 5/6/19


#Questions 4/28/19
- How to build plant of 3D system? X Same as 2D plant
- How to visualize a URDF? X Use visualizer script
- Does URDF need masses? X Yes - follow format from Cassie urdf
- Does Drake automatically find dynamics, or should we program a function to do so? X Dynamics come from simulator automatically, based on URDF
- How to integrate control inputs? X In full Cassie Simulation - change input from zero to not-zero
- How to print data from the simulation? X Signal logger - Mark sent link
- Should "floating base type" change for wheeled robot? X Yes (?) - Look at Cassie
'''
