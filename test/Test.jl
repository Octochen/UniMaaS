# using Pkg
# Pkg.add(url="https://github.com/Octochen/UniMaaS")
using MeshCat, MeshCatMechanisms, RigidBodyDynamics

using UniMaaS
using UniMaaS.simulation.joint_controller.time_optimal_joint_controller

robot, state, mvis = UniMaaS.simulation.Six_axis_robotic_arm.Koch_simulation_model()

# Applying control strategies to obtain optimal trajectories
t1, q1 = time_optimal_joint_trajectory(state, [0, 0, 0, -1.6, 0, 0], [4, 0, 0, -1.6, 0, -0.5], 0.2*ones(6), 0.2*ones(6), 0.1)
t2, q2 = time_optimal_joint_trajectory(state, [4, 0, 0, -1.6, 0, -0.5], [4, 0, 0, -1.6, 0, 0], 0.2*ones(6), 0.2*ones(6), 0.1)
t3, q3 = time_optimal_joint_trajectory(state, [4, 0, 0, -1.6, 0, 0], [0, 0, 0, -1.6, 0, 0], 0.2*ones(6), 0.2*ones(6), 0.1)
t4, q4 = time_optimal_joint_trajectory(state, [0, 0, 0, -1.6, 0, 0], [0, 0, 0, -1.6, 0, -0.5], 0.2*ones(6), 0.2*ones(6), 0.1)

# MeshCatMechanisms.animate(mvis, t1, q1; realtimerate = 1.);
MeshCatMechanisms.animate(mvis, vcat(t1, t2.+t1[end], t3.+t1[end].+t2[end], t4.+t1[end].+t2[end].+t3[end]), vcat(q1,q2,q3,q4); realtimerate = 1.);

MeshCatMechanisms.animate(mvis, vcat(t1, t2.+t1[end], t3.+t1[end].+t2[end], t4.+t1[end].+t2[end].+t3[end]), vcat(q1,q2,q3,q4); realtimerate = 1.);