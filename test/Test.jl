using Pkg
Pkg.add(url="https://github.com/Octochen/UniMaaS")
using MeshCat, MeshCatMechanisms, RigidBodyDynamics

using UniMaaS
using UniMaaS.simulation.joint_controller.time_optimal_joint_controller

robot, state, mvis = UniMaaS.simulation.Six_axis_robotic_arm.Koch_simulation_model()
ball, state_ball, mvis_ball = UniMaaS.simulation.Six_axis_robotic_arm.redball_simulation_model()

function one_task_period(
    state::RigidBodyDynamics.MechanismState,
    pf::AbstractVector{<:Real}
    )
    t1, q1 = time_optimal_joint_trajectory(state, [0.0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t2, q2 = time_optimal_joint_trajectory(state, [0.0, 0, 0, 0, 0, -0.5], [0, 0, 0, -1.6, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t3, q3 = time_optimal_joint_trajectory(state, [0.0, 0, 0, -1.6, 0, -0.5], [0, 0, 0, -1.6, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t4, q4 = time_optimal_joint_trajectory(state, [0, 0, 0, -1.6, 0, -0.2], [0, 0, 0, 0, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t5, q5 = time_optimal_joint_trajectory(state, [0, 0, 0, 0, 0, -0.2], pf.+[0, 0, 0, 0, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t6, q6 = time_optimal_joint_trajectory(state, pf.+[0, 0, 0, 0, 0, -0.2], pf.+[0, 0, 0, -1.6, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t7, q7 = time_optimal_joint_trajectory(state, pf.+[0, 0, 0, -1.6, 0, -0.2], pf.+[0, 0, 0, -1.6, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t8, q8 = time_optimal_joint_trajectory(state, pf.+[0, 0, 0, -1.6, 0, -0.5], pf.+[0.0, 0, 0, 0, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t9, q9 = time_optimal_joint_trajectory(state, pf.+[0, 0, 0, 0, 0, -0.5], pf.+[0.0, 0, 0, 0, 0, 0], 0.3*ones(6), 0.3*ones(6), 0.1)
    t10, q10 = time_optimal_joint_trajectory(state, pf.+[0.0, 0, 0, 0, 0, 0], [0.0, 0, 0, 0, 0, 0], 0.3*ones(6), 0.3*ones(6), 0.1)

    ts = copy(t1)
    offset = t1[end]
    for t in [t2, t3, t4, t5, t6, t7, t8, t9, t10]
        append!(ts, t .+ offset)
        offset += t[end]
    end
    qs = vcat(q1, q2, q3, q4, q5, q6, q7, q8, q9, q10)
    return ts, qs
end

# Applying control strategies to obtain optimal trajectories

ts, qs = one_task_period(state, [2.0, 0, 0, 0, 0, 0])

# MeshCatMechanisms.animate(mvis, t1, q1; realtimerate = 1.)
open(mvis_ball)
MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 2.)