using Pkg
Pkg.add(url="https://github.com/Octochen/UniMaaS")
using MeshCat, MeshCatMechanisms, RigidBodyDynamics, StaticArrays, CoordinateTransformations, Rotations 

using UniMaaS
using UniMaaS.simulation.joint_controller.time_optimal_joint_controller

vis = Visualizer()
open(vis)
robot, state, mvis = UniMaaS.simulation.Six_axis_robotic_arm.Koch_simulation_model(vis)
end_effector = findbody(robot, "link_6")

# Applying control strategies to obtain optimal trajectories
function one_task_period(
    state::RigidBodyDynamics.MechanismState,
    pf::AbstractVector{<:Real}
    )
    t1, q1 = time_optimal_joint_trajectory(state, 0.0, [0.0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t2, q2 = time_optimal_joint_trajectory(state, t1[end], [0.0, 0, 0, 0, 0, -0.5], [0, 0, 0, -1.6, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t3, q3 = time_optimal_joint_trajectory(state, t2[end], [0.0, 0, 0, -1.6, 0, -0.5], [0, 0, 0, -1.6, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t4, q4 = time_optimal_joint_trajectory(state, t3[end], [0, 0, 0, -1.6, 0, -0.2], [0, 0, 0, 0, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t5, q5 = time_optimal_joint_trajectory(state, t4[end], [0, 0, 0, 0, 0, -0.2], pf.+[0, 0, 0, 0, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t6, q6 = time_optimal_joint_trajectory(state, t5[end], pf.+[0, 0, 0, 0, 0, -0.2], pf.+[0, 0, 0, -1.6, 0, -0.2], 0.3*ones(6), 0.3*ones(6), 0.1)
    t7, q7 = time_optimal_joint_trajectory(state, t6[end], pf.+[0, 0, 0, -1.6, 0, -0.2], pf.+[0, 0, 0, -1.6, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t8, q8 = time_optimal_joint_trajectory(state, t7[end], pf.+[0, 0, 0, -1.6, 0, -0.5], pf.+[0.0, 0, 0, 0, 0, -0.5], 0.3*ones(6), 0.3*ones(6), 0.1)
    t9, q9 = time_optimal_joint_trajectory(state, t8[end], pf.+[0, 0, 0, 0, 0, -0.5], pf.+[0.0, 0, 0, 0, 0, 0], 0.3*ones(6), 0.3*ones(6), 0.1)
    t10, q10 = time_optimal_joint_trajectory(state, t9[end], pf.+[0.0, 0, 0, 0, 0, 0], [0.0, 0, 0, 0, 0, 0], 0.3*ones(6), 0.3*ones(6), 0.1)

    ts = vcat(t1, t2, t3, t4, t5, t6, t7, t8, t9, t10)
    qs = vcat(q1, q2, q3, q4, q5, q6, q7, q8, q9, q10)

    t_ball = ts
    q_ball = Vector{Vector{Float64}}()
    
    set_configuration!(state, [0.0, 0, 0, -1.6, 0, 0])
    transform = transform_to_root(state, end_effector)
    pos = translation(transform)
    rot = rotation(transform)
    T_end_to_world_initial = AffineMap(rot, pos)
    end_effector_center = @SVector [-0.05, 0.006, -0.01]
    end_effector_center_world_initial = T_end_to_world_initial(end_effector_center)
    for _ in 1:length(vcat(q1, q2, q3))
        push!(q_ball, vcat([1.0, 0.0, 0.0, 0.0], end_effector_center_world_initial))
    end

    for q in vcat(q4, q5, q6, q7)
        set_configuration!(state, q)
        transform = transform_to_root(state, end_effector)
        pos = translation(transform)
        rot = rotation(transform)
        T_end_to_world = AffineMap(rot, pos)
        end_effector_center = @SVector [-0.05, 0.006, -0.01]
        end_effector_center_world = T_end_to_world(end_effector_center)
        push!(q_ball, vcat([1.0, 0.0, 0.0, 0.0], end_effector_center_world))
    end

    for _ in 1:length(vcat(q8, q9, q10))
        push!(q_ball, vcat([1.0, 0.0, 0.0, 0.0], end_effector_center_world_initial))
    end

    return ts, qs, t_ball, q_ball
end

MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 2.0)



# ts, qs, t_ball, q_ball = one_task_period(state, [2.0, 0, 0, 0, 0, 0])

# set_configuration!(state, [0.0, 0, 0, -1.6, 0, 0])
# transform = transform_to_root(state, end_effector)
# pos = translation(transform)
# rot = rotation(transform)
# T_end_to_world_initial = AffineMap(rot, pos)
# end_effector_center = @SVector [-0.05, 0.006, -0.01]
# end_effector_center_world_initial = T_end_to_world_initial(end_effector_center)
# ball, state_ball, mvis_ball = UniMaaS.simulation.Six_axis_robotic_arm.redball_simulation_model(vis, vcat([1.0, 0.0, 0.0, 0.0], end_effector_center_world_initial))

# animation = Animation(vis["/ball"])
# function update_frame(vis::Visualizer, pos::SVector{3,Float64}, rot::RotMatrix{3,Float64})
#     tf = AffineMap(rot, pos)
#     settransform!(vis, tf)
# end

# for (i, q) in enumerate(qs[ball_s: ball_f])
#     atframe(animation, i) do
#         set_configuration!(state, q)
#         transform = transform_to_root(state, end_effector)
#         pos = translation(transform)
#         rot = rotation(transform)
#         update_frame(vis["/ball"], pos, rot)
#     end
# end


# setanimation!(vis["/ball"], animation)



# MeshCatMechanisms.animate(mvis_ball, ts, q_ball; realtimerate = 2.0)