using Pkg
import MuJoCo as MJ
import LinearAlgebra as LA
import DelimitedFiles
import ControlSystems, MatrixEquations

include("./inverse_kinematics/inverse_kinematics.jl")
using .inverse_kinematics: compute_quat_error, quat_to_vel, manual_jacobian

# model = MJ.load_model(joinpath(@__DIR__, "robot_models", "franka_emika_panda", "scene.xml")) # Load a model of a robotic arm
data = MJ.init_data(model)
# MJ. mj_forward(model,data)

body_id = Vector{Int}(undef, 6)
body_id[1] = MJ.mj_name2id(model, MJ.mjOBJ_BODY, "can4")
body_id[2] = MJ.mj_name2id(model, MJ.mjOBJ_BODY, "can3")
body_id[3] = MJ.mj_name2id(model, MJ.mjOBJ_BODY, "can2")
body_id[4] = MJ.mj_name2id(model, MJ.mjOBJ_BODY, "can1")
body_id[6] = MJ.mj_name2id(model, MJ.mjOBJ_JOINT, "finger_joint1")
# @assert body_id != -1 "not find object can1"

target_positions = Dict(
    :home                   => [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
    :pick_pre1              => [1.581, 0.0, 0.0, -1.1, 0.0, 1.2, 0.0, 255.0],
    :pick_pre2              => [1.581, 0.0, 0.0, -1.28, 0.0, 1.23, -0.6, 255.0],
    :pick                   => [1.581, 0.0, 0.0, -1.28, 0.0, 1.23, -0.6, 0.0],
    :place_pre1             => [1.581, 0.0, 0.0, -0.5, 0.0, 0.5, -0.6, 0.0],
    :place_pre2             => [0.751, 0.0, 0.0, -0.5, 0.0, 0.5, -0.6, 0.0],
    :place_pre3             => [0.0, 0.0, 0.0, -0.5, 0.0, 0.5, -0.6, 0.0],
    :place_pre4             => [-0.751, 0.0, 0.0, -0.5, 0.0, 0.5, -0.6, 0.0],
    :place_pre5             => [-1.551, 0.0, 0.0, -1.2, 0.0, 1.2, 0.0, 0.0],
    :place_pre6             => [-1.551, 0.0, 0.0, -1.28, 0.0, 1.28, 0.0, 0.0],
    :place                  => [-1.551, 0.0, 0.0, -1.28, 0.0, 1.28, 0.0, 255.0],
)

function mycontroller!(model, data)
    # t = data.time  # Get the current simulation time
    # if 1.0 <= t < 2.0
    #     data.ctrl[1:8] .= target_positions[:pick_pre1]
    # elseif 2.0 <= t < 3.0
    #     data.ctrl[1:8] .= target_positions[:pick_pre2]
    # elseif 3.0 <= t < 5.0
    #     data.ctrl[1:8] .= target_positions[:pick]
    # elseif 5.0 <= t < 6.0
    #     data.ctrl[1:8] .= target_positions[:place_pre1] 
    # elseif 6.0 <= t < 7.0
    #     data.ctrl[1:8] .= target_positions[:place_pre2]
    # elseif 7.0 <= t < 8.0
    #     data.ctrl[1:8] .= target_positions[:place_pre3]
    # elseif 8.0 <= t < 9.0
    #     data.ctrl[1:8] .= target_positions[:place_pre4]
    # elseif 9.0 <= t < 10.0
    #     data.ctrl[1:8] .= target_positions[:place_pre5]
    # elseif 10.0 <= t < 11.0
    #     data.ctrl[1:8] .= target_positions[:place_pre6]
    # elseif 11.0 <= t < 12.0
    #     data.ctrl[1:8] .= target_positions[:place]
    # elseif 12.0 <= t < 13.0
    #     data.ctrl[1:8] .= [-1.551, 0.0, 0.0, -0.5, 0.0, 0.5, 0.0, 255.0]
    # else
    #     data.ctrl[1:8] .= target_positions[:home]
    # end

    t = data.time  # Get the current simulation time
    if 0.0 <= t < 2.0
        pos_can = data.xpos[body_id[1] + 1, :].+ [0.0, 0.0, 0.5]
        current_pos = data.xpos[body_id[6] + 1, :]
        pos_error = pos_can - current_pos
        # quat_can = data.xquat[body_id[1] + 1, :]
        # quat_can = [0.0, 0.0, 0.0, 1.0]  # Example quaternion for the target orientation
        # current_quat = data.xquat[body_id[6] + 1, :]
        # quat_error = compute_quat_error(current_quat, quat_can)
        # rot_error = quat_to_vel(quat_error)
        rot_error = [0.0, 0.0, 0.0]  # Ignore orientation error for now]
        task_error = [pos_error; rot_error]
        J = manual_jacobian(model, data, body_id[6])
        jacp = MJ.mj_zeros(3, model.nv)
        jacr = MJ.mj_zeros(3, model.nv)
        MJ.mj_jacBody(model, data, jacp, jacr, body_id[6]+1)
        J = [jacp; jacr]
        lambda = 0.1
        W = LA.diagm([10.0, 10.0, 1.0, 1.0, 1.0, 1.0])
        J_dls =  LA.inv(J' * W * J + lambda^2 * LA.I(34)) * J' * W
        q_min = [-π, -π/2, -π, -π/4, -π, -π/4, -π, -π]
        q_max = [π, π/2, π, π/4, π, π/4, π, π]
        q_ctl =  (J_dls * task_error) * 0.1
        data.ctrl[1:8] .= data.qpos[1:8] .+ q_ctl[1:8]
        data.ctrl[1:8] .= clamp.(data.ctrl[1:8], q_min, q_max)
    elseif 2.0 <= t < 4.0
        data.qacc .=0.0
        data.ctrl[1:8] .= target_positions[:pick_pre1]
    elseif 4.0 <= t < 6.0
        data.ctrl[1:8] .= target_positions[:pick_pre2]
    elseif 6.0 <= t < 8.0
        data.ctrl[1:8] .= target_positions[:pick]
    elseif 8.0 <= t < 10.0
        pos_can = [0.0, 0.0, 0.64].+ [0.0, 0.0, 0.5]
        current_pos = data.xpos[body_id[6] + 1, :]
        pos_error = pos_can - current_pos
        rot_error = [0.0, 0.0, 0.0]  # Ignore orientation error for now]
        task_error = [pos_error; rot_error]
        J = manual_jacobian(model, data, body_id[6])
        jacp = MJ.mj_zeros(3, model.nv)
        jacr = MJ.mj_zeros(3, model.nv)
        MJ.mj_jacBody(model, data, jacp, jacr, body_id[6]+1)
        J = [jacp; jacr]
        lambda = 0.1
        W = LA.diagm([10.0, 10.0, 1.0, 1.0, 1.0, 1.0])
        J_dls =  LA.inv(J' * W * J + lambda^2 * LA.I(34)) * J' * W
        q_min = [-π, -π/2, -π, -π/4, -π, -π/4, -π, -π]
        q_max = [π, π/2, π, π/4, π, π/4, π, π]
        q_ctl =  (J_dls * task_error) * 0.1
        data.ctrl[1:8] .= data.qpos[1:8] .+ q_ctl[1:8]
        data.ctrl[1:8] .= clamp.(data.ctrl[1:8], q_min, q_max)
    elseif 10 <= t < 12.0
        pos_can = [-0.5, -0.5, 0.64].+ [0.0, 0.0, 0.5]
        current_pos = data.xpos[body_id[6] + 1, :]
        pos_error = pos_can - current_pos
        rot_error = [0.0, 0.0, 0.0]  # Ignore orientation error for now]
        task_error = [pos_error; rot_error]
        J = manual_jacobian(model, data, body_id[6])
        jacp = MJ.mj_zeros(3, model.nv)
        jacr = MJ.mj_zeros(3, model.nv)
        MJ.mj_jacBody(model, data, jacp, jacr, body_id[6]+1)
        J = [jacp; jacr]
        lambda = 0.1
        W = LA.diagm([10.0, 10.0, 1.0, 1.0, 1.0, 1.0])
        J_dls =  LA.inv(J' * W * J + lambda^2 * LA.I(34)) * J' * W
        q_min = [-π, -π/2, -π, -π/4, -π, -π/4, -π, -π]
        q_max = [π, π/2, π, π/4, π, π/4, π, π]
        q_ctl =  (J_dls * task_error) * 0.1
        data.ctrl[1:8] .= data.qpos[1:8] .+ q_ctl[1:8]
        data.ctrl[1:8] .= clamp.(data.ctrl[1:8], q_min, q_max)
    elseif 12.0 <= t < 14.0
        data.ctrl[1:8] .= target_positions[:place_pre5]
    elseif 14.0 <= t < 16.0
        data.ctrl[1:8] .= target_positions[:place_pre6]
    elseif 16.0 <= t < 18.0
        data.ctrl[1:8] .= target_positions[:place]
    elseif 18.0 <= t < 20.0
        data.ctrl[1:8] .= [-1.551, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 255.0]
    else
        data.ctrl[1:8] .= target_positions[:home]
    end
end
MJ.init_visualiser()  # Load required dependencies into session
data.qacc .=0.0
MJ.visualise!(model, data, controller=mycontroller!) # Run the visualiser
