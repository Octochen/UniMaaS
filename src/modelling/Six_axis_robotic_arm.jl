export Six_axis_robotic_arm

module Six_axis_robotic_arm

using Pkg

using CoordinateTransformations: Translation
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics

# using RigidBodyDynamics.OdeIntegrators
# function my_simulate(state0::MechanismState{X}, final_time, control! = zero_torque!;
#         Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X
#     T = cache_eltype(state0)
#     result = DynamicsResult{T}(state0.mechanism)
#     control_torques = similar(velocity(state0))
#     closed_loop_dynamics! = let result=result, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
#         function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
#             control!(control_torques, t, state)
#             dynamics!(result, state, control_torques; stabilization_gains=stabilization_gains)
#             copyto!(v̇, result.v̇)
#             copyto!(ṡ, result.ṡ)
#             nothing
#         end
#     end
#     tableau = runge_kutta_4(T)
#     storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
#     integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
#     integrate(integrator, final_time, Δt)
#     storage.ts, storage.qs, storage.vs
# end

function robot_arm_initial()
    urdf = joinpath(@__DIR__, "urdf", "follower", "follower.urdf")
    # urdf = joinpath(@__DIR__, "src", "modelling", "urdf", "follower", "follower.urdf")
    robot = parse_urdf(Float64, urdf)
    remove_fixed_tree_joints!(robot)
    state = MechanismState(robot, zeros(6), zeros(6))
    
    function s_control!(torques::AbstractVector, t, state::MechanismState)
        # torques[velocity_range(state, joints(robot)[1])] .= -1 .* velocity(state, joints(robot)[1])
        # torques[velocity_range(state, joints(robot)[1])] .= 0
        torques .= 0
    end;

    # function time_optimal_control!(torques::AbstractVector, t, state::MechanismState)
        
    # end
    function zero_torque!(torques::AbstractVector, t, state::MechanismState)
        torques .= 0
    end
    t, q, v = simulate(state, 20.0);
    mvis = MechanismVisualizer(robot, URDFVisuals(urdf))
    return t, q, mvis

end

end