export Six_axis_robotic_arm

module Six_axis_robotic_arm

using Pkg

using CoordinateTransformations: Translation
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics

# function robot_arm_initial()

    urdf = joinpath(@__DIR__, "urdf", "follower", "follower.urdf")
    # urdf = joinpath(@__DIR__, "src", "modelling", "urdf", "follower", "follower.urdf")
    robot = parse_urdf(Float64, urdf)
    remove_fixed_tree_joints!(robot)
    state = MechanismState(robot, zeros(6), zeros(6))
    
    function s_control!(torques::AbstractVector, t, state::MechanismState)
        # torques[velocity_range(state, joints(robot)[1])] .= -1 .* velocity(state, joints(robot)[1])
        torques[velocity_range(state, joints(robot)[1])] .= 0
    end;

    # function time_optimal_control!(torques::AbstractVector, t, state::MechanismState)
        
    # end
    
    t, q, v = simulate(state, 20.0, s_control!);
    mvis = MechanismVisualizer(robot, URDFVisuals(urdf))

    MeshCatMechanisms.animate(mvis, t, q; realtimerate = .1);

# end

end