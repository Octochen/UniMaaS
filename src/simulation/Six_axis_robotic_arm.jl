module Six_axis_robotic_arm
import MeshCat, MeshCatMechanisms, RigidBodyDynamics
export Koch_simulation_model, redball_simulation_model

function Koch_simulation_model(vis::MeshCatMechanisms.MechanismVisualizer)
    urdf = joinpath(@__DIR__, "urdf", "Koch_v1.1", "Koch_v1.1.urdf")
    # urdf = joinpath(@__DIR__, "src", "simulation", "urdf", "Koch_v1.1", "Koch_v1.1.urdf")
    robot = RigidBodyDynamics.parse_urdf(Float64, urdf)
    RigidBodyDynamics.remove_fixed_tree_joints!(robot)
    state = RigidBodyDynamics.MechanismState(robot, zeros(6), zeros(6))
    mvis = MeshCatMechanisms.MechanismVisualizer(robot, MeshCatMechanisms.URDFVisuals(urdf), vis["/robot"])
    return robot, state, mvis
end

function redball_simulation_model(vis::MeshCatMechanisms.MechanismVisualizer, q_ball::AbstractVector{<:Real})
    
    urdf = joinpath(@__DIR__, "urdf", "ball", "redball.urdf")
    # urdf = joinpath(@__DIR__, "src", "simulation", "urdf", "ball", "redball.urdf")
    redball = RigidBodyDynamics.parse_urdf(Float64, urdf)
    state_ball = RigidBodyDynamics.MechanismState(redball)
    RigidBodyDynamics.set_configuration!(state_ball, q_ball)
    mvis_ball = MeshCatMechanisms.MechanismVisualizer(redball, MeshCatMechanisms.URDFVisuals(urdf), vis["/ball"])
    MeshCatMechanisms.set_configuration!(mvis_ball, RigidBodyDynamics.configuration(state_ball))
    return redball, state_ball, mvis_ball
end

end