module Six_axis_robotic_arm
import MeshCat, MeshCatMechanisms, RigidBodyDynamics
export Koch_simulation_model

function Koch_simulation_model()
    urdf = joinpath(@__DIR__, "urdf", "Koch_v1.1", "Koch_v1.1.urdf")
    # urdf = joinpath(@__DIR__, "src", "simulation", "urdf", "Koch_v1.1", "Koch_v1.1.urdf")
    robot = RigidBodyDynamics.parse_urdf(Float64, urdf)
    RigidBodyDynamics.remove_fixed_tree_joints!(robot)
    state = RigidBodyDynamics.MechanismState(robot, zeros(6), zeros(6))
    mvis = MeshCatMechanisms.MechanismVisualizer(robot, MeshCatMechanisms.URDFVisuals(urdf))
    return robot, state, mvis
end

end