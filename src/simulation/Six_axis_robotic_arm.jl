export Six_axis_robotic_arm

module Six_axis_robotic_arm
import MeshCat, MeshCatMechanisms, RigidBodyDynamics

function Koch_simulation_model()
    urdf = joinpath(@__DIR__, "urdf", "Koch_v1.1", "Koch_v1.1.urdf")
    # urdf = joinpath(@__DIR__, "src", "simulation", "urdf", "Koch_v1.1", "Koch_v1.1.urdf")
    robot = RigidBodyDynamics.parse_urdf(Float64, urdf)
    RigidBodyDynamics.remove_fixed_tree_joints!(robot)
    return robot
end

end