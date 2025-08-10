export Six_axis_robotic_arm

module Six_axis_robotic_arm

using Pkg

using CoordinateTransformations: Translation
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics

vis = MeshCat.Visualizer()
# open(vis)  # open the visualizer in a separate tab/window
render(vis) # render the visualizer here inside the jupyter notebook

urdf = joinpath(@__DIR__, "urdf", "follower.urdf")
robot = parse_urdf(urdf)
delete!(vis)
mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
set_configuration!(mvis, [0.0, 0.0])

# state = MechanismState(robot, randn(2), randn(2))
# t, q, v = simulate(state, 5.0);
return robot
end