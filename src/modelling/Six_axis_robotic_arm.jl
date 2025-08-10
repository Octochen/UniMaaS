export Six_axis_robotic_arm

module Six_axis_robotic_arm
import RigidBodySim
import RigidBodyDynamics

urdf = joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf")

end