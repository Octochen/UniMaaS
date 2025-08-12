module simulation

include("Six_axis_robotic_arm.jl")
include("joint_controller/joint_controller.jl")

using .Six_axis_robotic_arm
using .joint_controller
export *

end