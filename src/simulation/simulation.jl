module simulation

include("Six_axis_robotic_arm.jl")
include("trajectory_planning/trajectory_planning.jl")
include("inverse_kinematics/inverse_kinematics.jl")

using .Six_axis_robotic_arm: *
using .trajectory_planning: *
using .inverse_kinematics: *
export *

end