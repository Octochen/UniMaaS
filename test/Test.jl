using Pkg
Pkg.add(url="https://github.com/Octochen/UniMaaS")
using UniMaaS, MeshCat, MeshCatMechanisms

t, q, mvis = UniMaaS.modelling.Six_axis_robotic_arm.robot_arm_initial()
MeshCatMechanisms.animate(mvis, t, q; realtimerate = .01);