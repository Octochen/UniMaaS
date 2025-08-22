module inverse_kinematics

import MuJoCo as MJ
import LinearAlgebra as LA

export mycontroller!, manual_jacobian, compute_quat_error, quat_to_vel

# Quaternion operations
# Quaternion conjugate
quat_conj(q::Vector{Float64}) = [q[1]; -q[2:4]]

# Quaternion multiplication
# q1 * q2 = [w1*w2 - dot(v1, v2); w1*v2 + w2*v1 + cross(v1, v2)]
# where q1 = [w1; v1] and q2 = [w2; v2]
function quat_mult(q1::Vector{Float64}, q2::Vector{Float64})
    w = q1[1]*q2[1] - LA.dot(q1[2:4], q2[2:4]) 
    xyz = q1[1]*q2[2:4] + q2[1]*q1[2:4] + LA.cross(q1[2:4], q2[2:4])
    return [w; xyz]
end

# Compute quaternion error
# q_err = target_quat ⊗ current_quat⁻¹
function compute_quat_error(target_quat, current_quat)
    # q_err = target_quat ⊗ current_quat⁻¹
    q_err = quat_mult(target_quat, quat_conj(current_quat))
    
    if q_err[1] < 0
        q_err .= -q_err
    end
    return q_err
end

# Convert quaternion error to angular velocity
# θ = 2 * atan2(||v||, w)
function quat_to_vel(q_err::Vector{Float64}, dt=1.0)
    # image_part
    imag_part = q_err[2:4]
    
    # When the quaternion is close to identity, we use Taylor expansion to avoid division by zero
    if abs(q_err[1]) >= 0.9999
        return 2.0 * imag_part / dt 
    else
        theta = 2 * atan(LA.norm(imag_part), q_err[1])
        axis = imag_part / LA.norm(imag_part)
        return theta * axis / dt
    end
end

# Manual Jacobian computation for a given site
# This function computes the Jacobian for a specific site in the model
function manual_jacobian(model::MJ.Model, data::MJ.Data, site_id::Int64)
    nv = 9
    jac_pos = Matrix{Float64}(undef, 3, nv)  # position Jacobian (3 x nv)
    jac_rot = Matrix{Float64}(undef, 3, nv)  # rotation Jacobian (3 x nv)
    
    # Get the position of the site in the world frame
    site_pos = data.xpos[site_id+1, :]

    # Iterate through each joint to compute the Jacobian
    for v in 1:nv
        # Find the joint ID corresponding to the current joint variable
        jnt_id = findfirst(==(v-1), model.jnt_dofadr)  # C-style 0-based
        
        if !isnothing(jnt_id)
            jnt_type = model.jnt_type[jnt_id]
            axis_world = data.xaxis[jnt_id[1], :]  # Get the joint axis in world coordinates
        
            if jnt_type == MJ.mjJNT_HINGE  # rotational joint
                joint_pos = data.xanchor[jnt_id[1]+1, :]
                jac_pos[:,v] = LA.cross(axis_world, site_pos - joint_pos)
                jac_rot[:,v] = axis_world
                
            elseif jnt_type == MJ.mjJNT_SLIDE  # translational joint
                jac_pos[:,v] = axis_world
                jac_rot[:,v] .= 0.0
            end
        end
    end
    J = Matrix{Float64}(undef, 6, nv)
    J = vcat(jac_pos, jac_rot)
    return J
end

end