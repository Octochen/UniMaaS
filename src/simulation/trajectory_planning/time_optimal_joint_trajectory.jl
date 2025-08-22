module time_optimal_joint_trajectory

export time_optimal_joint_trajectory

# total time for trapezoidal speed profile
function trapezoidal_speed(
    q0::T,
    qf::T,
    v_max::T,
    a_max::T
    ) where {T<:Real}

    Δq = abs(qf - q0)
    t_acc = v_max / a_max
    Δq_acc = 0.5 * a_max * t_acc^2

    if 2 * Δq_acc > Δq
        t_total = 2 * sqrt(Δq / a_max)
        return t_total
    else
        t_total = 2 * t_acc + (Δq - 2 * Δq_acc) / v_max
        return t_total
    end
end

# position at time t for trapezoidal speed profile
function trapezoidal_speed(
    q0::T, 
    qf::T, 
    v_max::T, 
    a_max::T, 
    t::Real
    ) where {T<:Real}
    
    if t <= 0
        return q0
    end

    Δq = qf - q0
    sign_Δq = sign(Δq)
    Δq_abs = abs(Δq)
    # acceleration, uniform speed and deceleration time
    t_acc = v_max / a_max
    Δq_acc = 0.5 * a_max * t_acc^2
    if 2 * Δq_acc > Δq_abs
        # triangular
        t_acc = sqrt(Δq_abs / a_max)
        t_total = 2 * sqrt(Δq_abs / a_max)
        if t >= t_total
            return qf
        elseif t <= t_acc
            # acceleration
            return q0 + sign_Δq * 0.5 * a_max * t^2
        else
            # deceleration
            t_decel = t_total - t
            return qf - sign_Δq * 0.5 * a_max * t_decel^2
        end
    else
        # trapezoid
        t_total = 2 * t_acc + (Δq_abs - 2 * Δq_acc) / v_max
        if t >= t_total
            return qf
        elseif t <= t_acc
            # acceleration
            return q0 + sign_Δq * 0.5 * a_max * t^2
        elseif t <= (t_total - t_acc)
            # uniform speed
            t_cruise = t - t_acc
            return q0 + sign_Δq * (Δq_acc + v_max * t_cruise)
        else
            # deceleration
            t_decel = t_total - t
            return qf - sign_Δq * 0.5 * a_max * t_decel^2
        end
    end
end

function time_optimal_joint_trajectory(
    state0::RigidBodyDynamics.MechanismState, 
    t_start::Real,
    q_start::AbstractVector{<:Real},
    q_target::AbstractVector{<:Real},
    max_vel::AbstractVector{<:Real},
    max_acc::AbstractVector{<:Real},
    dt::Real
    )
    n_joints = length(state0.q)
    @assert length(q_start) == n_joints
    @assert length(q_target) == n_joints

    # trajectory time for each joint
    t_total = trapezoidal_speed.(q_start, q_target, max_vel, max_acc)

    # Take the time of the slowest joint as the total time
    t_final = maximum(t_total)
    time_points = t_start:dt:t_start+t_final

    joint_trajectory = Vector{Vector{Float64}}(undef, length(time_points))
    for (i, t) in enumerate(time_points)
        singlejoint_trajectory = Vector{Float64}(undef, n_joints)
        for j in 1:n_joints
            singlejoint_trajectory[j] = trapezoidal_speed(
                q_start[j], q_target[j], max_vel[j], max_acc[j], t-t_start
            )
        end
        joint_trajectory[i] = singlejoint_trajectory
    end

    return time_points, joint_trajectory

end
# function trajectory_planning(target_end_effector_position)
# end

end