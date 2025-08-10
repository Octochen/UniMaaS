# Example usage
function fjssp_2J_2M()
    jobs = [1, 2]
    machines = [1, 2]
    operations = [(1, 2), (1, 1), (2, 2), (2, 1)]

    # Processing times: (job, operation) => time
    processing_times = Dict(
        (1, 1, 1) => 3, (1, 1, 2) => 2,
        (1, 2, 1) => 2, (1, 2, 2) => 1, 
        (2, 1, 1) => 2, (2, 1, 2) => 2,
        (2, 2, 1) => 1, (2, 2, 2) => 2 
    )

    machine_options = Dict(
        (1, 1) => [1], (1, 2) => [1 2],  # Job 1 operations
        (2, 1) => [1 2], (2, 2) => [1 2]   # Job 2 operations
    )
    
    makespan, start_times, machine_assignments, model = solve_flexible_jssp(jobs, machines, operations, processing_times, machine_options; H=1_000_000)

    return jobs, machines, operations, processing_times, machine_assignments, start_times, makespan, model
end

function fjssp_10J_10M()
    n=6
    m=6
    u=6
    jobs = 1:1:n
    machines = 1:1:m

    # Processing times: (job, operation) => time
    processing_times = Dict()
    for i in jobs
        for j in 1:1:u
            for k in machines
                processing_times[(i, j, k)]=1+round(3*rand())
            end
        end
    end
    
    # machine_options: (job, operation) => machine
    machine_options = Dict()
    for i in jobs
        for j in 1:1:u
            machine_options[(i, j)]=1:1:m
        end
    end
    operations = collect(keys(machine_options))
    
    makespan, start_times, machine_assignments, model = solve_flexible_jssp(jobs, machines, operations, processing_times, machine_options; H=1_000_000)
    
    return jobs, machines, operations, processing_times, machine_assignments, start_times, makespan, model
end