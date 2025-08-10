# Example usage
function djssp_2J_2M()
    jobs = [1, 2]
    machines = [1, 2]
    
    # Processing times: (job, operation) => time
    processing_times = Dict(
        (1, 1) => 3, (1, 2) => 2,  # Job 1 operations
        (2, 1) => 2, (2, 2) => 1   # Job 2 operations
    )
    
    # Machine assignments: (job, operation) => machine
    machine_assignments = Dict(
        (1, 1) => 1, (1, 2) => 2,  # Job 1 operations
        (2, 1) => 2, (2, 2) => 1   # Job 2 operations
    )

    return jobs, machines, processing_times, machine_assignments
end

function djssp_10J_10M()
    n=8
    m=8
    u=8
    jobs = 1:1:n
    machines = 1:1:m
    
    # Processing times: (job, operation) => time
    processing_times = Dict()
    for i in jobs
        for j in 1:1:u
            processing_times[(i, j)]=1+round(3*rand())
        end
    end
    
    # Machine assignments: (job, operation) => machine
    machine_assignments = Dict()
    for i in jobs
        for j in 1:1:u
            machine_assignments[(i, j)]=1+round((m-1)*rand())
        end
    end

    return jobs, machines, processing_times, machine_assignments
end