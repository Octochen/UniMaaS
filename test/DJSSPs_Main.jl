using UniMaaS

####################################################################################################

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

makespan, start_times, model = solve_deterministic_jsp(jobs, machines, processing_times, machine_assignments)

plot_jsp_ganttchart(jobs, machines, processing_times, machine_assignments, start_times)

