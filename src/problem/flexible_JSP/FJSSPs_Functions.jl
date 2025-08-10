using JuMP, Cbc

function solve_flexible_jssp(jobs, machines, operations, processing_times, machine_options; H=1_000_000)
    """
    Solves a Flexible Job-Shop Scheduling Problem (FJSSP) using Mixed Integer Programming.
    
    Parameters:
    - jobs: Vector of job IDs (e.g., [1, 2, 3])
    - machines: Vector of machine IDs (e.g., [1, 2, 3])
    - operations: Vector of (job, operation) pairs
    - processing_times: Dict((job, operation, machine) => time)
    - machine_options: Dict((job, operation) => Vector of compatible machines
    - H: A sufficiently large constant for big-M constraints
    
    Returns:
    - A tuple containing:
        * The optimal makespan (Cmax)
        * A dictionary of start times Dict((job, operation) => start_time)
        * A dictionary of machine assignments Dict((job, operation) => machine)
        * The optimization model for further inspection
    """

    # Create model - using Cbc for MIP
    model = Model(Cbc.Optimizer)
    set_optimizer_attribute(model, "seconds", 100.0)

    # Variables
    @variable(model, Cmax >= 0)  # Makespan
    @variable(model, t[operations] >= 0)  # Start time for each operation
    
    # Assignment variables: α[(i,μ,j)] = 1 if operation (i,μ) is assigned to machine j
    α = Dict()
    for op in operations
        for j in machine_options[op]
            α[(op, j)] = @variable(model, binary=true, base_name="α_$(op)_$j")
        end
    end
    
    # Precedence variables: β[(i,μ1),(i′,μ2)] = 1 if (i,μ1) is before (i′,μ2) on same machine
    # Initialize for all pairs that could potentially share a machine
    β_dict = Dict()
    for (op1, op2) in Iterators.product(operations, operations)
        if op1 == op2
            continue  # Skip same operation
        end
        
        # Check if operations have any machines in common
        common_machines = intersect(machine_options[op1], machine_options[op2])
        if !isempty(common_machines)
            β_dict[(op1, op2)] = @variable(model, binary=true, base_name="β_$(op1)_$(op2)")
        end
    end
    
    # Objective: Minimize makespan
    @objective(model, Min, Cmax)
    
    # Constraint: Each operation must be assigned to exactly one compatible machine
    for op in operations
        @constraint(model, sum(α[(op, j)] for j in machine_options[op]) == 1)
    end
    
    # Constraint: Makespan is at least the completion time of the last operation of each job
    for job in jobs
        job_ops = filter(op -> op[1] == job, operations)
        last_op = argmax(op -> op[2], job_ops)
        @constraint(model, Cmax >= t[last_op] + 
            sum(processing_times[(last_op[1], last_op[2], j)] * α[(last_op, j)] for j in machine_options[last_op]))
    end
    
    # Precedence constraints within each job
    for job in jobs
        job_ops = filter(op -> op[1] == job, operations)
        sorted_ops = sort(job_ops, by=op -> op[2])

        for i in 1:length(sorted_ops)-1
            op1 = sorted_ops[i]
            op2 = sorted_ops[i+1]
            @constraint(model, t[op2] >= t[op1] + 
                sum(processing_times[(op1[1], op1[2], j)] * α[(op1, j)] for j in machine_options[op1]))
        end
    end
    
    # Disjunctive constraints for operations that could share machines
    for ((op1, op2), β) in β_dict
        common_machines = intersect(machine_options[op1], machine_options[op2])
        
        for j in common_machines
            # If both operations are assigned to machine j, enforce sequencing
            @constraint(model, t[op2] >= t[op1] + processing_times[(op1[1], op1[2], j)] - 
                (2 - α[(op1, j)] - α[(op2, j)] + β) * H)
            
            @constraint(model, t[op1] >= t[op2] + processing_times[(op2[1], op2[2], j)] - 
                (3 - α[(op1, j)] - α[(op2, j)] - β) * H)
        end
    end

    # Solve the model
    optimize!(model)
    
    # Check solution status
    if termination_status(model) != MOI.OPTIMAL
        @warn "Solver did not find optimal solution. Status: $(termination_status(model))"
    end
    
    # Collect results
    start_times = Dict(op => value(t[op]) for op in operations)
    
    machine_assignments = Dict()
    for op in operations
        for j in machine_options[op]
            if value(α[(op, j)]) > 0.5  # Threshold for binary variable
                machine_assignments[op] = j
                break
            end
        end
    end
    
    makespan = value(Cmax)

    return makespan, start_times, machine_assignments, model
end