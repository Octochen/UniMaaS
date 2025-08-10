export deterministic_JSP

module deterministic_JSP
import JuMP, Cbc


function solve_deterministic_jsp(jobs, machines, processing_times, machine_assignments; H=1_000_000)
    """
    Solves a deterministic Job-Shop Scheduling Problem (JSP) using Mixed Integer Programming.
    
    Parameters:
    - jobs J_i: Vector of job IDs (e.g., [1, 2, 3])
    - machines M_j: Vector of machine IDs (e.g., [1, 2, 3])
    - processing_times p_ij: Dictionary of processing times Dict((job, operation) => time)
    - operation_IDs O_ij: vector of pairs (job, operation)
    - machine_assignments: Dictionary of machine assignments Dict((job, operation) => machine)
    - H: A sufficiently large constant for big-M constraints
    
    Returns:
    - A tuple containing:
        * The optimal makespan (Cmax)
        * A dictionary of start times Dict((job, operation) => start_time)
        * The optimization model for further inspection
    """
    
    # Create a list of all operations (job, operation) pairs; 
    operations = collect(keys(processing_times))
    
    # Create model
    model = Model(Cbc.Optimizer)
    
    # Variables
    JuMP.@variable(model, Cmax >= 0)  # Makespan
    JuMP.@variable(model, t[operations] >= 0)  # Start time for each operation

    # Precedence variables: β[(i,μ1),(i′,μ2)] = 1 if (i,μ1) is before (i′,μ2) on same machine
    # Initialize for all pairs that share the same machine
    β_dict = Dict()
    
    # Find all pairs of operations that share the same machine
    for (op1, op2) in Iterators.product(operations, operations)
        if op1 == op2
            continue  # Skip same operation
        end
        
        # Check if operations are on the same machine
        if machine_assignments[op1] == machine_assignments[op2]
            if op2[2] > op1[2]
                β_dict[(op1, op2)] = true
            end
        end
    end
    
    # Objective: Minimize makespan
    JuMP.@objective(model, Min, Cmax)
    
    # Constraint: Makespan is at least the completion time of the last operation of each job
    for job in jobs
        # Find all operations for this job
        # op -> op[1] == job is an anonymous function
        job_ops = filter(op -> op[1] == job, operations)
        # Find the last operation (assuming operations are ordered 1,2,3,...)
        last_op = argmax(op -> op[2], job_ops)
        JuMP.@constraint(model, Cmax >= t[last_op] + processing_times[last_op])
    end
    
    # Precedence constraints within each job
    for job in jobs
        # Get all operations for this job in order
        job_ops = filter(op -> op[1] == job, operations)
        # sort(collection; by=key_function, rev=false, alg=default_algorithm)
        sorted_ops = sort(job_ops, by=op -> op[2])

        # Add precedence constraints between consecutive operations
        for i in 1:length(sorted_ops)-1
            op1 = sorted_ops[i]
            op2 = sorted_ops[i+1]
            JuMP.@constraint(model, t[op2] >= t[op1] + processing_times[op1])
        end
    end
    
    # Disjunctive constraints for operations sharing the same machine
    for ((op1, op2), β) in β_dict
        m = machine_assignments[op1]  # Machine shared by op1 and op2
        
        # op1 before op2: t[op2] >= t[op1] + p[op1] - (1 - β) * H
        JuMP.@constraint(model, t[op2] >= t[op1] + processing_times[op1] - (1 - β) * H)
        
        # op2 before op1: t[op1] >= t[op2] + p[op2] - β * H
        JuMP.@constraint(model, t[op1] >= t[op2] + processing_times[op2] - β * H)
    end
    
    # Solve the model
    optimize!(model)
    
    # Check solution status
    if termination_status(model) != MOI.OPTIMAL
        error("Solver did not find an optimal solution. Status: $(termination_status(model))")
    end
    
    # Collect results
    # Dict(key_expr => value_expr for element in collection): quickly create a dictionary.
    start_times = Dict(op => value(t[op]) for op in operations)
    makespan = value(Cmax)
    
    return makespan, start_times, model
end


end