export plot_jsp_ganttchart

module plot_jsp_ganttchart
using Plots

function jsp_ganttchart(jobs, machines, processing_times, machine_assignments, start_times; 
                         title="Job Shop Scheduling Gantt Chart", size=(800, 400))
    """
    Plots a professional Gantt chart for JSP results using bar! function.
    
    Parameters:
    - jobs: Vector of job IDs
    - machines: Vector of machine IDs
    - processing_times: Dict of (job, operation) => processing time
    - machine_assignments: Dict of (job, operation) => machine
    - start_times: Dict of (job, operation) => start time
    - title: Chart title (optional)
    - size: Figure size (optional)
    
    Returns:
    - Gantt chart plot object
    """
    
    # Prepare data
    sorted_machines = sort(machines)
    job_colors = Dict(j => get_color_for_job(j) for j in jobs)
    
    # Create plot
    plt = Plots.plot(size=size, title=title, xlabel="Time", ylabel="Machine", 
               legend=:topleft, grid=true, titlefontsize=12)
    
    # Track which jobs we've already added to legend
    jobs_in_legend = Set()
    
    # Plot each operation as a horizontal bar
    for op in keys(processing_times)
        job, op_num = op
        machine = machine_assignments[op]
        start = start_times[op]
        duration = processing_times[op]
        machine_idx = findfirst(==(machine), sorted_machines)
        
        # Determine if we should add this job to legend
        show_in_legend = !(job in jobs_in_legend)
        if show_in_legend
            push!(jobs_in_legend, job)
        end
        
        # Plot the operation as a horizontal bar
        bar!(plt, [machine_idx], [start], fillto = start + duration,
            color = job_colors[job],
            bar_width = 0.8,
            orientation="h",
            label=show_in_legend ? "Job $job" : "",
            alpha=0.7,
            linewidth=0.5,
            linecolor=:black)
        # bar!(plt, [machine_idx], [duration], 
        #     bar_position=start,
        #     color=job_colors[job],
        #     label=show_in_legend ? "Job $job" : "",
        #     alpha=0.7,
        #     linewidth=0.5,
        #     linecolor=:black)
        
        # Add operation label in the center of the bar
        annotate!(plt, 
            start + duration/2, machine_idx, 
            text("J$(job)O$(op_num)", 2, :white, :center))
    end
    
    # Customize y-axis to show machine names
    Plots.yticks!(plt, 
        (1:length(sorted_machines), 
        ["Machine $m" for m in sorted_machines]))
    
    # Add makespan line and annotation
    completion_times = [start_times[op] + processing_times[op] for op in keys(processing_times)]
    makespan = maximum(completion_times)
    vline!(plt, [makespan], 
          linestyle=:dash, 
          linecolor=:red, 
          linewidth=2, 
          label="Makespan = $(round(makespan, digits=2))")
    
    return plt
end

function get_color_for_job(job_id)
    """
    Helper function to assign distinct colors to jobs.
    """
    colors = Vector(undef, 100)
    for i in 1:1:100
        colors[i] = RGB(rand(), rand(), rand())
    end
    return colors[mod1(job_id, length(colors))]
end

end