# include("FJSSPs_Pkgs.jl")
include("FJSSPs_Functions.jl")
include("FJSSPs_Examples.jl")
include("FJSSPs_GanttChart.jl")

using JuMP, Cbc
###################################################################################################

jobs, machines, operations, processing_times, machine_assignments, start_times, makespan, model = fjssp_10J_10M()

# Plot the disjunctive graph
FJSSPs_Gantt(jobs, machines, processing_times, machine_assignments, start_times)