using Pkg
Pkg.add(url="https://github.com/Octochen/UniMaaS")
using UniMaaS

const DJSP = UniMaaS.problem.deterministic_JSP
const GantChart = UniMaaS.util.plot_jsp_ganttchart
####################################################################################################
include("DJSSPs_Examples.jl")

jobs, machines, processing_times, machine_assignments = djssp_2J_2M()

makespan, start_times, model = DJSP.solve_deterministic_jsp(jobs, machines, processing_times, machine_assignments)

GantChart.jsp_ganttchart(jobs, machines, processing_times, machine_assignments, start_times)

