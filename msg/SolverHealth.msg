# What the MPC's backend did on one solve of the problem in wiki/mpc.md 1.
#
# wiki/mpc.md 5.3 lists four requirements on any backend integration and says
# outright why they are requirements: each of the four is a defect observed in
# the deployed iLQR integration -- solve failures reported as success,
# regularization latching high after a deadline miss, no exposed residuals, and
# no defined behaviour when the budget is exceeded. This message is where the
# first, the third and the fourth become visible outside the node; the second
# has nothing to publish, because the whole of it is that nothing is carried.
#
# `fault` is a crane_msgs/SupervisorStatus FAULT_* constant and is deliberately
# not renumbered, exactly as VelocityControllerHealth's is: the supervisor
# merges the code rather than translating it, and an operator panel renders one
# numbering. The only code this producer raises is FAULT_SOLVER, which
# wiki/control_architecture.md 5 row 3 gives to non-convergence and to a
# deadline miss alike. **Nothing consumes it yet** -- the supervisor merging it
# onto /crane/supervisor/status is issue 054.

# Requirement 1. A solve that hit its budget without converging has to be
# distinguishable from one that converged, so the verdict is three-valued and
# never a bool, and zero is the *unknown* state for the reason SwaySettled's is:
# a message nobody filled must not read as a healthy solve.
uint8 SOLVE_UNKNOWN=0
uint8 SOLVE_CONVERGED=1
uint8 SOLVE_BUDGET_EXCEEDED=2
uint8 SOLVE_FAILED=3

# Rows of `constraint_violation` -- the four constraints wiki/mpc.md 3.2
# softens, in that page's own numbering. Constraints 1, 2 and 5 have no row
# because they are hard and carry no slack variable at all: that is what hard
# means, and it is a property of the problem's shape rather than of its prices.
uint8 CONSTRAINT_SWAY=0            # constraint 3, |q_u - q_eq| <= q_u^+
uint8 CONSTRAINT_SWAY_RATE=1       # constraint 4, |dq_u| <= dq_u^+
uint8 CONSTRAINT_CYLINDER_FORCE=2  # constraint 6, |F_cyl| -- worst of six axes
uint8 CONSTRAINT_PUMP_FLOW=3       # constraint 7, the one shared pump

# Rows of `cost_term` -- the terms of wiki/mpc.md 2, in the order that page
# writes them, then the terminal cost and then 3.2's L1 slack price. Splitting
# them is the point of requirement 4: a total says the plan is expensive, and
# only the split says whether the expense is tracking, sway or effort.
uint8 COST_ACTUATED_POSITION=0
uint8 COST_ACTUATED_VELOCITY=1
uint8 COST_SWAY=2
uint8 COST_SWAY_RATE=3
uint8 COST_EFFORT=4
uint8 COST_INPUT=5
uint8 COST_TERMINAL=6
uint8 COST_SLACK=7

std_msgs/Header header
uint8 outcome
uint8 fault

# Requirement 1 again, and the half a three-valued verdict cannot carry: acados'
# **own** return code, not a boolean this integration invented. `qp_status` is
# separate and is not redundant -- acados reports a QP that merely hit HPIPM's
# iteration limit as ACADOS_SUCCESS, so a solve that stopped short is visible
# here and nowhere else.
int32 status
string status_word
int32 iterations       # under SQP_RTI this is one, every cycle (mpc 5)
int32 qp_status
int32 qp_iterations

# Requirement 3. The budget is wiki/mpc.md 4's `< 30 ms` and it is enforced
# rather than measured: on an overrun `applied_previous_solution` is true and
# what went out on /crane/mpc/horizon is the previous solution shifted by one
# step, with its last stage duplicated (mpc 6). It is also true on any other
# non-convergence, because mpc 6 gives failure the same fallback.
float64 solve_time     # s, acados' measured time_tot
float64 solve_budget   # s
bool applied_previous_solution

# The six actuated joints of ROS 2 Interfaces 3.2, carried rather than assumed
# for VelocityControllerHealth's reason: the sixth valve channel drives a
# different joint per tool, and a panel that says "gripper" is worth more than
# one that says a count. The cost and violation rows below are horizon-wide, so
# these name the axes `constraint_violation[CONSTRAINT_CYLINDER_FORCE]` was
# taken over rather than being parallel to it.
string[6] joint_names

# Requirement 4, first half: which soft constraint the solution spent slack on,
# and how much. Each row is in **its own normalised unit** -- 1.0 is one whole
# allowance past the limit -- so the four are directly comparable, and each is
# the worst over the whole horizon. `used_slack` is the predicate; a row is
# non-zero only where slack was actually spent, so "on which constraint" is read
# off the vector rather than off a second index field that would have to mean
# something on a solve that spent nothing.
bool used_slack
float64[4] constraint_violation
float64 slack_penalty  # the L1 price paid, in the cost's own units

# Requirement 4, second half: the cost of wiki/mpc.md 2 term by term, summed
# over the horizon. 5.3's reason for wanting them is that constraint violations
# and cost terms "are the only way to tell a tuning problem from a modelling
# one", and the PRD's risk table says the symptom of the latter looks exactly
# like the former.
float64[8] cost_term

# The same clause the log line ends in, so a panel reading this cannot disagree
# with one reading the node's output.
string message
