# The planned geometric path in joint space: q_a(sigma), sigma in [0, 1].
#
# Geometry, with no timing law in it. A path-following cost is written against
# the curve itself and chooses where on it to be, so what it needs is the curve
# and the pace the plan was written at -- not a schedule. `/crane/reference`
# carries the same plan resampled in time, which is what a trajectory follower
# needs instead. The two are one plan in two forms, published together and
# stamped alike: `header.stamp` is what pairs them.
std_msgs/Header header
# One joint name per column of `q_path`, in that order. The planned coordinates
# only. A consumer lifts its own columns by name; the order here is not a
# contract, the names are.
string[] joint_names
# q_a read at sigma = i / (rows - 1), row-major, `joint_names` columns wide --
# so `rows` is `q_path`'s length over `joint_names`'. Uniform in sigma and
# nothing else: a consumer refits in whatever basis it wants without knowing the
# planner's, and no timing enters the parameterisation. Both ends are samples,
# so the goal is a point on the curve rather than an approximation of one.
float64[] q_path
# What the plan means to spend the whole path in, so 1/duration is path
# parameter per second. A pace a progress cost is priced against, not a
# deadline: a consumer free to choose where on the path to be is free to take
# longer, and does.
builtin_interfaces/Duration duration
