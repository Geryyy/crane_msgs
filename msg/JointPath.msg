# The planned geometric path in joint space: q_a(sigma), sigma in [0, 1].
#
# Geometry, with no schedule in it. A path-following cost is written against
# the curve itself and chooses where on it to be, so what it needs is the curve
# and the pace the plan was written at (`duration`, and `s_profile` for how that
# pace is spent along the way) -- not a schedule. `/crane/reference`
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
# The plan's own time law s(t): the path parameter sampled uniformly in time
# over `duration`, first sample 0 and last 1, non-decreasing. Still a pace and
# not a schedule -- a consumer anchors it at wherever on the path it currently
# is -- but the pace the plan actually meant, point by point, where `duration`
# alone gives only its mean. The plan rests at both ends and the middle peaks
# well above that mean (1.76x measured, issue 250), so a consumer pricing
# against the mean runs the fast part of the move far too slow. Empty is
# allowed and means exactly that mean: a producer with no law in hand omits it.
float64[] s_profile
