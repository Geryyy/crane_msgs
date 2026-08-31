# Whether the load has stopped swinging, as a field a task layer can branch on.
#
# wiki/control_architecture.md 5 row 7 gives the supervisor one narrow duty about
# the sway: refuse to start a motion that depends on sway being settled, and
# report. This is the report. Nothing here damps, stops or refuses anything --
# active damping is a later slice, and the refusal is the caller's.
#
# Three values and not two, and the third one is the point. An estimate that is
# absent, stale or marked unusable makes "settled" unknowable, and unknowable
# must not read as settled: a grip action gated on a boolean would descend onto a
# swinging block the moment the bracketing IMU stopped answering. So a degraded
# estimate is SETTLED_UNKNOWN here and FAULT_STATE_HEALTH on
# crane_msgs/SupervisorStatus -- a sensor that stopped saying anything is a
# different fact from a load that is swinging, and an operator does different
# things about them.
uint8 SETTLED_UNKNOWN=0
uint8 SETTLED_NO=1
uint8 SETTLED_YES=2
std_msgs/Header header
uint8 settled
# dq_u [tip, tilt], rad/s -- the rates the verdict above was decided from, in
# that order. Carried because it is what the decision was made from: a consumer that disagrees with the verdict can see why.
# The rate and not the angle, deliberately -- the published angle is read out on
# the nominal hinge axes and carries an uncalibrated constant offset, while the
# rate is composed from the two gyro readings through the known chain and does
# not. NaN on a cycle where no rate could be read at all.
float64[2] velocity
# Why, in words an operator can act on. It is the same sentence the supervisor's
# status stream ends in, so the field and the sentence cannot disagree.
string message
