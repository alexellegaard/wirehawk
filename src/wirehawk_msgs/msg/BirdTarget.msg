# Deterrence target handed from the decision layer (mission planner) to the
# motion layer (bird sweeper). The decision layer collapses the whole bird
# state (present / fled / landed / flying) down to this: valid=true means
# "engage this point at `speed`", valid=false means "no target -> return home".
#
# A moving bird is just a `position` that changes; no velocity field yet.
geometry_msgs/Point position   # target position in the robot frame (z ignored in 2D mode)
bool valid                     # true = engage this point at `speed`; false = no target
float64 speed                  # desired approach speed [m/s] (5.0 engage, lower for return)
