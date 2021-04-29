import torch


def acceleration_constraints(velocity, acceleration, p):
    # accelerationConstraints - adjusts the acceleration based on acceleration
    # constraints
    #
    # Syntax:
    #    accelerationConstraints(velocity,acceleration,p)
    #
    # Inputs:
    #    acceleration - acceleration in driving direction
    #    velocity - velocity in driving direction
    #    p - longitudinal parameter structure
    #
    # Outputs:
    #    acceleration - acceleration in driving direction
    #
    # Example:
    #
    # Other m-files required: none
    # Subfunctions: none
    # MAT-files required: none
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      15-December-2017
    # Last update:  ---
    # Last revision:---

    # ------------- BEGIN CODE --------------

    # positive acceleration limit
    if velocity > p.v_switch:
        posLimit = p.a_max * p.v_switch / velocity
    else:
        posLimit = p.a_max

    # acceleration limit reached?
    if (velocity <= p.v_min and acceleration <= 0) or (
        velocity >= p.v_max and acceleration >= 0
    ):
        acceleration = 0
    elif acceleration <= -p.a_max:
        acceleration = -p.a_max
    elif acceleration >= posLimit:
        acceleration = posLimit

    return acceleration

    # ------------- END OF CODE --------------


def torch_acceleration_constraints(velocity, acceleration, p):
    # accelerationConstraints - adjusts the acceleration based on acceleration
    # constraints
    #
    # Syntax:
    #    accelerationConstraints(velocity,acceleration,p)
    #
    # Inputs:
    #    acceleration - acceleration in driving direction
    #    velocity - velocity in driving direction
    #    p - longitudinal parameter structure
    #
    # Outputs:
    #    acceleration - acceleration in driving direction
    #
    # Example:
    #
    # Other m-files required: none
    # Subfunctions: none
    # MAT-files required: none
    #
    # See also: ---

    # Author:       Matthias Althoff
    # Written:      15-December-2017
    # Last update:  ---
    # Last revision:---

    # ------------- BEGIN CODE --------------

    # positive acceleration limit
    posLimit = torch.zeros_like(acceleration) + p.a_max
    posLimit_saturated = velocity > p.v_switch
    posLimit[posLimit_saturated] = p.a_max * p.v_switch / velocity[posLimit_saturated]

    # acceleration limit reached?
    saturated_lower = torch.logical_and(velocity <= p.v_min, acceleration <= 0)
    saturated_upper = torch.logical_and(velocity >= p.v_max, acceleration >= 0)
    saturated = torch.logical_or(saturated_upper, saturated_lower)
    constrained_acceleration = torch.clamp(acceleration, min=-p.a_max)
    constrained_acceleration[acceleration >= posLimit] = posLimit[
        acceleration >= posLimit
    ]
    constrained_acceleration[saturated] = 0.0

    return constrained_acceleration

    # ------------- END OF CODE --------------
