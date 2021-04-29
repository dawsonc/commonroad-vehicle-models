import math

import torch

from vehiclemodels.utils.acceleration_constraints import (
    acceleration_constraints,
    torch_acceleration_constraints,
)
from vehiclemodels.utils.steering_constraints import (
    steering_constraints,
    torch_steering_constraints,
)


def vehicle_dynamics_ks_cog(x, u_init, p):
    """
    vehicle_dynamics_ks_cog - kinematic single-track vehicle dynamics
    reference point: center of mass

    Inputs:
        :param x: vehicle state vector
        :param u_init: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author: Gerald Würsching
    Written: 17-November-2020
    Last update: 17-November-2020
    Last revision: ---
    """
    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity at center of mass
    # x5 = yaw angle

    # wheelbase
    l_wb = p.a + p.b

    # consider steering constraints
    u = []
    u.append(
        steering_constraints(x[2], u_init[0], p.steering)
    )  # different name u_init/u due to side effects of u
    # consider acceleration constraints
    u.append(
        acceleration_constraints(x[3], u_init[1], p.longitudinal)
    )  # different name u_init/u due to side effects of u

    # slip angle (beta) from vehicle kinematics
    beta = math.atan(math.tan(x[2]) * p.b / l_wb)

    # system dynamics
    f = [
        x[3] * math.cos(beta + x[4]),
        x[3] * math.sin(beta + x[4]),
        u[0],
        u[1],
        x[3] * math.cos(beta) * math.tan(x[2]) / l_wb,
    ]

    return f


def torch_vehicle_dynamics_ks_cog(x, u_init, p):
    """
    vehicle_dynamics_ks_cog - kinematic single-track vehicle dynamics
    reference point: center of mass

    Inputs:
        :param x: vehicle state vector
        :param u_init: vehicle input vector
        :param p: vehicle parameter vector

    Outputs:
        :return f: right-hand side of differential equations

    Author: Gerald Würsching
    Written: 17-November-2020
    Last update: 17-November-2020
    Last revision: ---
    """
    # states
    # x1 = x-position in a global coordinate system
    # x2 = y-position in a global coordinate system
    # x3 = steering angle of front wheels
    # x4 = velocity at center of mass
    # x5 = yaw angle

    # wheelbase
    l_wb = p.a + p.b

    # consider steering constraints
    u = torch.zeros_like(u_init)
    u[:, 0] = torch_steering_constraints(x[:, 2], u_init[:, 0], p.steering)
    u[:, 1] = torch_acceleration_constraints(x[:, 3], u_init[:, 1], p.longitudinal)

    # slip angle (beta) from vehicle kinematics
    beta = torch.atan(torch.tan(x[:, 2]) * p.b / l_wb)

    # system dynamics
    f = torch.zeros_like(x)
    f[:, 0] = x[:, 3] * torch.cos(beta + x[:, 4])
    f[:, 1] = x[:, 3] * torch.sin(beta + x[:, 4])
    f[:, 2] = u[:, 0]
    f[:, 3] = u[:, 1]
    f[:, 4] = x[:, 3] * torch.cos(beta) * torch.tan(x[:, 2]) / l_wb

    return f
