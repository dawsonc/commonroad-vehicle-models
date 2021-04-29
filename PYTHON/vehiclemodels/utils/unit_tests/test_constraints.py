import unittest
import random

import torch

from vehiclemodels.parameters_vehicle2 import parameters_vehicle2
from vehiclemodels.utils.acceleration_constraints import (
    acceleration_constraints,
    torch_acceleration_constraints,
)
from vehiclemodels.utils.steering_constraints import (
    steering_constraints,
    torch_steering_constraints,
)


class TestConstraints(unittest.TestCase):
    def setUp(self):
        # Set a random seed for repeatability
        random.seed(0)
        torch.manual_seed(0)

        self.p = parameters_vehicle2()

    def test_acceleration_constraints(self):
        N_test = 100

        u = torch.Tensor(N_test, 2).uniform_(-1.0, 1.0)
        u[:, 0] *= 2 * self.p.steering.v_max
        u[:, 1] *= 2 * self.p.longitudinal.a_max

        velocity = torch.Tensor(
            N_test,
        ).uniform_(-1.0, 1.0)
        velocity *= 2 * self.p.longitudinal.v_max

        constrained_acceleration_torch = torch_acceleration_constraints(
            velocity, u[:, 1], self.p.longitudinal
        )

        for i in range(N_test):
            constrained_acceleration = acceleration_constraints(
                velocity[i].item(), u[i, 1].item(), self.p.longitudinal
            )

            self.assertAlmostEqual(
                constrained_acceleration,
                constrained_acceleration_torch[i].item(),
                places=5,
            )

    def test_steering_constraints(self):
        N_test = 100

        u = torch.Tensor(N_test, 2).uniform_(-1.0, 1.0)
        u[:, 0] *= 2 * self.p.steering.v_max
        u[:, 1] *= 2 * self.p.longitudinal.a_max

        steering_angle = torch.Tensor(
            N_test,
        ).uniform_(-1.0, 1.0)
        steering_angle *= 2 * self.p.steering.max

        constrained_steering_torch = torch_steering_constraints(
            steering_angle, u[:, 0], self.p.steering
        )

        for i in range(N_test):
            constrained_steering = steering_constraints(
                steering_angle[i].item(), u[i, 0].item(), self.p.steering
            )

            self.assertAlmostEqual(
                constrained_steering,
                constrained_steering_torch[i].item(),
                places=5,
            )
