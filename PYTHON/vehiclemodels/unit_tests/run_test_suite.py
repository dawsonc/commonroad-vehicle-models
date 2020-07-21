from vehiclemodels.unit_tests.test_derivatives import test_derivatives
from vehiclemodels.unit_tests.test_zeroInitialVelocity import test_zeroInitialVelocity


if __name__ == "__main__":
    res = test_derivatives()
    res = test_zeroInitialVelocity()

