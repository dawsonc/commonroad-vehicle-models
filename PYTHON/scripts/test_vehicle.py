from scipy.integrate import odeint
import numpy
import matplotlib.pyplot as plt
from matplotlib.pyplot import title, legend

from vehiclemodels.parameters_vehicle1 import parameters_vehicle1
from vehiclemodels.init_ks import init_ks
from vehiclemodels.init_st import init_st
from vehiclemodels.init_mb import init_mb
from vehiclemodels.vehicle_dynamics_ks import vehicle_dynamics_ks
from vehiclemodels.vehicle_dynamics_st import vehicle_dynamics_st
from vehiclemodels.vehicle_dynamics_mb import vehicle_dynamics_mb


def func_KS(x, t, u, p):
    f = vehicle_dynamics_ks(x, u, p)
    return f


def func_ST(x, t, u, p):
    f = vehicle_dynamics_st(x, u, p)
    return f


def func_MB(x, t, u, p):
    f = vehicle_dynamics_mb(x, u, p)
    return f


# load parameters
p = parameters_vehicle1()
g = 9.81  # [m/s^2]

# set options --------------------------------------------------------------
tStart = 0  # start time
tFinal = 1  # start time

delta0 = 0
vel0 = 15
Psi0 = 0
dotPsi0 = 0
beta0 = 0
sy0 = 0
initialState = [0, sy0, delta0, vel0, Psi0, dotPsi0, beta0]  # initial state for simulation
x0_KS = init_ks(initialState)  # initial state for kinematic single-track model
x0_ST = init_st(initialState)  # initial state for single-track model
x0_MB = init_mb(initialState, p)  # initial state for multi-body model
# --------------------------------------------------------------------------
#

t = numpy.arange(0, tFinal, 0.01)
u = [0, 0]
x = odeint(func_KS, x0_KS, t, args=(u, p))

print(x)

t = numpy.arange(0, tFinal, 0.01)
u = [0, 5]
x = odeint(func_KS, x0_KS, t, args=(u, p))

print(x)

# set input: rolling car (velocity should stay constant)
t = numpy.arange(0, tFinal, 0.01)
u = [0, 0]
print(x0_MB)
# simulate car
x = odeint(func_MB, x0_MB, t, args=(u, p))
# plot velocity
plt.plot(t, [tmp[3] for tmp in x])
plt.show()

# set input: braking car (wheel spin and velocity should decrease  similar wheel spin)
v_delta = 0.0
acc = -0.7 * g
u = [v_delta, acc]
# simulate car
x_brake = odeint(func_MB, x0_MB, t, args=(u, p))
# position
plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
plt.show()
# velocity
plt.plot(t, [tmp[3] for tmp in x_brake])
plt.show()
# wheel spin
title('wheel spin braking')
plt.plot(t, [tmp[23] for tmp in x_brake])
plt.plot(t, [tmp[24] for tmp in x_brake])
plt.plot(t, [tmp[25] for tmp in x_brake])
plt.plot(t, [tmp[26] for tmp in x_brake])
plt.show()
# pitch
plt.plot(t, [tmp[8] for tmp in x_brake])
plt.show()

# set input: accelerating car (wheel spin and velocity should increase  more wheel spin at rear)
v_delta = 0.0
acc = 0.63 * g
u = [v_delta, acc]
# simulate car
x_acc = odeint(func_MB, x0_MB, t, args=(u, p))
# position
plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
title('positions acceleration')
plt.show()
# velocity
plt.plot(t, [tmp[3] for tmp in x_acc])
plt.show()
# wheel spin
plt.plot(t, [tmp[23] for tmp in x_acc])
plt.plot(t, [tmp[24] for tmp in x_acc])
plt.plot(t, [tmp[25] for tmp in x_acc])
plt.plot(t, [tmp[26] for tmp in x_acc])
title('wheel spins acceleration')
legend(['left front', 'right front', 'left rear', 'right rear'])
plt.show()
# pitch
plt.plot(t, [tmp[8] for tmp in x_acc])
plt.show()
# orientation
plt.plot(t, [tmp[4] for tmp in x_acc])
title('orientation')
plt.show()

# steering to left
v_delta = 0.15
u = [v_delta, 0]

# simulate full car
x_left = odeint(func_MB, x0_MB, t, args=(u, p))

# simulate single-track model
x_left_st = odeint(func_ST, x0_ST, t, args=(u, p))

# simulate kinematic single-track model
x_left_ks = odeint(func_KS, x0_KS, t, args=(u, p))

# position
title('positions turning')
plt.plot([tmp[0] for tmp in x_left], [tmp[1] for tmp in x_left])
plt.plot([tmp[0] for tmp in x_left_st], [tmp[1] for tmp in x_left_st])
plt.plot([tmp[0] for tmp in x_left_ks], [tmp[1] for tmp in x_left_ks])
legend(['MB','ST','KS'])
plt.axis('equal')
plt.autoscale()
plt.show()
# orientation
title('orientations turning')
plt.plot(t, [tmp[4] for tmp in x_left])
plt.plot(t, [tmp[4] for tmp in x_left_st])
plt.plot(t, [tmp[4] for tmp in x_left_ks])
legend(['MB','ST','KS'])
plt.show()
# steering
title('steering turning')
plt.plot(t, [tmp[2] for tmp in x_left])
plt.plot(t, [tmp[2] for tmp in x_left_st])
plt.plot(t, [tmp[2] for tmp in x_left_ks])
legend(['MB','ST','KS'])
plt.show()
# yaw rate
title('yaw rate turning')
plt.plot(t, [tmp[5] for tmp in x_left])
plt.plot(t, [tmp[5] for tmp in x_left_st])
legend(['MB','ST'])
plt.show()
# slip angle
title('slip angle turning')
plt.plot(t, [tmp[10] / tmp[3] for tmp in x_left])
plt.plot(t, [tmp[6] for tmp in x_left_st])
plt.show()


# compare position for braking/normal
# position
title('position comparison MB')
plt.plot([tmp[0] for tmp in x_left], [tmp[1] for tmp in x_left])
plt.plot([tmp[0] for tmp in x_brake], [tmp[1] for tmp in x_brake])
plt.plot([tmp[0] for tmp in x_acc], [tmp[1] for tmp in x_acc])
legend(['turning','braking', 'accelerating'])
plt.show()
# compare slip angles
title('slip angle comparison MB')
plt.plot(t, [tmp[10] / tmp[3] for tmp in x_left])
plt.plot(t, [tmp[10] / tmp[3] for tmp in x_brake])
plt.plot(t, [tmp[10] / tmp[3] for tmp in x_acc])
legend(['turning','braking', 'accelerating'])
plt.show()
# orientation
title('orientation comparison MB')
plt.plot(t, [tmp[4] for tmp in x_left])
plt.plot(t, [tmp[4] for tmp in x_brake])
plt.plot(t, [tmp[4] for tmp in x_acc])
legend(['turning','braking', 'accelerating'])
plt.show()
# pitch
title('pitch comparison MB')
plt.plot(t, [tmp[8] for tmp in x_left])
plt.plot(t, [tmp[8] for tmp in x_brake])
plt.plot(t, [tmp[8] for tmp in x_acc])
legend(['turning','braking', 'accelerating'])
plt.show()

# ------------- END OF CODE --------------
