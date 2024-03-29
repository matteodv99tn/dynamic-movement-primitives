import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pinocchio as pin
import os.path
from scipy.spatial.transform import Rotation as Rot

prj_dir = os.path.dirname(os.path.realpath(__file__))
outfile = os.path.join(prj_dir, 'test', 'data', 'synthetic.csv')


def min_jerk_traj(q0: np.ndarray, q1: np.ndarray, t: np.ndarray):
    # https://arxiv.org/pdf/2102.07459.pdf
    T = t[-1] - t[0]
    tau = (t - t[0]) / T
    pos = np.zeros((len(q0), len(t)))
    vel = np.zeros((len(q0), len(t)))
    acc = np.zeros((len(q0), len(t)))
    for i in range(len(q0)):
        pos[i, :] = q0[i] + (q1[i] - q0[i]) * (10 * tau**3 - 15 * tau**4 +
                                               6 * tau**5)
        vel[i, :] = (q1[i] - q0[i]) * (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
        acc[i, :] = (q1[i] - q0[i]) * (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2
    return pos, vel, acc


Thalf = 2.0
dt = 0.002
N = int(Thalf / dt)
tfirst = np.arange(0, dt * N, dt)
tsecond = np.arange(dt * N, 2 * dt * N, dt)

q0 = np.array([0.1, 0.3, 0.5, -0.5, -1.0, 0])
q1 = np.array([0.5, 0.8, -0.1, 0.5, -0.4, 1.5])

Q1, Qd1, Qdd1 = min_jerk_traj(q0, q1, tfirst)
Q2, Qd2, Qdd2 = min_jerk_traj(q1, q0, tsecond)

Nreps = 100
Qinit = np.full((Nreps, 6), q0).T
Qdinit = np.zeros((Nreps, 6)).T

t = np.concatenate((tfirst, tsecond))

Q = np.hstack((Qinit, Q1, Q2, Qinit)).T
Qd = np.hstack((Qdinit, Qd1, Qd2, Qdinit)).T
Qdd = np.hstack((Qdinit, Qdd1, Qdd2, Qdinit)).T

q1 = Q[:, 0]
q2 = Q[:, 1]
q3 = Q[:, 2]
q4 = Q[:, 3]
q5 = Q[:, 4]
q6 = Q[:, 5]

t = np.arange(0, dt * len(q1), dt)
N = len(t)

if False:
    plt.figure()
    plt.plot(t, q1, label='q1')
    plt.plot(t, q2, label='q2')
    plt.plot(t, q3, label='q3')
    plt.plot(t, q4, label='q4')
    plt.plot(t, q5, label='q5')
    plt.plot(t, q6, label='q6')
    plt.legend()
    plt.title('Joint positions')

    q1dnum = np.diff(q1) / dt
    q1ddnum = np.diff(q1dnum) / dt

    plt.figure()
    plt.plot(t, Qd[:, 0], label='qd1')
    plt.plot(t, Qd[:, 1], label='qd2')
    plt.plot(t, Qd[:, 2], label='qd3')
    plt.plot(t, Qd[:, 3], label='qd4')
    plt.plot(t, Qd[:, 4], label='qd5')
    plt.plot(t, Qd[:, 5], label='qd6')
    plt.legend()
    plt.title('Joint velocities')

    plt.figure()
    plt.plot(t, Qdd[:, 0], label='qdd1')
    plt.plot(t, Qdd[:, 1], label='qdd2')
    plt.plot(t, Qdd[:, 2], label='qdd3')
    plt.plot(t, Qdd[:, 3], label='qdd4')
    plt.plot(t, Qdd[:, 4], label='qdd5')
    plt.plot(t, Qdd[:, 5], label='qdd6')
    plt.legend()
    plt.title('Joint accelerations')
    plt.show()

mdl = pin.buildModelsFromUrdf('ur.urdf')[0]
data = mdl.createData()
RFid = mdl.getFrameId('tool0')

x = np.zeros(N)
y = np.zeros(N)
z = np.zeros(N)
qx = np.zeros(N)
qy = np.zeros(N)
qz = np.zeros(N)
qw = np.zeros(N)

vx = np.zeros(N)
vy = np.zeros(N)
vz = np.zeros(N)
wx = np.zeros(N)
wy = np.zeros(N)
wz = np.zeros(N)

ax = np.zeros(N)
ay = np.zeros(N)
az = np.zeros(N)
awx = np.zeros(N)
awy = np.zeros(N)
awz = np.zeros(N)

prev_quat = np.zeros(4)

tmp = Rot.from_matrix(np.eye(3)).as_quat()
print('x:', tmp[0])
print('y:', tmp[1])
print('z:', tmp[2])
print('w:', tmp[3])

for i in range(N):
    q = Q[i, :]
    v = Qd[i, :]
    a = Qdd[i, :]

    pin.forwardKinematics(mdl, data, q, v, a)
    frame = pin.updateFramePlacement(mdl, data, RFid)

    v_ee = pin.getFrameVelocity(mdl, data, RFid)
    a_ee = pin.getFrameAcceleration(mdl, data, RFid)

    x[i] = frame.translation[0]
    y[i] = frame.translation[1]
    z[i] = frame.translation[2]

    quat = Rot.from_matrix(frame.rotation).as_quat()
    if quat.dot(np.array([0, 0, 0, 1.0])) < 0:
        quat = -quat
    if i > 0:
        if np.dot(quat, prev_quat) < 0:
            quat = -quat
    prev_quat = quat

    qx[i] = quat[0]
    qy[i] = quat[1]
    qz[i] = quat[2]
    qw[i] = quat[3]

    vx[i] = v_ee.linear[0]
    vy[i] = v_ee.linear[1]
    vz[i] = v_ee.linear[2]
    wx[i] = v_ee.angular[0]
    wy[i] = v_ee.angular[1]
    wz[i] = v_ee.angular[2]

    ax[i] = a_ee.linear[0]
    ay[i] = a_ee.linear[1]
    az[i] = a_ee.linear[2]
    awx[i] = a_ee.angular[0]
    awy[i] = a_ee.angular[1]
    awz[i] = a_ee.angular[2]

Qdnorm = np.linalg.norm(Qd, axis=0)

idx_start = 0
idx_end = N

ones = np.ones(10)
zeros = np.zeros(10)

if True:
    plt.figure()
    plt.plot(t, x, label='x')
    plt.plot(t, y, label='y')
    plt.plot(t, z, label='z')
    plt.title('End-effector position')
    plt.legend()

    plt.figure()
    plt.plot(t, qx, label='qx')
    plt.plot(t, qy, label='qy')
    plt.plot(t, qz, label='qz')
    plt.plot(t, qw, label='qw')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.title('End-effector orientation (quaternion)')
    plt.legend()

if True:
    plt.figure()
    plt.plot(vx, label='vx')
    plt.plot(vy, label='vy')
    plt.plot(vz, label='vz')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.title('End-effector linear velocity')
    plt.legend()

    plt.figure()
    plt.plot(wx, label='wx')
    plt.plot(wy, label='wy')
    plt.plot(wz, label='wz')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.title('End-effector angular velocity')
    plt.legend()

    plt.figure()
    plt.plot(t, ax, label='ax')
    plt.plot(t, ay, label='ay')
    plt.plot(t, az, label='az')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.title('End-effector linear acceleration')
    plt.legend()

    plt.figure()
    plt.plot(t, awx, label='awx')
    plt.plot(t, awy, label='awy')
    plt.plot(t, awz, label='awz')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.title('End-effector angular acceleration')
    plt.legend()

df = pd.DataFrame({
    't': t,
    'x': x,
    'y': y,
    'z': z,
    'qx': qx,
    'qy': qy,
    'qz': qz,
    'qw': qw,
    'vx': vx,
    'vy': vy,
    'vz': vz,
    'wx': wx,
    'wy': wy,
    'wz': wz,
    'ax': ax,
    'ay': ay,
    'az': az,
    'awx': awx,
    'awy': awy,
    'awz': awz
})
df.to_csv(outfile, index=False, header=False)

print('Data saved to', outfile)

plt.show()
