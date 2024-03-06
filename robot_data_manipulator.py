import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import pinocchio as pin
import os.path
from scipy.spatial.transform import Rotation as Rot


filename = os.path.expanduser(os.path.join('~', 'dmps', 'data', 'joint_states.csv'))
prj_dir = os.path.dirname(__file__)
outfile = os.path.expanduser(os.path.join(prj_dir, 'test', 'data', 'robot.csv'))

df = pd.read_csv(filename)
dt = 0.002

idx_start = 170
idx_end = 2250

q1 = df.get('q1').values[idx_start:idx_end]
q2 = df.get('q2').values[idx_start:idx_end]
q3 = df.get('q3').values[idx_start:idx_end]
q4 = df.get('q4').values[idx_start:idx_end]
q5 = df.get('q5').values[idx_start:idx_end]
q6 = df.get('q6').values[idx_start:idx_end]
qd1 = df.get('v1').values[idx_start:idx_end]
qd2 = df.get('v2').values[idx_start:idx_end]
qd3 = df.get('v3').values[idx_start:idx_end]
qd4 = df.get('v4').values[idx_start:idx_end]
qd5 = df.get('v5').values[idx_start:idx_end]
qd6 = df.get('v6').values[idx_start:idx_end]

Q = np.array([q1, q2, q3, q4, q5, q6])
Qd = np.array([qd1, qd2, qd3, qd4, qd5, qd6])

Q = np.hstack((Q, np.flip(Q, axis=1)))
Qd = np.hstack((Qd, np.flip(Qd, axis=1)))
Qdd = np.hstack((np.zeros((6,1)), np.diff(Qd, axis=1) / dt))

q1 = Q[0, :]
q2 = Q[1, :]
q3 = Q[2, :]
q4 = Q[3, :]
q5 = Q[4, :]
q6 = Q[5, :]
qd1 = Qd[0, :]
qd2 = Qd[1, :]
qd3 = Qd[2, :]
qd4 = Qd[3, :]
qd5 = Qd[4, :]
qd6 = Qd[5, :]
N = len(q1)
t = np.arange(0, dt * N, dt)

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

    plt.figure()
    plt.plot(t, Qd[0], label='qd1')
    plt.plot(t, Qd[1], label='qd2')
    plt.plot(t, Qd[2], label='qd3')
    plt.plot(t, Qd[3], label='qd4')
    plt.plot(t, Qd[4], label='qd5')
    plt.plot(t, Qd[5], label='qd6')
    plt.legend()
    plt.title('Joint velocities')

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

print("Q:", Q.shape)

for i in range(N):
    q = Q[:, i]
    v = Qd[:, i]
    a = Qdd[:, i]

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

if False:
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

plt.show()
