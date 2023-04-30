import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

lidar_pd = pd.read_csv("lidar_results.csv")
radar_pd = pd.read_csv("radar_results.csv")

lidar_pos = np.asarray(lidar_pd)[:, 1:3]
radar_pos = np.asarray(radar_pd)[:, 1:3]

plt.figure(figsize=(8, 6))
plt.plot(lidar_pos[:, 0], lidar_pos[:, 1], linewidth=4, label="lidar + IMU")
plt.plot(radar_pos[:, 0], radar_pos[:, 1], "--", label="radar + IMU")
plt.scatter(53.9, -3.43, marker="*", c="r", label="start")
plt.scatter(24.5, 3.14, marker="*", c="b", label="end")

plt.title("Relocalization results")
plt.legend(loc="lower right")
plt.savefig("results.png", dpi=400, bbox_inches="tight")
plt.show()

lidar_odom = pd.read_csv('lidar_odom.csv')
radar_odom = pd.read_csv('radar_odom.csv')

lidar_odom['stamp'] -= lidar_odom['stamp'][0]
lidar_odom['std_x'] = np.sqrt(lidar_odom['var_x'].to_numpy())
lidar_odom['std_y'] = np.sqrt(lidar_odom['var_y'].to_numpy())
lidar_odom['std_theta'] = np.sqrt(lidar_odom['var_theta'].to_numpy())

radar_odom['stamp'] -= radar_odom['stamp'][0]
radar_odom['std_x'] = np.sqrt(radar_odom['var_x'].to_numpy())
radar_odom['std_y'] = np.sqrt(radar_odom['var_y'].to_numpy())
radar_odom['std_theta'] = np.sqrt(radar_odom['var_theta'].to_numpy())

figure, axis = plt.subplots(3, 1, figsize=(8, 8))
axis[0].set_title("pose standard deviation")

axis[0].plot(lidar_odom['stamp'], lidar_odom['std_x'], label='lidar')
axis[0].plot(radar_odom['stamp'], radar_odom['std_x'], label='radar')
axis[0].legend()

axis[1].plot(lidar_odom['stamp'], lidar_odom['std_y'])
axis[1].plot(radar_odom['stamp'], radar_odom['std_y'])


axis[2].plot(lidar_odom['stamp'], lidar_odom['std_theta'])
axis[2].plot(radar_odom['stamp'], radar_odom['std_theta'])


axis[0].set(ylabel='std x')
axis[1].set(ylabel='std y')
axis[2].set(ylabel='std theta', xlabel='time')

plt.savefig('pose_cov', dpi=400, bbox_inches="tight")
plt.show()

f_l_x = interpolate.interp1d(lidar_odom['stamp'], lidar_odom['x'])
lidar_x = f_l_x(radar_odom['stamp'])

f_l_y = interpolate.interp1d(lidar_odom['stamp'], lidar_odom['y'])
lidar_y = f_l_y(radar_odom['stamp'])

quat_lidar = np.asarray(
    [lidar_odom['qx'], lidar_odom['qy'], lidar_odom['qz'], lidar_odom['qw']]).T
quat_lidar = R.from_quat(quat_lidar)

slerp = Slerp(lidar_odom['stamp'], quat_lidar)
lidar_quat = slerp(radar_odom['stamp'])

radar_quat = np.asarray(
    [radar_odom['qx'], radar_odom['qy'], radar_odom['qz'], radar_odom['qw']]).T
radar_quat = R.from_quat(radar_quat)

diff = lidar_quat.inv() * radar_quat
diff = diff.as_euler('xyz')[:, 2]

pos_err = np.array([radar_odom['x']-lidar_x, radar_odom['y']-lidar_y]).T
pos_err = np.linalg.norm(pos_err, axis=1)

figure, axis = plt.subplots(2, 1, figsize=(8, 6))
axis[0].set_title("pose error")
axis[0].plot(radar_odom['stamp'], pos_err)

axis[1].plot(radar_odom['stamp'], diff, color="orange")

axis[0].set(ylabel='position error (m)')
axis[1].set(ylabel='heading error (rad)', xlabel='time')

plt.savefig('pos_err', dpi=400, bbox_inches="tight")
plt.show()
