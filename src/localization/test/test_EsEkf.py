from process_data import ds_handler #type:ignore
from py_localization import * #EsEkf, ImuMeasurement, State, quat_to_euler #type:ignore
import numpy as np
from matplotlib import pyplot as plt

imu_measurements = ds_handler.get_imu_measurements()
gnss_measurements  = ds_handler.get_gnss_measurements()
gt_measurements  = ds_handler.get_gt_measurements()
gt_times = []
[gt_times.append(state.time) for state in gt_measurements]
gns_times = []
[gns_times.append(m.t) for m in gnss_measurements]

gt_index = gt_times.index(imu_measurements[0].get_time())
init_state = gt_measurements[gt_index]
imu_var = np.asarray([1, 1, 1, 1, 1, 1]) * 0.1
gnss_var = np.asarray([1, 1, 1]) * 0.01

filter = EsEkf(init_state, [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
print(filter)

outputs = []
outs_gnss = []
gnss_index = 0
for i in range(len(imu_measurements)):
    out = filter.run_step(imu_measurements[i], imu_var)
    imu_t = imu_measurements[i].get_time() 

    # for j in range(gnss_index, len(gnss_measurements)):
    #     gns_t = gnss_measurements[j].t
    #     # print("time diff: {}".format(gns_t - imu_t))
    #     if abs(gns_t - imu_t) < 0.2 and (gns_t - imu_t) > 0.0:
    #         out = filter.run_step(gnss_measurements[j], gnss_var)
    #         outs_gnss.append(out)
    #         gnss_index = j + 1
    #         print("gnss_measurement index: {}\t\tdt: {}".format(gnss_index, gns_t - imu_t))
    #         break

    outputs.append(out)



print("Finished!")
print("num outputs: {}".format(len(outputs)))

# process output data
t_out = []
x_out = []
y_out = []
z_out = []
vx_out = []
vy_out = []
vz_out = []
roll_out = []
pitch_out = []
yaw_out = []
for out in outputs:
    t_out.append(out.time)
    x_out.append(out.get_position()[0])
    y_out.append(out.get_position()[1])
    z_out.append(out.get_position()[2])
    vx_out.append(out.get_velocity()[0])
    vy_out.append(out.get_velocity()[1])
    vz_out.append(out.get_velocity()[2])
    e_angles = quat_to_euler(out.rot)
    roll_out.append(e_angles[0])
    pitch_out.append(e_angles[1])
    yaw_out.append(e_angles[2])


# process ground truth data
t_gt = ds_handler.gt_raw['time']
x_gt = ds_handler.gt_raw['x']
y_gt = ds_handler.gt_raw['y']
z_gt = ds_handler.gt_raw['z']
vx_gt = ds_handler.gt_raw['vx']
vy_gt = ds_handler.gt_raw['vy']
vz_gt = ds_handler.gt_raw['vz']
roll_gt = ds_handler.gt_raw['roll']
pitch_gt = ds_handler.gt_raw['pitch']
yaw_gt = ds_handler.gt_raw['yaw']


# process imu data
t_imu = ds_handler.imu_raw['time']
accel_x = ds_handler.imu_raw['accel_x']
accel_y = ds_handler.imu_raw['accel_y']
accel_z = ds_handler.imu_raw['accel_z']
compas = ds_handler.imu_raw['compas']
gyro_x = ds_handler.imu_raw['gyro_x']
gyro_y = ds_handler.imu_raw['gyro_y']
gyro_z = ds_handler.imu_raw['gyro_z']


# process gnss data
t_gnss = ds_handler.gnss_raw['times']
x_gnss = ds_handler.gnss_raw['xs']
y_gnss = ds_handler.gnss_raw['ys']
x_meas = [gnss_2_position(m)[0] for m in gnss_measurements]
y_meas = [gnss_2_position(m)[1] for m in gnss_measurements]
z_meas = [gnss_2_position(m)[2] for m in gnss_measurements]
alt_gnss = ds_handler.gnss_raw['alts']
lat_gnss = ds_handler.gnss_raw['lats']
lon_gnss = ds_handler.gnss_raw['lons']


# plot position
# plt.figure(1)
# plt.plot(x_gt, y_gt, 'b--', x_out, y_out, 'red', x_gt[0], y_gt[0], 'x')
# plt.xlabel("x position")
# plt.ylabel("y position")
# plt.title("Position")
# plt.legend(["ground truth", "estimated", "start"])

ax = plt.axes(projection='3d')
ax.plot3D(x_gt, y_gt, z_gt, '--b')
ax.plot3D(x_out, y_out, z_out, 'r')
# ax.plot3D(x_meas, y_meas, z_meas, '*')
ax.set_xlabel('Easting [m]')
ax.set_ylabel('Northing [m]')
ax.set_zlabel('Up [m]')
ax.set_title('Ground Truth and Estimated Trajectory')
ax.set_zlim(-1, 1)
ax.legend(loc=(0.62,0.77))
# ax.view_init(elev=45, azim=-50)

plt.draw()

# plot position and rotation
plt.figure(2)
plt.subplot(2,3,1)
plt.plot(t_gt, yaw_gt, 'b--', t_out, yaw_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Yaw")
plt.xlabel("time")
plt.ylabel("rad")
plt.draw()

plt.subplot(2,3,2)
plt.plot(t_gt, roll_gt, 'b--', t_out, roll_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Roll")
plt.xlabel("time")
plt.ylabel("rad")
plt.draw()

plt.subplot(2,3,3)
plt.plot(t_gt, pitch_gt, 'b--', t_out, pitch_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Pitch")
plt.xlabel("time")
plt.ylabel("rad")
plt.draw()

plt.subplot(2,3,4)
plt.plot(t_gt, x_gt, 'b--', t_out, x_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("X vs Time")
plt.xlabel("time")
plt.ylabel("x position")
plt.draw()

plt.subplot(2,3,5)
plt.plot(t_gt, y_gt, 'b--', t_out, y_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Y vs Time")
plt.xlabel("time")
plt.ylabel("y position")
plt.draw()

plt.subplot(2,3,6)
plt.plot(t_gt, z_gt, 'b--', t_out, z_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Z vs Time")
plt.xlabel("time")
plt.ylabel("z position")
plt.draw()

# plot velocity
plt.figure(3)
plt.subplot(1,3,1)
plt.plot(t_gt, vx_gt, 'b--', t_out, vx_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("X velocity vs Time")
plt.xlabel("time")
plt.ylabel("x velocity")
plt.draw()

plt.subplot(1,3,2)
plt.plot(t_gt, vy_gt, 'b--', t_out, vy_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Y velocity vs Time")
plt.xlabel("time")
plt.ylabel("y velocity")
plt.draw()

plt.subplot(1,3,3)
plt.plot(t_gt, vz_gt, 'b--', t_out, vz_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Z velocity vs Time")
plt.xlabel("time")
plt.ylabel("z velocity")
plt.draw()

# plot imu data
# plt.figure(4)
# plt.subplot(2,3,1)
# plt.plot(t_imu, accel_x, 'b--')
# plt.title("X Acceleration vs Time")
# plt.xlabel("time")
# plt.ylabel("x acceleration")
# plt.draw()

# plt.subplot(2,3,2)
# plt.plot(t_imu, accel_y, 'b--')
# plt.title("Y Acceleration vs Time")
# plt.xlabel("time")
# plt.ylabel("y acceleration")
# plt.draw()

# plt.subplot(2,3,3)
# plt.plot(t_imu, accel_z, 'b--')
# plt.title("Z Acceleration vs Time")
# plt.xlabel("time")
# plt.ylabel("z acceleration")
# plt.draw()

# plt.subplot(2,3,4)
# plt.plot(t_imu, gyro_x, 'b--')
# plt.title("X Rotation vs Time")
# plt.xlabel("time")
# plt.ylabel("x angular velocity")
# plt.draw()

# plt.subplot(2,3,5)
# plt.plot(t_imu, gyro_y, 'b--')
# plt.title("Y Rotation vs Time")
# plt.xlabel("time")
# plt.ylabel("y angular velocity")
# plt.draw()

# plt.subplot(2,3,6)
# plt.plot(t_imu, gyro_z, 'b--')
# plt.title("Z Rotation vs Time")
# plt.xlabel("time")
# plt.ylabel("z angular velocity")
# plt.draw()


plt.show()

