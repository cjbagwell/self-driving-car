from process_data import ds_handler #type:ignore
from py_localization import EsEkf, ImuMeasurement, State, quat_to_euler #type:ignore
import numpy as np
from matplotlib import pyplot as plt

imu_measurements = ds_handler.get_imu_data()
gt_measurements  = ds_handler.get_gt_data()
gt_times = []
[gt_times.append(state.time) for state in gt_measurements]

gt_index = gt_times.index(imu_measurements[0].get_time())
init_state = gt_measurements[gt_index]
imu_var = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

filter = EsEkf(init_state, [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
print(filter)

outputs = []
for m in imu_measurements:
    outputs.append(filter.run_step(m, imu_var))

t_out = []
x_out = []
y_out = []
z_out = []
roll_out = []
pitch_out = []
yaw_out = []
for out in outputs:
    t_out.append(out.time)
    x_out.append(out.get_position()[0])
    y_out.append(out.get_position()[1])
    z_out.append(out.get_position()[2])
    e_angles = quat_to_euler(out.rot)
    roll_out.append(e_angles[0])
    pitch_out.append(e_angles[1])
    yaw_out.append(e_angles[2])


print("Finished!")
print("num outputs: {}".format(len(outputs)))
t = ds_handler.gt_raw['time']
x_gt = ds_handler.gt_raw['x']
y_gt = ds_handler.gt_raw['y']
z_gt = ds_handler.gt_raw['z']
roll_gt = ds_handler.gt_raw['roll']
pitch_gt = ds_handler.gt_raw['pitch']
yaw_gt = ds_handler.gt_raw['yaw']

plt.figure(1)
plt.plot(x_gt, y_gt, 'b--', x_out, y_out, 'red', x_gt[0], y_gt[0], 'x')
plt.xlabel("x position")
plt.ylabel("y position")
plt.title("Position")
plt.legend(["ground truth", "estimated", "start"])
plt.draw()

plt.figure(2)
plt.subplot(2,3,1)
plt.plot(t, yaw_gt, 'b--', t_out, yaw_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Yaw")
plt.xlabel("time")
plt.ylabel("rad")
plt.draw()

plt.subplot(2,3,2)
plt.plot(t, roll_gt, 'b--', t_out, roll_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Roll")
plt.xlabel("time")
plt.ylabel("rad")
plt.draw()

plt.subplot(2,3,3)
plt.plot(t, pitch_gt, 'b--', t_out, pitch_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Pitch")
plt.xlabel("time")
plt.ylabel("rad")
plt.draw()

plt.subplot(2,3,4)
plt.plot(t, x_gt, 'b--', t_out, x_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("X vs Time")
plt.xlabel("time")
plt.ylabel("x position")
plt.draw()

plt.subplot(2,3,5)
plt.plot(t, y_gt, 'b--', t_out, y_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Y vs Time")
plt.xlabel("time")
plt.ylabel("y position")
plt.draw()

plt.subplot(2,3,6)
plt.plot(t, z_gt, 'b--', t_out, z_out, 'r')
plt.legend(["ground truth","estimated"])
plt.title("Z vs Time")
plt.xlabel("time")
plt.ylabel("z position")
plt.draw()

plt.show()

