from process_data import ds_handler #type:ignore
from py_localization import EsEkf, ImuMeasurement, State #type:ignore
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
    outputs.append(filter.run_step(m, imu_var).get_position())


x_out = []
y_out = []
for out in outputs:
    x_out.append(out[0])
    y_out.append(out[1])

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
plt.plot(x_gt, y_gt, 'blue', x_out, y_out, 'red', x_gt[0], y_gt[0], 'x')
plt.xlabel("x position")
plt.ylabel("y position")
plt.title("Position")
plt.legend(["ground truth", "estimated", "start"])
plt.draw()

plt.figure(2)
plt.plot(t, yaw_gt, 'b--')
plt.draw()

plt.show()

