from process_data import ds_handler
from py_localization import EsEkf, ImuMeasurement, State
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


x = []
y = []
for out in outputs:
    x.append(out[0])
    y.append(out[1])

print("Finished!")
print("num outputs: {}".format(len(outputs)))

plt.figure(1)
plt.plot(x, y)
plt.draw()

x = []
y = []
for s in gt_measurements:
    pos = s.get_position()
    x.append(pos[0])
    y.append(pos[1])

plt.figure(2)
plt.plot(x, y, 'blue', x[0], y[0], 'r x')
plt.title("GT Data")
plt.show()