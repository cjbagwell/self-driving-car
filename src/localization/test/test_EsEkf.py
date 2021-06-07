from process_data import ds_handler
from py_localization import EsEkf, ImuMeasurement, State

imu_measurements = ds_handler.get_imu_data()
gt_measurements  = ds_handler.get_gt_data()
gt_times = []
[gt_times.append(state.time) for state in gt_measurements]

gt_index = gt_times.index(imu_measurements[0].get_time())
init_state = gt_measurements[gt_index]
imu_var = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
print(init_state)



filter = EsEkf(init_state, [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
print(filter)

newState = filter.run_step(imu_measurements[0], imu_var)
print(newState)

for m in imu_measurements:
    pass