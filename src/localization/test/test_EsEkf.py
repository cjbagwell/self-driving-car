from process_data import ds_handler
from py_localization import EsEkf, ImuMeasurement, State

imu_measurements = ds_handler.get_imu_data()
gt_measurements  = ds_handler.get_gt_data()
gt_times = []
[gt_times.append(state.time) for state in gt_measurements]

gt_index = gt_times.index(imu_measurements[10].get_time())
init_pose = gt_measurements[gt_index]
print(init_pose)

initState = State()


for m in imu_measurements:
    pass