import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import glob
import os
import py_localization as loc  # type:ignore

# helper functions


def latlon2position(lat, lon):
    r_earth = 6.371*10**6
    x = lon * r_earth
    y = -lat * r_earth
    return x, y


def deg2rad(angle):
    return angle * 3.14159 / 180


class DatasetHandler:
    def __init__(self, dataset_dir):
        self.dataset_dir = dataset_dir if dataset_dir[-1] == '/' else dataset_dir + '/'
        self.imu_raw = {}
        self.gnss_raw = {}
        self.camera_raw = {}
        self.gt_raw = {}
        self.init_imu_data()
        self.init_gnss_data()
        self.init_camera_data()
        self.init_gt_data()

    def init_gnss_data(self):
        data_file = open(self.dataset_dir + "GNSS_data.txt", 'r')
        self.gnss_raw["frames"] = []
        self.gnss_raw["times"] = []
        self.gnss_raw["lats"] = []
        self.gnss_raw["lons"] = []
        self.gnss_raw["alts"] = []
        self.gnss_raw["xs"] = []
        self.gnss_raw["ys"] = []

        # split data
        lns = data_file.read().splitlines()
        data_file.close()
        labels = lns[0].split(',')
        print(labels)
        frame_index = labels.index('frame')
        time_index = labels.index('time')
        lat_index = labels.index('lat')
        lon_index = labels.index('lon')
        alt_index = labels.index('alt')

        for i in range(1, len(labels)-1):
            elems = [elem for elem in lns[i].split(',')]
            self.gnss_raw["frames"].append(int(elems[frame_index]))
            self.gnss_raw["times"].append(float(elems[time_index]))
            self.gnss_raw["lats"].append(np.deg2rad(float(elems[lat_index])))
            self.gnss_raw["lons"].append(np.deg2rad(float(elems[lon_index])))
            self.gnss_raw["alts"].append(float(elems[alt_index]))

        # convert lat lons to positions
        gnsp = [latlon2position(lat, lon) for lat, lon in zip(
            self.gnss_raw["lats"], self.gnss_raw["lons"])]
        self.gnss_raw["xs"] = [x for x, y in gnsp]
        self.gnss_raw["ys"] = [y for x, y in gnsp]

    def init_imu_data(self):
        data_file = open(self.dataset_dir + "IMU_data.txt", 'r')
        self.imu_raw['frames'] = []
        self.imu_raw['times'] = []
        self.imu_raw['accel_xs'] = []
        self.imu_raw['accel_ys'] = []
        self.imu_raw['accel_zs'] = []
        self.imu_raw['compas'] = []
        self.imu_raw['gyro_xs'] = []
        self.imu_raw['gyro_ys'] = []
        self.imu_raw['gyro_zs'] = []

        # split data
        lns = data_file.read().splitlines()
        data_file.close()
        labels = lns[0].split(',')
        frame_index =   labels.index('frame')
        time_index =    labels.index('time')
        accel_x_index = labels.index('accel_x')
        accel_y_index = labels.index('accel_y')
        accel_z_index = labels.index('accel_z')
        compas_index =  labels.index('compas')
        gyro_x_index =  labels.index('gyro_x')
        gyro_y_index =  labels.index('gyro_y')
        gyro_z_index =  labels.index('gyro_z')

        print(labels)
        for i in range(1, len(lns)-1):
            ln = lns[i]
            elems = [elem for elem in ln.split(',')]
            self.imu_raw['frames'  ].append(int(elems[frame_index]))
            self.imu_raw['times'   ].append(float(elems[time_index]))
            self.imu_raw['accel_xs'].append(float(elems[accel_x_index]))
            self.imu_raw['accel_ys'].append(float(elems[accel_y_index]))
            self.imu_raw['accel_zs'].append(float(elems[accel_z_index]))
            self.imu_raw['compas'  ].append(float(elems[compas_index]))
            self.imu_raw['gyro_xs' ].append(float(elems[gyro_x_index]))
            self.imu_raw['gyro_ys' ].append(float(elems[gyro_y_index]))
            self.imu_raw['gyro_zs' ].append(float(elems[gyro_z_index]))

    def init_camera_data(self):
        img_dir = os.path.join(self.dataset_dir, "images/")
        self.camera_raw['frames'] = []
        self.camera_raw['times'] = []
        self.camera_raw['image_paths'] = []

        data_file = open(self.dataset_dir + "CAMERA_data.txt", 'r')
        lns = data_file.read().splitlines()
        labels = lns[0].split(',')
        print(labels)

        frame_index = labels.index('frame')
        time_index =  labels.index('time')
        name_index =  labels.index('file_name')

        data_file.close()
        for i in range(1, len(lns)-1):
            elems = [elem for elem in lns[i].split(',')]
            self.camera_raw['frames'].append(int(elems[frame_index]))
            self.camera_raw['times'].append(float(elems[time_index]))
            self.camera_raw['image_paths'].append(
                os.path.join(img_dir, elems[name_index]))

    def init_gt_data(self):
        data_file = open(self.dataset_dir + "GT_data.txt", 'r')
        self.gt_raw['frames'] = []
        self.gt_raw['times'] = []
        self.gt_raw['xs'] = []
        self.gt_raw['ys'] = []
        self.gt_raw['zs'] = []
        self.gt_raw['vxs'] = []
        self.gt_raw['vys'] = []
        self.gt_raw['vzs'] = []
        self.gt_raw['rolls'] = []
        self.gt_raw['pitches'] = []
        self.gt_raw['yaws'] = []

        # split data
        lns = data_file.read().splitlines()
        labels = lns[0].split(',')
        print(labels)

        frame_index =   labels.index('frame')
        time_index =    labels.index('time')
        x_index =       labels.index('x')
        y_index =       labels.index('y')
        z_index =       labels.index('z')
        vx_index =      labels.index('vx')
        vy_index =      labels.index('vy')
        vz_index =      labels.index('vz')
        roll_index =    labels.index('roll')
        pitche_index =  labels.index('pitch')
        yaw_index =     labels.index('yaw')

        data_file.close()
        for i in range(1, len(lns)-1):
            elems = [elem for elem in lns[i].split(',')]
            self.gt_raw['frames' ].append(int(elems[frame_index]))
            self.gt_raw['times'  ].append(float(elems[time_index]))
            self.gt_raw['xs'     ].append(float(elems[x_index]))
            self.gt_raw['ys'     ].append(float(elems[y_index]))
            self.gt_raw['zs'     ].append(float(elems[z_index]))
            self.gt_raw['vxs'    ].append(float(elems[vx_index]))
            self.gt_raw['vys'    ].append(float(elems[vy_index]))
            self.gt_raw['vzs'    ].append(float(elems[vz_index]))
            self.gt_raw['rolls'  ].append(np.deg2rad(float(elems[roll_index])))
            self.gt_raw['pitches'].append(np.deg2rad(float(elems[pitche_index])))
            self.gt_raw['yaws'   ].append(np.deg2rad(float(elems[yaw_index])))

    def get_gnss_measurements(self):
        gnss_measurements = []
        for frame, t, lat, lon, alt in zip(self.gnss_raw['frames'],
                                           self.gnss_raw['times'],
                                           self.gnss_raw['lats'],
                                           self.gnss_raw['lons'],
                                           self.gnss_raw['alts']):
            gnss_measurements.append(loc.GnssMeasurement(frame, t, alt, lat, lon))
        return gnss_measurements

    def get_imu_measurements(self):
        imu_measurements = []
        for frame, t, ax, ay, az, c, gx, gy, gz in zip(self.imu_raw['frames'],
                                     self.imu_raw['times'],
                                     self.imu_raw['accel_xs'],
                                     self.imu_raw['accel_ys'],
                                     self.imu_raw['accel_zs'],
                                     self.imu_raw['compas'],
                                     self.imu_raw['gyro_xs'],
                                     self.imu_raw['gyro_ys'],
                                     self.imu_raw['gyro_zs']):
            # TODO: implement frame in ImuMeasurement
            imu_measurements.append(loc.ImuMeasurement([ax,ay,az], c, [gx, gy, gz], t))

        return imu_measurements

    def get_gt_states(self):
        states = []
        for (frame, t, x, y, z, vx, vy, vz, roll, pitch, yaw) in zip(
                                                              self.gt_raw['frames'],
                                                              self.gt_raw['times'],
                                                              self.gt_raw['xs'],
                                                              self.gt_raw['ys'],
                                                              self.gt_raw['zs'],
                                                              self.gt_raw['vxs'],
                                                              self.gt_raw['vys'],
                                                              self.gt_raw['vzs'],
                                                              self.gt_raw['rolls'],
                                                              self.gt_raw['pitches'],
                                                              self.gt_raw['yaws']):
            rot = loc.Quaternion([roll, pitch, yaw], False).as_vector()
            states.append(loc.State([x, y, z], [vx, vy, vz], rot, t, frame))
        return states

    def get_vo_training_set(self):
        pass

    def get_vo_test_set(self):
        pass
