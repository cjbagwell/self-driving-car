import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import random
import math
import sys
import os
import torch
from torch._C import dtype
from torch.utils.data import Dataset, DataLoader

sys.path.append(os.path.join("/home/jordan/Projects/self-driving-car"))
try:
    import build.src.localization.py_localization as loc  # type:ignore
except:
    print("Error finding localization library")

# helper functions
def latlon2position(lat, lon):
    r_earth = 6.371*10**6
    x = lon * r_earth
    y = -lat * r_earth
    return x, y

class CarlaDataset(Dataset):
    def __init__(self, dataset_dir, load_vo_dataset=True):
        self.dataset_dir = dataset_dir if dataset_dir[-1] == '/' else dataset_dir + '/'
        self.imu_raw    = {}
        self.gnss_raw   = {}
        self.camera_raw = {}
        self.gt_raw     = {}
        self.vo_raw     = {}
        self.num_vo_frames = -1
        
        # Load in Data
        self.init_imu_data()
        self.init_gnss_data()
        self.init_camera_data()
        self.init_gt_data()
        if load_vo_dataset:
            self.init_vo_data()
    
    def init_gnss_data(self):
        data_file = open(self.dataset_dir + "GNSS_data.txt", 'r')
        self.gnss_raw["frames"  ] = []
        self.gnss_raw["times"   ] = []
        self.gnss_raw["lats"    ] = []
        self.gnss_raw["lons"    ] = []
        self.gnss_raw["alts"    ] = []
        self.gnss_raw["xs"      ] = []
        self.gnss_raw["ys"      ] = []

        # split data
        lns = data_file.read().splitlines()
        data_file.close()
        labels = lns[0].split(',')
        
        frame_index = labels.index('frame')
        time_index  = labels.index('time')
        lat_index   = labels.index('lat')
        lon_index   = labels.index('lon')
        alt_index   = labels.index('alt')

        for i in range(1, len(labels)-1):
            elems = [elem for elem in lns[i].split(',')]
            self.gnss_raw["frames"  ].append(int(elems[frame_index]))
            self.gnss_raw["times"   ].append(float(elems[time_index]))
            self.gnss_raw["lats"    ].append(np.deg2rad(float(elems[lat_index])))
            self.gnss_raw["lons"    ].append(np.deg2rad(float(elems[lon_index])))
            self.gnss_raw["alts"    ].append(float(elems[alt_index]))

        # convert lat lons to positions
        gnsp = [latlon2position(lat, lon) for lat, lon in zip(self.gnss_raw["lats"], self.gnss_raw["lons"])]
        self.gnss_raw["xs"] = [x for x, y in gnsp]
        self.gnss_raw["ys"] = [y for x, y in gnsp]

    def init_imu_data(self):
        data_file = open(self.dataset_dir + "IMU_data.txt", 'r')
        self.imu_raw['frames'   ] = []
        self.imu_raw['times'    ] = []
        self.imu_raw['accel_xs' ] = []
        self.imu_raw['accel_ys' ] = []
        self.imu_raw['accel_zs' ] = []
        self.imu_raw['compas'   ] = []
        self.imu_raw['gyro_xs'  ] = []
        self.imu_raw['gyro_ys'  ] = []
        self.imu_raw['gyro_zs'  ] = []

        # split data
        lns = data_file.read().splitlines()
        data_file.close()
        labels = lns[0].split(',')
        frame_index     = labels.index('frame')
        time_index      = labels.index('time')
        accel_x_index   = labels.index('accel_x')
        accel_y_index   = labels.index('accel_y')
        accel_z_index   = labels.index('accel_z')
        compas_index    = labels.index('compas')
        gyro_x_index    = labels.index('gyro_x')
        gyro_y_index    = labels.index('gyro_y')
        gyro_z_index    = labels.index('gyro_z')

        for i in range(1, len(lns)-1):
            ln = lns[i]
            elems = [elem for elem in ln.split(',')]
            self.imu_raw['frames'   ].append(  int(elems[frame_index]))
            self.imu_raw['times'    ].append(float(elems[time_index]))
            self.imu_raw['accel_xs' ].append(float(elems[accel_x_index]))
            self.imu_raw['accel_ys' ].append(float(elems[accel_y_index]))
            self.imu_raw['accel_zs' ].append(float(elems[accel_z_index]))
            self.imu_raw['compas'   ].append(float(elems[compas_index]))
            self.imu_raw['gyro_xs'  ].append(float(elems[gyro_x_index]))
            self.imu_raw['gyro_ys'  ].append(float(elems[gyro_y_index]))
            self.imu_raw['gyro_zs'  ].append(float(elems[gyro_z_index]))

    def init_camera_data(self):
        img_dir = os.path.join(self.dataset_dir, "images/")
        self.camera_raw['frames'] = []
        self.camera_raw['times' ] = []
        self.camera_raw['image_paths'] = []

        data_file = open(self.dataset_dir + "CAMERA_data.txt", 'r')
        lns = data_file.read().splitlines()
        labels = lns[0].split(',')

        frame_index = labels.index('frame')
        time_index  = labels.index('time')
        name_index  = labels.index('file_name')

        data_file.close()
        for i in range(1, len(lns)-1):
            elems = [elem for elem in lns[i].split(',')]
            self.camera_raw['frames'].append(int(elems[frame_index]))
            self.camera_raw['times' ].append(float(elems[time_index]))
            self.camera_raw['image_paths'].append(os.path.join(img_dir, elems[name_index]))

    def init_gt_data(self):
        data_file = open(self.dataset_dir + "GT_data.txt", 'r')
        self.gt_raw['frames'  ] = []
        self.gt_raw['times'   ] = []
        self.gt_raw['xs'      ] = []
        self.gt_raw['ys'      ] = []
        self.gt_raw['zs'      ] = []
        self.gt_raw['vxs'     ] = []
        self.gt_raw['vys'     ] = []
        self.gt_raw['vzs'     ] = []
        self.gt_raw['rolls'   ] = []
        self.gt_raw['pitches' ] = []
        self.gt_raw['yaws'    ] = []

        # split data
        lns = data_file.read().splitlines()
        labels = lns[0].split(',')

        frame_index  = labels.index('frame')
        time_index   = labels.index('time')
        x_index      = labels.index('x')
        y_index      = labels.index('y')
        z_index      = labels.index('z')
        vx_index     = labels.index('vx')
        vy_index     = labels.index('vy')
        vz_index     = labels.index('vz')
        roll_index   = labels.index('roll')
        pitche_index = labels.index('pitch')
        yaw_index    = labels.index('yaw')

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

    def init_vo_data(self):
        gt_states = self.get_gt_states()
        num_frames = len(self.camera_raw['frames'])
        frame0 = self.camera_raw['frames'][0]
        gt_index = self.gt_raw['frames'].index(frame0)
        im0 = cv.imread(self.camera_raw['image_paths'][0], cv.IMREAD_COLOR)
        NUM_CHANNELS = im0.shape[2]
        HEIGHT = im0.shape[0]
        WIDTH = im0.shape[1]
        self.vo_raw['images']       = torch.zeros(num_frames, 2*NUM_CHANNELS, HEIGHT, WIDTH, dtype=torch.uint8)
        self.vo_raw['out_state']    = torch.zeros(num_frames, 9)
        
        # Load images
        images = []
        i = 0
        print("loading images")
        for path in self.camera_raw['image_paths']:
            image = cv.imread(path, cv.IMREAD_COLOR)
            image = torch.as_tensor(image, dtype=torch.uint8)
            image = image.reshape(NUM_CHANNELS, HEIGHT, WIDTH)
            images.append(image)
            i+=1
            print("loading image {} of {}".format(i, num_frames))

        # Process frames
        prev_state = gt_states[gt_index]
        for i in range(1, len(self.camera_raw['frames']) - 1):
            # update frame vars
            frame = self.camera_raw['frames'][i]
            img_prev = images[i-1]
            img = images[i]
            gt_index = self.gt_raw['frames'].index(frame, gt_index)
            state = gt_states[gt_index]
            r = torch.as_tensor(loc.quat_to_mat(state.rot)).reshape(3,3)

            # change in orientation
            q_change = prev_state.rot.get_inverse() * state.rot
            e_angles = torch.as_tensor(loc.quat_to_euler(q_change)).reshape(3,1)

            # change in position (camera frame)
            prev_pos = torch.as_tensor(prev_state.get_position()).reshape(3,1)
            pos = torch.as_tensor(state.get_position()).reshape(3,1)
            pos_c = r.inverse().matmul(pos)
            prev_pos_c = r.inverse().matmul(prev_pos)
            pos_change = pos_c - prev_pos_c
            
            #change in velocity
            vel = torch.as_tensor(state.get_velocity()).reshape(3,1)
            vel_c = r.matmul(vel)

            state_vec = torch.zeros(9,1)
            state_vec[0:3, :] = pos_change
            state_vec[3:6, :] = vel_c
            state_vec[6:, :]  = e_angles
            input_volume = torch.zeros(2*NUM_CHANNELS, HEIGHT, WIDTH, dtype=torch.uint8)
            input_volume[0:3] = img
            input_volume[3: ] = img_prev
            
            self.vo_raw['images'   ][i-1] = input_volume
            self.vo_raw['out_state'][i-1] = state_vec.reshape(9)

            # Update loop variables
            prev_state = state

        self.num_vo_frames = self.vo_raw['images'].shape[0]
        return 

    def __getitem__(self, index):
        return self.vo_raw['images'][index], self.vo_raw['out_state'][index]

    def __len__(self):
        return self.num_vo_frames

    def get_gnss_measurements(self):
        gnss_measurements = []
        for frame, t, lat, lon, alt in zip(self.gnss_raw['frames'],
                                           self.gnss_raw['times'],
                                           self.gnss_raw['lats'],
                                           self.gnss_raw['lons'],
                                           self.gnss_raw['alts']):
            gnss_measurements.append(
                loc.GnssMeasurement(frame, t, alt, lat, lon))
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
            imu_measurements.append(loc.ImuMeasurement(
                [ax, ay, az], c, [gx, gy, gz], t))

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

def test():
    dataset_path = os.path.join("/home/jordan/Datasets/CarlaDatasets", "TestDataset01")
    dataset = CarlaDataset(dataset_path)
    dataloader = DataLoader(dataset=dataset, batch_size=4, shuffle=False, num_workers=2)
    data_iter = iter(dataloader)
    data = data_iter.next()

    img, out_state = data
    plt.imshow(img[0, 0:3].reshape(600,800,3))
    plt.show()
    print(f"output_state: {out_state[0]}")
    
# test()