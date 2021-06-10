import matplotlib.pyplot as plt
import numpy as np

import py_localization as loc

# helper functions
def latlon2position(lat, lon):
    r_earth = 6.371*10**6
    x =  lon * r_earth
    y = -lat * r_earth
    return x, y

def deg2rad(angle):
    return angle * 3.14159 / 180

##################################################
# Dataset Handler
##################################################
class DatasetHandler:
    def __init__(self):
        self.gnss_data = {}
        self.imu_data = []
        self.gt_data = []
        self.gt_raw = {}
    
    def set_gnss_data(self, ts, lats, lons, alts, xs, ys):
        self.gnss_data["times"] = ts
        self.gnss_data["lats"] = lats
        self.gnss_data["lons"] = lons
        self.gnss_data["xs"] = xs
        self.gnss_data["ys"] = ys
    
    def set_imu_data(self, ts, accel, comp, gyro):
        self.imu_data = []
        for t, a, c, g in zip(ts, accel, comp, gyro):
            self.imu_data.append(loc.ImuMeasurement(a, c, g, t))

    def set_gt_data(self, ts, locs, vels, rots):
        xs = []
        ys = []
        zs = []
        roll = []
        pitch = []
        yaw = []
        self.gt_data = []
        for t, pos, vel, rot in zip(ts, locs, vels, rots):
            quat = loc.euler_to_quat(rot)
            state = loc.State(pos, vel, quat.as_vector(), t, -1)
            self.gt_data.append(state)
            xs.append(pos[0])
            ys.append(pos[1])
            zs.append(pos[2])
            roll.append(rot[0])                      
            pitch.append(rot[1])
            yaw.append(rot[2])
        self.gt_raw['time'] = ts
        self.gt_raw['x'] = xs
        self.gt_raw['y'] = ys
        self.gt_raw['z'] = zs
        self.gt_raw['roll'] = roll
        self.gt_raw['pitch'] = pitch
        self.gt_raw['yaw'] = yaw
    
    def get_gnss_data(self):
        return self.gnss_data["times"], self.gnss_data["lats"], self.gnss_data["lons"], \
            self.gnss_data["xs"], self.gnss_data["ys"]

    def get_imu_data(self):
        return self.imu_data

    def get_gt_data(self):
        return self.gt_data

ds_handler = DatasetHandler()

##################################################
# Process GNSS Data
##################################################
data_file = open("GNSS_data.txt", 'r')
gnss_times = []
lats = []
lons = []
alts = []

# split data
lns = data_file.read().splitlines()
data_file.close()
for ln in lns:
    elems = ln.split(',')
    elems = [float(elem) for elem in elems]
    gnss_times.append(elems[0])
    lats.append(elems[1])
    lons.append(elems[2])
    alts.append(elems[3])

# convert from degrees
lats = [np.deg2rad(lat) for lat in lats]
lons = [np.deg2rad(lon) for lon in lons]

# convert lat lons to positions
gnsp = [latlon2position(lat, lon) for lat, lon in zip(lats, lons)]
gnsx = [x for x, y in gnsp]
gnsy = [y for x, y in gnsp]
ds_handler.set_gnss_data(gnss_times, alts, lons, alts, gnsx, gnsy)

##################################################
# Process IMU Data
##################################################
data_file = open("IMU_data.txt", 'r')
imu_times = []
accel = []
comp = []
gyro = []

# split data
lns = data_file.read().splitlines()
data_file.close()
for ln in lns:
    elems = ln.split(',')
    elems = [float(elem) for elem in elems]
    imu_times.append(elems[0])
    accel.append([elems[1]/1845, elems[2]/1845, elems[3]/1845])
    comp.append(elems[4])
    gyro.append([elems[5], elems[6], elems[7]])
ds_handler.set_imu_data(imu_times, accel, comp, gyro)

##################################################
# Process GT Location Data
##################################################
data_file = open("GT_data.txt", 'r')
gt_times = []
locs = []
vels = []
rots = []

# split data
lns = data_file.read().splitlines()
data_file.close()
for ln in lns:
    elems = ln.split(',')
    elems = [float(elem) for elem in elems]
    gt_times.append(elems[0])
    locs.append([elems[1], elems[2], elems[3]])
    vels.append([elems[4], elems[5], elems[6]])
    rots.append([deg2rad(elems[7]), deg2rad(elems[8]), deg2rad(elems[9])])
ds_handler.set_gt_data(gt_times, locs, vels, rots)





##################################################
# Visualize Data
##################################################
# xs = []
# ys = []
# zs = []
# for x,y,z in accel:
#     xs.append(x)
#     ys.append(y)
#     zs.append(z)

# print(xs)
# plt.plot(imu_times, xs, 'b', imu_times, ys, 'r')
# plt.show()

# gnss_times, lats, lons, gnss_xs, gnss_ys = ds_handler.get_gnss_data()

# plt.figure(1)
# plt.plot(gnss_times, gnss_xs, 'r', gnss_times, gnss_ys, 'b')
# plt.legend(["GNSS x", "GNSS y"])
# plt.xlabel("Time [s]")
# plt.ylabel("Position [m]")
# plt.draw()

# gt_times, gt_xs, gt_ys, gt_zs = ds_handler.get_gt_data()
# plt.figure(2)
# plt.plot(gt_times, gt_xs, 'r', gt_times, gt_ys, 'b')
# plt.legend(["GT x", "GT y",])
# plt.xlabel("time [s]")
# plt.ylabel("position [m]")
# plt.draw()
# plt.show()


# # print(gnss_data_file.read())
# # while True:
# #     ln = gnss_data_file.readline()
# #     ln.strip()
# #     if ln != "":
# #         elems = ln.split(',')
# #         # print(elems)
# #     else:
# #         break
