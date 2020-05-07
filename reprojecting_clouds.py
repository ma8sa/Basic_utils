import os
import cv2
import h5py
import numpy as np


def morphed_points(points,R,T):
    print(R)

    new_points = np.dot(points,R) 
    new_points = new_points.T + T
    return new_points.T


def load_pcd_array(pcd_path, fmt='xyz'):
    if pcd_path.endswith('hdf5'):
        fmt = 'h5'
    if fmt == 'xyz':
        cloud = pcl.load(pcd_path)
        pcd = np.zeros((cloud.size, 3), dtype=np.float32)
        for i in range(cloud.size):
            pcd[i] = cloud[i]
    elif fmt == 'xyzi':
        cloud = pcl.load_XYZI(pcd_path)
        pcd = np.zeros((cloud.size, 4), dtype=np.float32)
        for i in range(cloud.size):
            pcd[i] = cloud[i]
    elif fmt == 'xyzrgb':
        cloud = pcl.load_XYZRGB(pcd_path)
        pcd = np.zeros((cloud.size, 6), dtype=np.float32)
        for i in range(cloud.size):
            pcd[i] = cloud[i]
    elif fmt == 'h5':
        f = h5py.File(pcd_path,'r')

        f_x = np.array(f['X'][:])
        f_y = np.array(f['Y'][:])
        f_z = np.array(f['Z'][:])
        
        pcd = np.vstack((f_x,f_y))
        pcd = np.vstack((pcd,f_z))
        pcd = pcd.T
        
        #NOTE why are there multiple point clouds
    print("shape of pcd being used {} ".format(pcd.shape))
    
    return pcd



if __name__ == "__main__":

    out_dir = "../../exp_lidarlidar/lidar_sl1_lidar_sr1/results/"

    root = "/home/ironmonger/camera_calib/WP5_SensorCalibration/Software/py36/cl_calib/Data/input/Prescan/Lidar"
    
    pcd_dir_1 = root + "/03/"
    pcd_dir_2 = root + "/09/"

    calib_filename = os.path.join(out_dir, 'lidar_lidar_calibration_points_points.txt')
    RT_filename = os.path.join(out_dir, 'lidar_lidar_calibration_RT_points_points.xml')

    RT = cv2.FileStorage(RT_filename, cv2.FILE_STORAGE_READ)

    R = RT.getNode("R")
    T = RT.getNode("T")

    R = np.array(R.mat())
    T = np.array(T.mat())


    pcd_1 = load_pcd_array(pcd_dir_1 + "10.hdf5")
    pcd_2 = load_pcd_array(pcd_dir_2 + "10.hdf5")

    pcd_1_new = morphed_points(pcd_1,R,T)
    pcd_2_new = morphed_points(pcd_2,R,T)
    
    print(" pcd shape {}, pcd new shape {}".format(pcd_1.shape,pcd_1_new.shape))

    print(pcd_1_new.shape)
    pcd_1_new.tofile("pcd_1_new.bin")
    pcd_2_new.tofile("pcd_2_new.bin")

    pcd_1.tofile("pcd_1.bin")
    pcd_2.tofile("pcd_2.bin")
    print(pcd_1_new.shape)

    test_ = np.fromfile(
        "pcd_1_new.bin", dtype=np.float64, count=-1).reshape([-1, 3])
    print("test shape")
    print(test_.shape)

    print(pcd_1[0])
    print(pcd_1_new[0])
    input()

    #print(pcd_2)
