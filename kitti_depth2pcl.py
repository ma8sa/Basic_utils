import numpy as np
import open3d as o3d
import pptk
import h5py
import glob
import cv2
import sys
import os



def sgbm_depth(imgL,imgR,frame_num):

    window_size = 7                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
     
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=256,             # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=5,
        P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=0,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    
    
    # FILTER Parameters
    lmbda = 80000
    sigma = 1.2
    visual_multiplier = 1.0
     
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    
    
    displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
    
    dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!

    
    #TODO change save func 
    #filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
    #print(filteredImg)
    #print(type(filteredImg))
    #input()
    #filteredImg = np.uint8(filteredImg)
    #return displ
    return filteredImg


def read_calib_file(filepath):

    data = {}

    with open(filepath, 'r') as f:
        for line in f.readlines():


            if len(line) > 3:
                key, value = line.split(':', 1)

            # The only non-float values in these files are dates, which
            # we don't care about anyway
            try:
                data[key] = np.array([float(x) for x in value.split()])
            except ValueError:
                pass

    return data


def transform_from_rot_trans(R, t):
    R = R.reshape(3, 3)
    t = t.reshape(3, 1)
    return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


def project_to_3d(point_cloud_file,calib_file):


        points = np.load(point_cloud_file)
        points= points.reshape(points.shape[-2],points.shape[-1])


        # scale

        x_scale = 1.0
        y_scale = 1.0

        #CALIB
        calib_f = read_calib_file(calib_path)

        R0 = calib_f['R0_rect']
        P0 = calib_f['P0']
        Tr = calib_f['Tr_velo_to_cam']

        #reshape
        R0 = R0.reshape(3,3)
        R_inv = np.linalg.inv(R0)
        P0 = P0.reshape(3,4)
        P0 = P0[:,:-1]
        Tr = Tr.reshape(3,4)
        #zeros = np.zeros((1,4))
        #Tr = np.vstack((Tr,zeros))
        #Tr[3,3] = 1
        Tr_inv = -Tr
        Tr_inv[:3,:3] = np.linalg.inv(Tr[:3,:3]) #inverse of rotation

        #points[:,-1] = 0
        count = 0

        #SCALING
        fx =  P0[1,1] 
        fy =  P0[0,0] 

        cx = x_scale * P0[1,2]
        cy = y_scale * P0[0,2]

        
        cx =  (P0[1,2]-156)  
        cy =  (P0[0,2]-154)  

        points_velo = []

        for i,px in enumerate(points):
            for j,p in enumerate(px):

                Z = p
                if disp == True:
                    Z = (fx*0.54)/p
                X = (Z*(i-cx))/fx
                Y = (Z*(j-cy))/fy
                
                camera_frame_3d = np.array([Y,X,Z])
                un_rect_3d = np.ones(4)
                un_rect_3d[:3] = R_inv.dot(camera_frame_3d)# orig
                #un_rect_3d[:3] = np.dot(camera_frame_3d.T,R_inv) # new

                velo_3d = Tr_inv.dot(un_rect_3d)
                #velo_3d = np.dot(un_rect_3d.T,Tr_inv)
                velo_3d = np.append(velo_3d,[1.0])

                points_velo.append(velo_3d)

                #TODO beware of the order of X,Y

                count +=1

        points_velo = np.array(points_velo,dtype=np.float32)
        return points_velo
        #points_velo = points_velo[:,:-1]

        #o3d.draw_geometries([point_cloud])

if __name__ == "__main__":


    points_file = ""
    calib_file = ""




