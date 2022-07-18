import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import open3d as o3d

img = "./left_image.png"
pcd = o3d.io.read_point_cloud("./test_pcd.pcd")


# P2 (3 x 4) for left eye
P2 = np.matrix([[1624.7336487407558, 0.0, 640.5, -0.0], [0.0, 1624.7336487407558, 480.5, 0.0], [0.0, 0.0, 1.0, 0.0]]).reshape(3,4)
Tr_velo_to_cam = np.matrix([[0, -1, 0, -0.2], [0.0, 0, -1, -0.4], [1, 0.0, 0.0, -0.3]]).reshape(3,4)

Tr_velo_to_cam = np.insert(Tr_velo_to_cam,3,values=[0,0,0,1],axis=0)

points = np.asarray(pcd.points) 

# TODO: use fov filter? 
velo = np.insert(points,3,1,axis=1).T
velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)
cam = P2 * Tr_velo_to_cam * velo

cam = np.delete(cam,np.where(cam[2,:]<0)[1],axis=1)
print(cam.size)
# get u,v,z
cam[:2] /= cam[2,:]
# do projection staff
plt.figure(figsize=(12,5),dpi=96,tight_layout=True)
png = mpimg.imread(img)
IMG_H,IMG_W,_ = png.shape
# restrict canvas in range
plt.axis([0,IMG_W,IMG_H,0])
plt.imshow(png)
# filter point out of canvas
u,v,z = cam
u_out = np.logical_or(u<0, u>IMG_W)
v_out = np.logical_or(v<0, v>IMG_H)
outlier = np.logical_or(u_out, v_out)
cam = np.delete(cam,np.where(outlier),axis=1)
# generate color map from depth
u,v,z = cam
plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2)
# plt.title(name)
# plt.savefig(f'./data_object_image_2/testing/projection/{name}.png',bbox_inches='tight')
plt.show()