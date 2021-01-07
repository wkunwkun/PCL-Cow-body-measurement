# -*- coding: utf-8 -*-
#from __future__ import print_function
import pcl
import cv2
import numpy as np
import math
from pcl import pcl_visualization

#rotation
def rotation_matrix(axis,theta):
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis,axis))
    a = math.cos(theta / 2.0)
    b , c ,d = -axis *math.sin(theta / 2.0)
    aa ,bb ,cc ,dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

depth = cv2.imread("picture/6.png",-1)
rows = len(depth)
cols = len(depth[0])
pointcloud = []
ground_points = []
curve_venter = [None,None,None]
curve_back = [None,None,None]

camera_factor = 1000
camera_cx = 649.402466
camera_cy = 649.402466
camera_fx = 642.685730
camera_fy = 642.685730

for m in range(0, rows):
    for n in range(0, cols):
        d = depth[m][n]
        if d == 0:
            d = np.nan
        else:

            z = float(d)/camera_factor
            x = (n - camera_cx)  * z / camera_fx
            y = (m - camera_cy)  * z / camera_fy
            points =[x,y,z]
            pointcloud.append(points)


pointcloud = np.array(pointcloud, dtype = np.float32)
cloud = pcl.PointCloud(pointcloud)

#passthrough_filter
passthrough = cloud.make_passthrough_filter()
passthrough.set_filter_field_name ("z")
passthrough.set_filter_limits (0.41, 0.65)
cloud_filtered = passthrough.filter ()

#Plane_model_segmentation
seg = cloud_filtered.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_distance_threshold(0.01)
seg.set_normal_distance_weight(0.01)
seg.set_max_iterations(100)
indices, coefficients = seg.segment()

#segmentation
ground_cloud = cloud_filtered.extract(indices,False)
cloud = cloud_filtered.extract(indices,True)

#transform
axis = np.cross( [0, 1 ,0] ,[coefficients[0],coefficients[1],coefficients[2]])
theta = np.arccos(np.dot([coefficients[0],coefficients[1],coefficients[2]], [0, 1 ,0]))
rotx = rotation_matrix(axis,theta)
np_cloud = np.dot(np.array(cloud , dtype = np.float32) ,rotx )
#ground_plane = np.dot(rotx,[coefficients[0],coefficients[1],coefficients[2]])
cloud = pcl.PointCloud(np.array(np_cloud, dtype = np.float32))

#smoothing
#search_method = cloud.make_kdtree()
#mls = cloud.make_moving_least_squares()
#mls.set_polynomial_order(2)
#mls.set_Search_Method(search_method)
#mls.set_search_radius(0.03)
#cloud = mls.process()

#get cow body
np_cloud_x = np_cloud[:,0]
np_cloud_y = np_cloud[:,1]
# np_cloud_z = np_cloud[:,2]
x_min = np.min(np_cloud_x)
x_max = np.max(np_cloud_x)
y_min = np.min(np_cloud_y)
y_max = np.max(np_cloud_y)
# z_min = np.min(np_cloud_z)
# z_max = np.max(np_cloud_z)

x_passthrough = cloud.make_passthrough_filter()
x_passthrough.set_filter_field_name ("x")
x_passthrough.set_filter_limits ( 0.85 * x_min , 0.75 * x_max +0.25 * x_min)
cow_without_head = x_passthrough.filter ()

y_passthrough = cow_without_head.make_passthrough_filter()
y_passthrough.set_filter_field_name ("y")
y_passthrough.set_filter_limits (0.6 * y_min + 0.4 * y_max ,y_max)
cow_without_legs = y_passthrough.filter ()

#cut into slices
cow_body = np.array(cow_without_legs, dtype = np.float32)
cow_body = cow_body[np.lexsort(cow_body[:,::-1].T)]

np_cloud_x = cow_body[:,0]
x_min = np.min(np_cloud_x)
x_max = np.max(np_cloud_x)
x_unit = (x_max - x_min) / 500

for i in range(-1,500) :
  x_silce =cow_body[ np.where((np_cloud_x > (x_min + i * x_unit) ) & (np_cloud_x < ( x_min + (i + 1) * x_unit)))]
  if len(x_silce) == 0:
      x_silce = [None,None,None]
  else:
      z_max =x_silce[np.argmax( x_silce[:,2] )]
      z_min = x_silce[np.argmin(x_silce[:, 2])]
      curve_venter = np.vstack( (curve_venter ,z_max) )
      curve_back = np.vstack((curve_back, z_min))

cow_venter = np.vstack((curve_venter , curve_back))
venter_point = pcl.PointCloud(np.array(cow_venter, dtype = np.float32))
#cow_back = pcl.PointCloud(np.array(curve_back, dtype = np.float32))


#visit
#pcl.save_PointNormal(ground_cloud,"ground.pcd",format = 'pcd')
viewer = pcl_visualization.PCLVisualizering()
viewer.SetBackgroundColor(0,0,0)
#viewer.AddPointCloud_ColorHandler()
#viewer.AddPointCloudNormals(ground_cloud, normals)
viewer.AddPointCloud(venter_point, bytes(0))
viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 5, bytes(0))
viewer.AddCoordinateSystem(0.1)
viewer.Spin()
