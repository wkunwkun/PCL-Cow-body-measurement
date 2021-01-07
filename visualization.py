# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
import pcl
import pcl.pcl_visualization
import os
class Point(object):
    def _init_(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
points = []
# from pcl.pcl_registration import icp, gicp, icp_nl



cloud = pcl.load_XYZRGB('niu.pcd')
visual = pcl.pcl_visualization.CloudViewing()
# PointXYZ
# visual.ShowMonochromeCloud(cloud)

# visual.ShowGrayCloud(cloud, b'cloud')
visual.ShowColorCloud(cloud, b'cloud')
# visual.ShowColorACloud(cloud, b'cloud')

# while True:
#     visual.WasStopped()
# end

flag = True
while flag:
    flag != visual.WasStopped()
end


