import pcl
import numpy as np
from pcl import pcl_visualization


cloud = pcl.PointCloud()
cloud.from_array(np.array(depth,dtype=np.float32))
viewer = pcl_visualization.PCLVisualizering()
viewer.AddPointCloud(cloud, bytes(0))
viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 5, bytes(0))
viewer.Spin()