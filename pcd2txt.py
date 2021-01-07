import os

class Point(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
points = []
filename = 'ground'

with open(filename+'.pcd') as f:
    for line in  f.readlines()[11:len(f.readlines())-1]:
        strs = line.split(' ')
        points.append(Point(strs[0],strs[1],strs[2].strip()))
fw = open(filename+'.txt','w')
for i in range(len(points)):
     linev = points[i].x+" "+points[i].y+" "+points[i].z+"\n"
     fw.writelines(linev)
fw.close()