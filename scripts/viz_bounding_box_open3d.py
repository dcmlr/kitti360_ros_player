import cv2 
import open3d as o3d
import numpy as np

# fs =  cv2.FileStorage("./train/2013_05_28_drive_0000_sync.xml", cv2.FILE_STORAGE_READ)
fs =  cv2.FileStorage("./train_full/2013_05_28_drive_0000_sync.xml", cv2.FILE_STORAGE_READ)

fn = fs.getNode("object111")
vert = fn.getNode("vertices").mat()
fac = fn.getNode("faces").mat()
transform = fn.getNode("transform").mat()

mesh = o3d.geometry.TriangleMesh()
transformed = np.array(transformed)
mesh.vertices = o3d.utility.Vector3dVector(transformed)

mesh.triangles = o3d.utility.Vector3iVector(fac)

mesh = mesh.transform(transform)
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True, mesh_show_wireframe=True)
