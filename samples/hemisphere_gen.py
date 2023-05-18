import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

def theta_phi_to_xyz(theta, phi, radius):
    """
    convert theta, phi to xyz
    """
    x = radius * np.sin(theta) * np.cos(phi)
    y = radius * np.sin(theta) * np.sin(phi)
    z = radius * np.cos(theta)

    return np.array([x, y, z])

def sphere_gen(radius: float,
               num_points: int, 
               num_steps: int,
               origin: np.ndarray = np.array([0, 0, 0], dtype=np.float32) 
               ) -> list:
    """
    generate hemisphere point cloud
    """
    # step 1. set points num in each height as n_view
    thetas = [(np.pi/2) * (i / num_steps) for i in range(num_steps)]
    radiuses = np.sin(thetas)
    n_view = np.round(num_points * radiuses / np.sum(radiuses)).astype(int)

    # step 2. generate points
    points = []
    rmats = []

    for n_v, theta in zip(n_view, thetas):
        for i in range(n_v):
            phi = 2 * np.pi * i / n_v
            pts = theta_phi_to_xyz(theta, phi, radius) + origin

            # calculate orientation
            u_z = -1 * theta_phi_to_xyz(theta, phi, 1)
            u_y = theta_phi_to_xyz(theta + np.pi/2, phi, 1)
            u_x = np.cross(u_y, u_z)

            rmat = np.linalg.inv(np.array([u_x, u_y, u_z]))

            points.append(pts)
            rmats.append(rmat)

    transforms = []

    # points and oris to transform matrix
    for point, rmat in zip(points, rmats):
        transform = np.eye(4)
        transform[:3, 3] = point
        transform[:3, :3] = rmat
        transforms.append(transform)

    return transforms

# Open3D를 사용하여 3D 점군을 시각화합니다.
def visualize_cloud(transforms):
    meshes = []

    for tf in transforms:
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        meshes.append(mesh.transform(tf).scale(0.1, [0, 0, 0]))
    
    o3d.visualization.draw_geometries(meshes)


if __name__ == "__main__":
    points = sphere_gen(20, 100, 5)

    visualize_cloud(points)