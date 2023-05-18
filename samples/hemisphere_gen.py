import numpy as np
import open3d as o3d

def sphere_gen(radius: float,
               num_points: int, 
               num_steps: int,
               origin: np.ndarray = np.array([0, 0, 0], dtype=np.float32) 
               ) -> list:
    """
    generate hemisphere point cloud
    """
    # step 1. set points num in each height
    radius_in_each_step = [np.sin((np.pi / 2) * ( i / num_steps)) for i in range(num_steps)]
    n_view_itr = num_points * np.array(radius_in_each_step) / np.sum(radius_in_each_step)
    n_view_in_each_steps = np.round(n_view_itr).astype(int)

    # step 2. generate points
    points = []

    for num, theta in zip(n_view_in_each_steps, radius_in_each_step):
        for i in range(num):
            phi = 2 * np.pi * i / num
            x = radius * np.sin(theta) * np.cos(phi) + origin[0]
            y = radius * np.sin(theta) * np.sin(phi) + origin[1]
            z = radius * np.cos(theta) + origin[2]

            points.append([x, y, z])

    return points

# Open3D를 사용하여 3D 점군을 시각화합니다.
def visualize_cloud(points):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    o3d.visualization.draw_geometries([cloud])


if __name__ == "__main__":
    points = sphere_gen(1.5, 100, 5)

    visualize_cloud(points)