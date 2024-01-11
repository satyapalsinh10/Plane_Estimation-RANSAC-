import open3d as o3d
import numpy as np

def fit_plane_ransac(point_cloud, max_iterations=820, distance_threshold=0.0115):


    points = np.asarray(point_cloud.points)

    best_inliers = []
    best_plane = None

    for iteration in range(max_iterations):
        random_indices = np.random.choice(len(points), 3, replace=False)
        random_points = points[random_indices, :]
        v1 = random_points[1] - random_points[0]
        v2 = random_points[2] - random_points[0]
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal)
        d = -np.dot(normal, random_points[0])
        distances = np.abs(np.dot(points, normal) + d) / np.linalg.norm(normal)
        inliers = np.where(distances < distance_threshold)[0]
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = (normal[0], normal[1], normal[2], d)

    return best_plane


pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

colored_pcd = o3d.geometry.PointCloud()
colored_pcd.points = pcd.points  
colored_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors))  

# Fit a plane using RANSAC
best_plane = fit_plane_ransac(pcd)
distance_threshold = 0.029
normal = np.array([best_plane[0], best_plane[1], best_plane[2]])
d = best_plane[3]
distances = np.abs(np.dot(np.asarray(pcd.points), normal) + d) / np.linalg.norm(normal)
best_inliers = np.where(distances < distance_threshold)[0]

# Color the inlier points in red in the copied point cloud
inlier_colors = np.asarray(colored_pcd.colors)
inlier_colors[best_inliers, :] = [1, 0, 0]
colored_pcd.colors = o3d.utility.Vector3dVector(inlier_colors)

# Visualize the original point cloud with the best-fit plane highlighted
o3d.visualization.draw_geometries([pcd, colored_pcd],
                                  zoom=1,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])