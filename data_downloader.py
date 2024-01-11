import open3d as o3d

if __name__ == "__main__":
    # Set the data root directory (optional)
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

    # Download and read the PCDPointCloud dataset
    dataset = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(dataset.path)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd], zoom=1, front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

    # Optionally, you can save the downloaded point cloud to a file
    o3d.io.write_point_cloud("downloaded_point_cloud.pcd", pcd)

