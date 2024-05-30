import vision.QAK.reading_data as rd

if __name__ == "__main__":
    base_path = "../data1500/void_1500-47/stairs0"
    visualize = True
    S3D = rd.Simulation3D(base_path=base_path, visualize=visualize)

    # simulation loop
    for image_path, depth_path, pose_path in zip(S3D.images, S3D.sparse_depths, S3D.poses):
        rgb, pcd, point = S3D.simulation_reading_one_time(image_path, depth_path, pose_path)
        ### HERE WE CAN ADD SOME FUNCTIONALITY
        # TODO: ALL THE REST
        ###
