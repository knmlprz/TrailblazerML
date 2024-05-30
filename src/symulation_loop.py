import vision.QAK.reading_data as rd

if __name__ == "__main__":
    base_path = "../data1500/void_1500-47/stairs0"
    visualize = True

    read = rd.Param_reding_3D(base_path = base_path, visualize = visualize)

    for image_path, depth_path, pose_path in zip(read.images, read.sparse_depths, read.poses):

        rgb, pcd, point = read.symulation_reading_one_time(image_path,depth_path, pose_path)
        ### HERE WE CAN ADD SOME FUNCTIONALITY
        # TODO: ALL THE REST
        ###