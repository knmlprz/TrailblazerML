import numpy as np
import vision.oak.reading_data as rd
import map2d.translator as m2d
from map2d.pcd2track import TrackMaker

if __name__ == "__main__":
    base_path = "./data1500/void_1500-47/stairs0"
    visualize = False
    S3D = rd.Simulation3D(base_path=base_path, visualize=visualize)

    res = 200
    map_2d = np.full((res, res), np.nan)
    value_range = (-1, 1)
    # visualization_type - 0 means using open3d, any other number means using regular matplotlib
    mapper = m2d.PointCloudMapper(res, value_range, visualize=True, visualization_type=1)
    track_maker = TrackMaker(threshold=0.2, has_visualization=True)

    # simulation loop
    for image_path, depth_path, pose_path in zip(S3D.images, S3D.sparse_depths, S3D.poses):
        rgb, pcd, point = S3D.simulation_reading_one_time(image_path, depth_path, pose_path)
        ### HERE WE CAN ADD SOME FUNCTIONALITY
        # TODO: ALL THE REST
        ###
        # track = track_maker.point_cloud_to_track(pcd.points)
        map_2d = mapper.point_cloud_to_2d_map(pcd,point)
