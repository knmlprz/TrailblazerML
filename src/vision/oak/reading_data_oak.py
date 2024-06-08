import open3d as o3d
from camera_oak import CameraOAK




if __name__ == "__main__":
    config = {
        'rgb_camera': {
            'resolution': 'THE_1080_P',
            'socket': 'CAM_A',
            'isp_scale': (1, 3),
            'fps': 5
        },
        'mono_cameras': {
            'resolution': 'THE_400_P',
            'fps': 5
        },
        'depth': {
            'preset': 'HIGH_DENSITY',
            'median_filter': 'KERNEL_7x7',
            'left_right_check': True,
            'extended_disparity': False,
            'subpixel': True,
            'align_to': 'CAM_A'
        },
        'imu': {
            'ACCELEROMETER_RAW': 500,
            'GYROSCOPE_RAW': 400,
            'ROTATION_VECTOR': 400,
            'batch_threshold': 1,
            'max_batch_reports': 10
        }
    }
    camera = CameraOAK(config)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    points = []
    line_set = o3d.geometry.LineSet()
    vis.add_geometry(line_set)
    while True:
        rgb, pcd, pose = camera.get_data()

        # plot = o3d.geometry.Image(rgb)
        # vis.add_geometry(plot)
        # vis.poll_events()
        # vis.update_renderer()


        points.append(pose[:3, 3])
        pcd.transform(pose)
        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        line_set.points = o3d.utility.Vector3dVector(points)
        if len(points) > 1:
            lines = [[j, j + 1] for j in range(len(points) - 1)]
            line_set.lines = o3d.utility.Vector2iVector(lines)
            colors = [[1, 0, 0] for _ in range(len(lines))]
            line_set.colors = o3d.utility.Vector3dVector(colors)
        vis.update_geometry(line_set)
        vis.poll_events()
        vis.update_renderer()
        if len(points) > 30:
            break
    vis.run()
    vis.destroy_window()

