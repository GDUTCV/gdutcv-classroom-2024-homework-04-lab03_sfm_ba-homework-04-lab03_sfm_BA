import os
import numpy as np
from preprocess import RESULT_DIR
import open3d as o3d


def visualize_point_cloud(pts: np.ndarray):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    o3d.visualization.draw([pcd])

def save_point_cloud(pts: np.ndarray, output_file: str):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    o3d.io.write_point_cloud(output_file, pcd)


# def main():
#     points3d_save_file = os.path.join(RESULT_DIR, 'points3d.npy')
#     points3d = np.load(points3d_save_file)
#     visualize_point_cloud(pts=points3d)

def main():
    points3d_save_file = os.path.join(RESULT_DIR, 'points3d.npy')
    points3d = np.load(points3d_save_file)

    # 指定输出文件路径
    output_file = os.path.join(RESULT_DIR, 'points3d_output.ply')
    save_point_cloud(pts=points3d, output_file=output_file)
    print(f"Point cloud saved to {output_file}")


if __name__ == '__main__':
    main()
