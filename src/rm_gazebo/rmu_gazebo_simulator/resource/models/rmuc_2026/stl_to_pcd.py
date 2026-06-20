#!/usr/bin/env python3
import trimesh
import open3d as o3d
import argparse
import os

def convert_stl_to_pcd(input_file, output_file, num_samples, voxel_size=None):
    if not os.path.exists(input_file):
        print(f"错误: 找不到输入文件 '{input_file}'")
        return

    print(f"正在加载模型: {input_file} ...")
    mesh = trimesh.load(input_file)

    # 1. 采样点云
    print(f"正在从表面采样 {num_samples} 个点...")
    points = mesh.sample(num_samples)

    # 2. 转换为 Open3D 格式
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 3. 核心优化：体素下采样 (Voxel Downsampling)
    # 这能保证点云密度均匀，去除平面上冗余的点，保留特征
    if voxel_size and voxel_size > 0:
        print(f"正在进行体素下采样，间距: {voxel_size}m ...")
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"下采样后点数: {len(pcd.points)}")

    # 4. 核心优化：强制二进制格式保存
    print(f"正在保存点云至: {output_file} ...")
    # write_ascii=False 确保保存为二进制，加载速度提升 10 倍以上
    # compressed=True 可以进一步减小体积
    o3d.io.write_point_cloud(output_file, pcd, write_ascii=False, compressed=True)
    
    file_size = os.path.getsize(output_file) / (1024 * 1024)
    print(f"转换完成！文件大小: {file_size:.2f} MB")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="针对 RoboMaster 优化的 STL 转 PCD 工具")
    parser.add_argument("input", help="输入的 STL 文件路径")
    parser.add_argument("output", help="输出的 PCD 文件路径")
    parser.add_argument("-n", "--samples", type=int, default=6000000, help="初始采样点数量")
    parser.add_argument("-v", "--voxel", type=float, default=0.01, help="体素大小(米)，建议 0.01-0.05")

    args = parser.parse_args()
    convert_stl_to_pcd(args.input, args.output, args.samples, args.voxel)
