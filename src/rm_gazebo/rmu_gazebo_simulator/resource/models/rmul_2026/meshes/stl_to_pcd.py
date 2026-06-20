#!/usr/bin/env python3
import trimesh
import open3d as o3d
import argparse
import os

def convert_stl_to_pcd(input_file, output_file, num_samples):
    # 1. 检查输入文件是否存在
    if not os.path.exists(input_file):
        print(f"错误: 找不到输入文件 '{input_file}'")
        return

    print(f"正在加载模型: {input_file} ...")
    mesh = trimesh.load(input_file)

    # 2. 采样点云
    print(f"正在从表面采样 {num_samples} 个点...")
    points = mesh.sample(num_samples)

    # 3. 转换为 Open3D 格式
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 4. 保存文件
    print(f"正在保存点云至: {output_file} ...")
    o3d.io.write_point_cloud(output_file, pcd)
    print("转换完成！")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="将 STL 模型转换为 PCD 点云格式")
    
    # 定义命令行参数
    parser.add_argument("input", help="输入的 STL 文件路径")
    parser.add_argument("output", help="输出的 PCD 文件路径")
    parser.add_argument("-n", "--samples", type=int, default=100000, help="采样点数量 (默认: 10000)")

    args = parser.parse_args()

    convert_stl_to_pcd(args.input, args.output, args.samples)
