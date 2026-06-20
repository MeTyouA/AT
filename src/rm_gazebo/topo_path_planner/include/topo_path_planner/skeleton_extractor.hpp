/**
 * @file skeleton_extractor.hpp
 * @brief 骨架提取模块 - 从代价地图提取车道中心线
 */

#ifndef TOPO_PATH_PLANNER_SKELETON_EXTRACTOR_HPP_
#define TOPO_PATH_PLANNER_SKELETON_EXTRACTOR_HPP_

#include <Eigen/Core>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace topo_path_planner
{

/**
 * @class SkeletonExtractor
 * @brief 骨架提取器 - 使用形态学操作提取自由空间的中心线
 */
class SkeletonExtractor
{
public:
  /**
   * @brief 构造函数
   */
  SkeletonExtractor();

  ~SkeletonExtractor() = default;

  /**
   * @brief 从代价地图提取骨架
   * @param costmap 输入代价地图
   * @return 骨架点的集合
   */
  std::vector<Eigen::Vector2d> extractSkeleton(
    const nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief 获取骨架图像（可视化用）
   */
  const cv::Mat& getSkeletonImage() const { return skeleton_image_; }

  /**
   * @brief 设置形态学操作参数
   */
  void setKernelSize(int size) { kernel_size_ = size; }

  /**
   * @brief 设置是否细化骨架
   */
  void setThinning(bool enable) { enable_thinning_ = enable; }

private:
  cv::Mat skeleton_image_;    ///< 骨架图像
  int kernel_size_;           ///< 形态学核大小
  bool enable_thinning_;      ///< 是否启用细化

  /**
   * @brief 将代价地图转换为OpenCV图像
   */
  cv::Mat costmapToImage(const nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief 使用Zhang-Suen算法细化骨架
   */
  void thinning(cv::Mat& image);

  /**
   * @brief 从骨架图像提取点坐标
   */
  std::vector<Eigen::Vector2d> extractPointsFromSkeleton(
    const cv::Mat& skeleton,
    const nav2_costmap_2d::Costmap2D* costmap);

  /**
   * @brief Zhang-Suen细化算法的一次迭代
   */
  void thinningIteration(cv::Mat& image, int iter);

  /**
   * @brief 形态学骨架提取
   */
  cv::Mat morphologicalSkeleton(const cv::Mat& binary);
};

}  // namespace topo_path_planner

#endif  // TOPO_PATH_PLANNER_SKELETON_EXTRACTOR_HPP_
