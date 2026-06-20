/**
 * @file skeleton_extractor.cpp
 * @brief 骨架提取模块实现
 */

#include "topo_path_planner/skeleton_extractor.hpp"
#include <iostream>

namespace topo_path_planner
{

SkeletonExtractor::SkeletonExtractor()
  : kernel_size_(3),
    enable_thinning_(true)
{
}

std::vector<Eigen::Vector2d> SkeletonExtractor::extractSkeleton(
  const nav2_costmap_2d::Costmap2D* costmap)
{
  if (!costmap) {
    return {};
  }

  // 1. 将代价地图转换为二值图像
  cv::Mat binary = costmapToImage(costmap);

  // 2. 形态学骨架提取
  cv::Mat skeleton = morphologicalSkeleton(binary);

  // 3. 可选：进一步细化
  if (enable_thinning_) {
    thinning(skeleton);
  }

  skeleton_image_ = skeleton.clone();

  // 4. 从骨架图像提取点坐标
  return extractPointsFromSkeleton(skeleton, costmap);
}

cv::Mat SkeletonExtractor::costmapToImage(
  const nav2_costmap_2d::Costmap2D* costmap)
{
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();

  cv::Mat image(size_y, size_x, CV_8UC1);

  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      unsigned char cost = costmap->getCost(x, size_y - 1 - y);  // 翻转y轴以匹配图像坐标
      // 自由空间设为255（白色），障碍物设为0（黑色）
      if (cost < 253) {  // INSCRIBED_INFLATED_OBSTACLE = 253
        image.at<uint8_t>(y, x) = 255;
      } else {
        image.at<uint8_t>(y, x) = 0;
      }
    }
  }

  return image;
}

cv::Mat SkeletonExtractor::morphologicalSkeleton(const cv::Mat& binary)
{
  cv::Mat skeleton(binary.size(), CV_8UC1, cv::Scalar(0));
  cv::Mat temp;
  cv::Mat eroded;
  cv::Mat element = cv::getStructuringElement(
    cv::MORPH_CROSS,
    cv::Size(kernel_size_, kernel_size_));

  cv::Mat opened;
  cv::morphologyEx(binary, opened, cv::MORPH_OPEN, element);

  bool done;
  do {
    cv::erode(opened, eroded, element);
    cv::dilate(eroded, temp, element);
    cv::subtract(opened, temp, temp);
    cv::bitwise_or(skeleton, temp, skeleton);
    eroded.copyTo(opened);

    done = (cv::countNonZero(opened) == 0);
  } while (!done);

  return skeleton;
}

void SkeletonExtractor::thinning(cv::Mat& image)
{
  // 使用Zhang-Suen细化算法
  cv::Mat prev = image.clone();
  cv::Mat diff;

  do {
    thinningIteration(image, 0);
    thinningIteration(image, 1);
    cv::absdiff(image, prev, diff);
    image.copyTo(prev);
  } while (cv::countNonZero(diff) > 0);
}

void SkeletonExtractor::thinningIteration(cv::Mat& image, int iter)
{
  cv::Mat marker = cv::Mat::zeros(image.size(), CV_8UC1);

  for (int y = 1; y < image.rows - 1; ++y) {
    for (int x = 1; x < image.cols - 1; ++x) {
      if (image.at<uint8_t>(y, x) == 0) continue;

      uchar p2 = image.at<uint8_t>(y - 1, x);
      uchar p3 = image.at<uint8_t>(y - 1, x + 1);
      uchar p4 = image.at<uint8_t>(y, x + 1);
      uchar p5 = image.at<uint8_t>(y + 1, x + 1);
      uchar p6 = image.at<uint8_t>(y + 1, x);
      uchar p7 = image.at<uint8_t>(y + 1, x - 1);
      uchar p8 = image.at<uint8_t>(y, x - 1);
      uchar p9 = image.at<uint8_t>(y - 1, x - 1);

      int A = (p2 == 0 && p3 == 255) + (p3 == 0 && p4 == 255) +
              (p4 == 0 && p5 == 255) + (p5 == 0 && p6 == 255) +
              (p6 == 0 && p7 == 255) + (p7 == 0 && p8 == 255) +
              (p8 == 0 && p9 == 255) + (p9 == 0 && p2 == 255);

      int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;

      int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
      int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

      if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0) {
        marker.at<uint8_t>(y, x) = 255;
      }
    }
  }

  image &= ~marker;
}

std::vector<Eigen::Vector2d> SkeletonExtractor::extractPointsFromSkeleton(
  const cv::Mat& skeleton,
  const nav2_costmap_2d::Costmap2D* costmap)
{
  std::vector<Eigen::Vector2d> points;

  for (int y = 0; y < skeleton.rows; ++y) {
    for (int x = 0; x < skeleton.cols; ++x) {
      if (skeleton.at<uint8_t>(y, x) > 0) {
        double wx, wy;
        unsigned int map_y = skeleton.rows - 1 - y;
        costmap->mapToWorld(x, map_y, wx, wy);
        points.emplace_back(wx, wy);
      }
    }
  }

  return points;
}

}  // namespace topo_path_planner
