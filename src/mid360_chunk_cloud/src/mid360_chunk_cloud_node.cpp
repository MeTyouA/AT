#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

class Mid360ChunkCloudNode : public rclcpp::Node
{
public:
  Mid360ChunkCloudNode()
  : Node("mid360_chunk_cloud_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mid360_chunked_cloud");
    chunk_count_ = declare_parameter<int>("chunk_count", 5);
    input_cloud_hz_ = declare_parameter<double>("input_cloud_hz", 10.0);
    timestamp_field_name_ = declare_parameter<std::string>("timestamp_field_name", "timestamp");
    timestamp_is_absolute_ = declare_parameter<bool>("timestamp_is_absolute", true);
    fallback_to_index_split_ = declare_parameter<bool>("fallback_to_index_split", true);
    publish_empty_chunk_ = declare_parameter<bool>("publish_empty_chunk", false);
    output_xyzi_only_ = declare_parameter<bool>("output_xyzi_only", true);
    zero_intensity_ = declare_parameter<bool>("zero_intensity", true);
    qos_depth_ = declare_parameter<int>("qos_depth", 5);
    drop_old_chunks_when_new_cloud_arrives_ =
      declare_parameter<bool>("drop_old_chunks_when_new_cloud_arrives", true);
    debug_ = declare_parameter<bool>("debug", true);

    if (chunk_count_ < 1) {
      RCLCPP_WARN(get_logger(), "chunk_count < 1, force set to 1");
      chunk_count_ = 1;
    }
    if (input_cloud_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "input_cloud_hz <= 0, force set to 10.0");
      input_cloud_hz_ = 10.0;
    }
    if (qos_depth_ < 1) {
      RCLCPP_WARN(get_logger(), "qos_depth < 1, force set to 1");
      qos_depth_ = 1;
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(static_cast<size_t>(qos_depth_));

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&Mid360ChunkCloudNode::cloudCallback, this, std::placeholders::_1));

    const double publish_interval_sec = 1.0 / (input_cloud_hz_ * static_cast<double>(chunk_count_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_interval_sec)),
      std::bind(&Mid360ChunkCloudNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "mid360_chunk_cloud_node started");
    RCLCPP_INFO(get_logger(), "input_topic: %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "chunk_count: %d", chunk_count_);
    RCLCPP_INFO(get_logger(), "input_cloud_hz: %.3f", input_cloud_hz_);
    RCLCPP_INFO(
      get_logger(), "estimated output frequency: %.3f Hz",
      input_cloud_hz_ * static_cast<double>(chunk_count_));
    RCLCPP_INFO(get_logger(), "timestamp_field_name: %s", timestamp_field_name_.c_str());
    RCLCPP_INFO(
      get_logger(), "timestamp_is_absolute: %s", timestamp_is_absolute_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "fallback_to_index_split: %s", fallback_to_index_split_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "output_xyzi_only: %s", output_xyzi_only_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "zero_intensity: %s", zero_intensity_ ? "true" : "false");
  }

private:
  struct SplitResult
  {
    std::vector<sensor_msgs::msg::PointCloud2> chunks;
    bool used_timestamp_split{false};
    bool used_fallback{false};
    double duration{0.0};
    size_t total_points{0};
  };

  struct CopyLayout
  {
    bool output_xyzi_only{false};
    bool input_xyz_contiguous{false};
    uint32_t output_point_step{0};
    int input_x_offset{-1};
    int input_y_offset{-1};
    int input_z_offset{-1};
    int input_intensity_offset{-1};
    std::vector<sensor_msgs::msg::PointField> output_fields;
  };

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (drop_old_chunks_when_new_cloud_arrives_ && pending_index_ < pending_chunks_.size()) {
      pending_chunks_.clear();
      pending_index_ = 0;
    }

    SplitResult result = splitCloud(*cloud_msg);
    pending_chunks_ = std::move(result.chunks);
    pending_index_ = 0;

    if (debug_) {
      const size_t avg_chunk_points = chunk_count_ > 0 ?
        result.total_points / static_cast<size_t>(chunk_count_) : 0;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "points=%zu, timestamp_duration=%.9f s, chunk_count=%d, avg_chunk_points=%zu, "
        "timestamp_split=%s, fallback=%s, output_xyzi_only=%s",
        result.total_points, result.duration, chunk_count_, avg_chunk_points,
        result.used_timestamp_split ? "true" : "false",
        result.used_fallback ? "true" : "false",
        output_xyzi_only_ ? "true" : "false");
    }
  }

  void timerCallback()
  {
    while (pending_index_ < pending_chunks_.size()) {
      auto & chunk = pending_chunks_[pending_index_];
      ++pending_index_;

      if (!publish_empty_chunk_ && chunk.width == 0) {
        continue;
      }

      auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>(std::move(chunk));
      publisher_->publish(std::move(msg));
      break;
    }

    if (pending_index_ >= pending_chunks_.size()) {
      pending_chunks_.clear();
      pending_index_ = 0;
    }
  }

  SplitResult splitCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    SplitResult result;
    result.total_points = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);

    if (result.total_points == 0 || cloud.point_step == 0) {
      result.chunks = makeEmptyChunks(cloud, cloud.header.stamp, 0.0);
      return result;
    }

    const size_t required_data_size =
      result.total_points * static_cast<size_t>(cloud.point_step);
    if (cloud.data.size() < required_data_size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 data is smaller than width * height * point_step, drop this cloud");
      return result;
    }

    const int timestamp_offset = findFieldOffset(
      cloud, timestamp_field_name_, sensor_msgs::msg::PointField::FLOAT64, true);
    if (timestamp_offset < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 has no supported FLOAT64 timestamp field named '%s'",
        timestamp_field_name_.c_str());
      return splitByIndex(cloud, true, 0.0);
    }

    if (static_cast<size_t>(timestamp_offset) + sizeof(double) >
      static_cast<size_t>(cloud.point_step))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "timestamp field offset exceeds point_step, fallback_to_index_split=%s",
        fallback_to_index_split_ ? "true" : "false");
      return splitByIndex(cloud, true, 0.0);
    }

    double min_ts = std::numeric_limits<double>::infinity();
    double max_ts = -std::numeric_limits<double>::infinity();
    bool timestamp_valid = true;

    for (size_t point_index = 0; point_index < result.total_points; ++point_index) {
      double timestamp = 0.0;
      if (!readPointTimestamp(cloud, timestamp_offset, point_index, timestamp) ||
        !std::isfinite(timestamp))
      {
        timestamp_valid = false;
        break;
      }

      min_ts = std::min(min_ts, timestamp);
      max_ts = std::max(max_ts, timestamp);
    }

    const double duration = max_ts - min_ts;
    result.duration = duration;

    if (!timestamp_valid || duration <= 0.0 || !std::isfinite(duration)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "timestamp invalid or duration abnormal, duration=%.9f, fallback_to_index_split=%s",
        duration, fallback_to_index_split_ ? "true" : "false");
      return splitByIndex(cloud, true, duration);
    }

    result.used_timestamp_split = true;
    result.used_fallback = false;
    result.chunks = buildChunksByTimestamp(cloud, timestamp_offset, min_ts, duration);
    return result;
  }

  bool readPointTimestamp(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const int timestamp_offset,
    const size_t point_index,
    double & timestamp) const
  {
    // PointCloud2 不是 std::vector<PointXYZ> 这类普通点云容器，而是一整块连续二进制
    // data。每个点在 data 中占用的字节数由 point_step 给出，不同驱动可以在一个点内部
    // 放入 x/y/z/intensity/tag/line/timestamp 等不同字段。
    //
    // fields 描述“单个点内部”的字段布局：field.offset 是该字段相对于当前点起始地址
    // 的字节偏移。第 point_index 个点的起始地址是：
    //   cloud.data.data() + point_index * cloud.point_step
    // 所以 timestamp 字段地址必须再加上 timestamp_field.offset。
    //
    // 这里不能把地址直接 reinterpret_cast 成 double* 再解引用，因为 PointCloud2 的二进制
    // buffer 不保证 double 按 CPU 要求对齐。使用 memcpy 读取 8 字节可以避免未对齐访问问题。
    const size_t point_offset = point_index * static_cast<size_t>(cloud.point_step);
    const size_t timestamp_address = point_offset + static_cast<size_t>(timestamp_offset);
    std::memcpy(&timestamp, cloud.data.data() + timestamp_address, sizeof(double));
    return true;
  }

  std::vector<sensor_msgs::msg::PointCloud2> buildChunksByTimestamp(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const int timestamp_offset,
    const double min_ts,
    const double duration)
  {
    const size_t total_points = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    const double chunk_duration = duration / static_cast<double>(chunk_count_);
    const double inv_chunk_duration = static_cast<double>(chunk_count_) / duration;

    std::vector<uint32_t> point_chunk_ids(total_points, 0);
    std::vector<size_t> chunk_point_counts(static_cast<size_t>(chunk_count_), 0);

    for (size_t point_index = 0; point_index < total_points; ++point_index) {
      double timestamp = 0.0;
      readPointTimestamp(cloud, timestamp_offset, point_index, timestamp);

      int chunk_id = static_cast<int>((timestamp - min_ts) * inv_chunk_duration);
      if (chunk_id < 0) {
        chunk_id = 0;
      }
      if (chunk_id >= chunk_count_) {
        chunk_id = chunk_count_ - 1;
      }

      point_chunk_ids[point_index] = static_cast<uint32_t>(chunk_id);
      ++chunk_point_counts[static_cast<size_t>(chunk_id)];
    }

    CopyLayout layout = makeCopyLayout(cloud);
    std::vector<builtin_interfaces::msg::Time> stamps;
    stamps.reserve(static_cast<size_t>(chunk_count_));
    for (int i = 0; i < chunk_count_; ++i) {
      const double chunk_offset = static_cast<double>(i) * chunk_duration;
      if (timestamp_is_absolute_) {
        stamps.push_back(makeStampFromNanoseconds(
          static_cast<int64_t>((min_ts + chunk_offset) * 1e9)));
      } else {
        stamps.push_back(addSeconds(cloud.header.stamp, chunk_offset));
      }
    }

    std::vector<sensor_msgs::msg::PointCloud2> chunks =
      makeChunksWithPointCounts(cloud, layout, stamps, chunk_point_counts);

    std::vector<size_t> write_point_offsets(static_cast<size_t>(chunk_count_), 0);
    for (size_t point_index = 0; point_index < total_points; ++point_index) {
      const size_t chunk_id = static_cast<size_t>(point_chunk_ids[point_index]);
      const size_t write_point_index = write_point_offsets[chunk_id]++;
      uint8_t * dst = chunks[chunk_id].data.data() +
        write_point_index * static_cast<size_t>(layout.output_point_step);
      copyPointToOutput(cloud, point_index, layout, dst);
    }

    return chunks;
  }

  SplitResult splitByIndex(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const bool fallback_requested,
    const double observed_duration)
  {
    SplitResult result;
    result.total_points = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    result.duration = observed_duration;
    result.used_timestamp_split = false;
    result.used_fallback = fallback_requested && fallback_to_index_split_;

    if (!fallback_to_index_split_) {
      result.chunks.clear();
      return result;
    }

    const size_t total_points = result.total_points;
    const double frame_duration = 1.0 / input_cloud_hz_;
    const double chunk_duration = frame_duration / static_cast<double>(chunk_count_);

    std::vector<size_t> chunk_point_counts(static_cast<size_t>(chunk_count_), 0);
    std::vector<size_t> chunk_begin_indices(static_cast<size_t>(chunk_count_), 0);
    for (int i = 0; i < chunk_count_; ++i) {
      const size_t begin = total_points * static_cast<size_t>(i) / static_cast<size_t>(chunk_count_);
      const size_t end =
        total_points * static_cast<size_t>(i + 1) / static_cast<size_t>(chunk_count_);
      chunk_begin_indices[static_cast<size_t>(i)] = begin;
      chunk_point_counts[static_cast<size_t>(i)] = end - begin;
    }

    std::vector<builtin_interfaces::msg::Time> stamps;
    stamps.reserve(static_cast<size_t>(chunk_count_));
    for (int i = 0; i < chunk_count_; ++i) {
      stamps.push_back(addSeconds(cloud.header.stamp, static_cast<double>(i) * chunk_duration));
    }

    CopyLayout layout = makeCopyLayout(cloud);
    result.chunks = makeChunksWithPointCounts(cloud, layout, stamps, chunk_point_counts);

    for (int i = 0; i < chunk_count_; ++i) {
      sensor_msgs::msg::PointCloud2 & chunk = result.chunks[static_cast<size_t>(i)];
      const size_t begin = chunk_begin_indices[static_cast<size_t>(i)];
      const size_t count = chunk_point_counts[static_cast<size_t>(i)];

      if (!layout.output_xyzi_only && layout.input_intensity_offset < 0) {
        const size_t bytes = count * static_cast<size_t>(cloud.point_step);
        const uint8_t * src = cloud.data.data() + begin * static_cast<size_t>(cloud.point_step);
        std::memcpy(chunk.data.data(), src, bytes);
        continue;
      }

      for (size_t local_index = 0; local_index < count; ++local_index) {
        uint8_t * dst = chunk.data.data() +
          local_index * static_cast<size_t>(layout.output_point_step);
        copyPointToOutput(cloud, begin + local_index, layout, dst);
      }
    }

    return result;
  }

  std::vector<sensor_msgs::msg::PointCloud2> makeEmptyChunks(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const builtin_interfaces::msg::Time & base_stamp,
    const double chunk_duration)
  {
    CopyLayout layout = makeCopyLayout(cloud);
    std::vector<builtin_interfaces::msg::Time> stamps;
    std::vector<size_t> point_counts(static_cast<size_t>(chunk_count_), 0);

    stamps.reserve(static_cast<size_t>(chunk_count_));
    for (int i = 0; i < chunk_count_; ++i) {
      stamps.push_back(addSeconds(base_stamp, static_cast<double>(i) * chunk_duration));
    }

    return makeChunksWithPointCounts(cloud, layout, stamps, point_counts);
  }

  std::vector<sensor_msgs::msg::PointCloud2> makeChunksWithPointCounts(
    const sensor_msgs::msg::PointCloud2 & source,
    const CopyLayout & layout,
    const std::vector<builtin_interfaces::msg::Time> & stamps,
    const std::vector<size_t> & point_counts) const
  {
    std::vector<sensor_msgs::msg::PointCloud2> chunks;
    chunks.reserve(static_cast<size_t>(chunk_count_));

    for (int i = 0; i < chunk_count_; ++i) {
      const size_t point_count = point_counts[static_cast<size_t>(i)];

      sensor_msgs::msg::PointCloud2 chunk;
      chunk.header.frame_id = source.header.frame_id;
      chunk.header.stamp = stamps[static_cast<size_t>(i)];
      chunk.height = 1;
      chunk.width = static_cast<uint32_t>(point_count);
      chunk.fields = layout.output_fields;
      chunk.is_bigendian = source.is_bigendian;
      chunk.point_step = layout.output_point_step;
      chunk.row_step = chunk.point_step * chunk.width;
      chunk.is_dense = source.is_dense;
      chunk.data.resize(point_count * static_cast<size_t>(layout.output_point_step), 0U);

      chunks.push_back(std::move(chunk));
    }

    return chunks;
  }

  CopyLayout makeCopyLayout(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    CopyLayout layout;

    if (output_xyzi_only_) {
      const int x_offset = findFieldOffset(
        cloud, "x", sensor_msgs::msg::PointField::FLOAT32, true);
      const int y_offset = findFieldOffset(
        cloud, "y", sensor_msgs::msg::PointField::FLOAT32, true);
      const int z_offset = findFieldOffset(
        cloud, "z", sensor_msgs::msg::PointField::FLOAT32, true);

      if (x_offset >= 0 && y_offset >= 0 && z_offset >= 0) {
        layout.output_xyzi_only = true;
        layout.output_point_step = 16;
        layout.input_x_offset = x_offset;
        layout.input_y_offset = y_offset;
        layout.input_z_offset = z_offset;
        layout.input_xyz_contiguous =
          y_offset == x_offset + static_cast<int>(sizeof(float)) &&
          z_offset == y_offset + static_cast<int>(sizeof(float));
        layout.output_fields = makeXyziFields();
        return layout;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "output_xyzi_only=true but x/y/z FLOAT32 fields are invalid, preserve original layout");
    }

    layout.output_xyzi_only = false;
    layout.output_point_step = cloud.point_step;
    layout.output_fields = cloud.fields;
    if (zero_intensity_) {
      layout.input_intensity_offset = findFieldOffset(
        cloud, "intensity", sensor_msgs::msg::PointField::FLOAT32, true);
    }
    return layout;
  }

  std::vector<sensor_msgs::msg::PointField> makeXyziFields() const
  {
    std::vector<sensor_msgs::msg::PointField> fields;
    fields.reserve(4);

    sensor_msgs::msg::PointField field;
    field.count = 1;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;

    field.name = "x";
    field.offset = 0;
    fields.push_back(field);

    field.name = "y";
    field.offset = 4;
    fields.push_back(field);

    field.name = "z";
    field.offset = 8;
    fields.push_back(field);

    field.name = "intensity";
    field.offset = 12;
    fields.push_back(field);

    return fields;
  }

  void copyPointToOutput(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const size_t point_index,
    const CopyLayout & layout,
    uint8_t * dst) const
  {
    const uint8_t * src = cloud.data.data() + point_index * static_cast<size_t>(cloud.point_step);

    if (layout.output_xyzi_only) {
      // 性能模式：输出只保留 XYZI 四个 float32 字段，i 固定为 0。
      // chunk.data 在创建时已经用 0 填充，因此这里仅写入 x/y/z，intensity 保持 0。
      if (layout.input_xyz_contiguous) {
        std::memcpy(dst, src + static_cast<size_t>(layout.input_x_offset), 3U * sizeof(float));
      } else {
        std::memcpy(dst, src + static_cast<size_t>(layout.input_x_offset), sizeof(float));
        std::memcpy(dst + 4, src + static_cast<size_t>(layout.input_y_offset), sizeof(float));
        std::memcpy(dst + 8, src + static_cast<size_t>(layout.input_z_offset), sizeof(float));
      }
      return;
    }

    std::memcpy(dst, src, static_cast<size_t>(cloud.point_step));
    if (layout.input_intensity_offset >= 0) {
      std::memset(dst + static_cast<size_t>(layout.input_intensity_offset), 0, sizeof(float));
    }
  }

  int findFieldOffset(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & field_name,
    const uint8_t datatype,
    const bool warn_on_mismatch)
  {
    for (const auto & field : cloud.fields) {
      if (field.name != field_name) {
        continue;
      }

      if (field.datatype != datatype) {
        if (warn_on_mismatch) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "field '%s' exists but datatype is unsupported", field_name.c_str());
        }
        return -1;
      }

      const size_t field_size = pointFieldDatatypeSize(datatype);
      if (field_size == 0 ||
        static_cast<size_t>(field.offset) + field_size > static_cast<size_t>(cloud.point_step))
      {
        if (warn_on_mismatch) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "field '%s' offset exceeds point_step", field_name.c_str());
        }
        return -1;
      }

      return static_cast<int>(field.offset);
    }

    if (warn_on_mismatch) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "field '%s' not found", field_name.c_str());
    }
    return -1;
  }

  size_t pointFieldDatatypeSize(const uint8_t datatype) const
  {
    switch (datatype) {
      case sensor_msgs::msg::PointField::FLOAT32:
        return sizeof(float);
      case sensor_msgs::msg::PointField::FLOAT64:
        return sizeof(double);
      default:
        return 0;
    }
  }

  builtin_interfaces::msg::Time addSeconds(
    const builtin_interfaces::msg::Time & stamp,
    const double offset_sec) const
  {
    const int64_t base_ns =
      static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
    const int64_t offset_ns = static_cast<int64_t>(offset_sec * 1e9);
    return makeStampFromNanoseconds(base_ns + offset_ns);
  }

  builtin_interfaces::msg::Time makeStampFromNanoseconds(const int64_t stamp_ns) const
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    return stamp;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string timestamp_field_name_;
  int chunk_count_{5};
  double input_cloud_hz_{10.0};
  bool timestamp_is_absolute_{true};
  bool fallback_to_index_split_{true};
  bool publish_empty_chunk_{false};
  bool output_xyzi_only_{true};
  bool zero_intensity_{true};
  int qos_depth_{5};
  bool drop_old_chunks_when_new_cloud_arrives_{true};
  bool debug_{true};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<sensor_msgs::msg::PointCloud2> pending_chunks_;
  size_t pending_index_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360ChunkCloudNode>());
  rclcpp::shutdown();
  return 0;
}
