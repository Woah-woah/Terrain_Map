// TerrainLayer：把 terrain 语义地图（NONE/SLOPE/TUNNEL）当作“人工可通行覆盖层”叠加到 Nav2 costmap。
//
// 数据流：
//   terrain.yaml -> terrain.pgm -> terrain_grid_（初始化时读取一次，之后不再做磁盘 IO）
//   updateCosts: master(i,j) -> mapToWorld -> costmap frame -> terrain frame -> worldToTerrainCell -> terrain_grid_
//
// 约定：
//   terrain 语义固定为 0=NONE、1=SLOPE、2=TUNNEL，PGM 像素值原样解释，不按 occupancy 阈值处理。
//   栅格索引与 ROS map 一致：index = my * width + mx，mx 向右、my 向上，PGM 的 Y 翻转在加载阶段完成。
//   语义对 costmap 的作用：
//     NONE  -> 不修改 master costmap
//     SLOPE -> 强制写为 nav2_costmap_2d::FREE_SPACE
//     TUNNEL-> 强制写为 nav2_costmap_2d::FREE_SPACE
//   （SLOPE 与 TUNNEL 对 costmap 完全等价，1/2 的区别仍保留在 terrain_grid_ 中，
//     供将来的其他行为模块继续区分坡道与狗洞。人工涂过即强制认为可通行，因此会覆盖 LETHAL/NO_INFORMATION。）

#include "terrain_layer/terrain_layer.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
// ============================================================================
// terrain 语义值，与 terrain.yaml 中 encoding 段一一对应
// ============================================================================
constexpr uint8_t kSemanticNone = 0;
constexpr uint8_t kSemanticSlope = 1;
constexpr uint8_t kSemanticTunnel = 2;

// ============================================================================
// 轻量 PGM 读取（支持 P5 二进制 / P2 ASCII，忽略 '#' 注释）
// 像素值原样读取为 uint8 语义值，不使用 occupied_thresh / free_thresh / negate
// ============================================================================
struct PgmImage
{
  unsigned int width{0};
  unsigned int height{0};
  unsigned int max_value{0};
  std::vector<uint8_t> pixels;  // 按图片行优先排列：image_y * width + x
};

// 读取一个 PGM 头部字段，自动跳过空白与 '#' 注释行
std::string readPgmToken(std::istream & in, const std::string & path)
{
  while (true) {
    const int next = in.peek();
    if (next == std::char_traits<char>::eof()) {
      throw std::runtime_error("读取 PGM 头部时意外到达文件末尾: " + path);
    }
    if (next == '#') {
      in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(next)) != 0) {
      in.get();
      continue;
    }
    break;
  }

  std::string token;
  if (!(in >> token)) {
    throw std::runtime_error("读取 PGM 头部字段失败: " + path);
  }
  return token;
}

unsigned int parseUnsignedToken(
  const std::string & token, const std::string & field_name, const std::string & path)
{
  try {
    std::size_t parsed_chars = 0;
    const unsigned long value = std::stoul(token, &parsed_chars);
    if (parsed_chars != token.size() || value > std::numeric_limits<unsigned int>::max()) {
      throw std::out_of_range("不是合法的无符号整数");
    }
    return static_cast<unsigned int>(value);
  } catch (const std::exception &) {
    throw std::runtime_error(
            "文件 " + path + " 中的 " + field_name + " 非法: '" + token + "'");
  }
}

PgmImage readSemanticPgm(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    throw std::runtime_error("无法打开 terrain 图像文件: " + path);
  }

  const std::string magic = readPgmToken(in, path);
  if (magic != "P5" && magic != "P2") {
    throw std::runtime_error(
            "文件 " + path + " 的 PGM 魔数 '" + magic + "' 不支持（仅支持 P5 二进制与 P2 ASCII）");
  }

  PgmImage image;
  image.width = parseUnsignedToken(readPgmToken(in, path), "PGM 宽度", path);
  image.height = parseUnsignedToken(readPgmToken(in, path), "PGM 高度", path);
  image.max_value = parseUnsignedToken(readPgmToken(in, path), "PGM max_value", path);

  if (image.width == 0 || image.height == 0) {
    throw std::runtime_error(
            "文件 " + path + " 的 PGM 尺寸非法: " + std::to_string(image.width) + "x" +
            std::to_string(image.height));
  }
  if (image.max_value == 0 || image.max_value > 255) {
    throw std::runtime_error(
            "文件 " + path + " 的 PGM max_value 不支持: " + std::to_string(image.max_value) +
            "（terrain 语义使用 uint8，必须是 1..255）");
  }

  const std::size_t pixel_count =
    static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height);
  image.pixels.resize(pixel_count);

  if (magic == "P5") {
    // max_value 之后只能有一个空白字符作为分隔符，二进制像素数据紧随其后
    char separator = '\0';
    if (!in.get(separator)) {
      throw std::runtime_error("P5 文件缺少像素数据: " + path);
    }
    if (separator == '\r' && in.peek() == '\n') {  // 兼容 CRLF 头部
      in.get();
    }

    in.read(
      reinterpret_cast<char *>(image.pixels.data()),
      static_cast<std::streamsize>(pixel_count));
    if (in.gcount() != static_cast<std::streamsize>(pixel_count)) {
      throw std::runtime_error(
              "P5 像素数据被截断: " + path + " 期望 " + std::to_string(pixel_count) +
              " 字节, 实际 " + std::to_string(static_cast<long long>(in.gcount())) + " 字节");
    }
  } else {
    for (std::size_t i = 0; i < pixel_count; ++i) {
      const unsigned int value =
        parseUnsignedToken(readPgmToken(in, path), "PGM 像素值", path);
      if (value > image.max_value) {
        throw std::runtime_error(
                "文件 " + path + " 中的 PGM 像素值 " + std::to_string(value) +
                " 超过 max_value " + std::to_string(image.max_value));
      }
      image.pixels[i] = static_cast<uint8_t>(value);
    }
  }

  // 语义值原样校验：不截断、不归一化、不当作 occupancy 概率
  for (const uint8_t value : image.pixels) {
    if (value != kSemanticNone && value != kSemanticSlope && value != kSemanticTunnel) {
      throw std::runtime_error(
              "文件 " + path + " 包含不支持的语义值 (unsupported semantic value): " +
              std::to_string(static_cast<int>(value)));
    }
  }

  return image;
}

// ============================================================================
// terrain.yaml 读取小工具
// ============================================================================
YAML::Node requireNode(
  const YAML::Node & parent, const std::string & key, const std::string & yaml_path)
{
  const YAML::Node node = parent[key];
  if (!node || !node.IsScalar()) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 缺少标量字段 '" + key + "'");
  }
  return node;
}

int parseIntField(const YAML::Node & parent, const std::string & key, const std::string & yaml_path)
{
  const YAML::Node node = requireNode(parent, key, yaml_path);
  try {
    return node.as<int>();
  } catch (const std::exception & e) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的整数字段 '" + key + "' 非法: " + e.what());
  }
}
}  // namespace

namespace terrain_layer
{
// ============================================================================
// 初始化：读取参数 -> 加载 terrain 文件 -> 校验基础指针 -> 打印统计
// ============================================================================
void TerrainLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TerrainLayer 无法获取父 Nav2 节点");
  }

  // Nav2 Humble Layer 惯例：declareParameter() 会自动补上 "<插件实例名>." 前缀
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  if (!enabled_) {
    // Nav2 Layer 标准行为：禁用时不加载数据，updateBounds/updateCosts 也全部空转
    RCLCPP_WARN(
      logger_, "TerrainLayer '%s' 已禁用 (enabled=false)，跳过 terrain 地图加载。",
      name_.c_str());
    return;
  }

  declareParameter("terrain_yaml", rclcpp::ParameterValue(std::string("")));
  node->get_parameter(name_ + ".terrain_yaml", terrain_yaml_);
  if (terrain_yaml_.empty()) {
    throw std::runtime_error("参数 '" + name_ + ".terrain_yaml' 未设置");
  }

  // terrain 只在初始化时读取一次，运行期不再做磁盘 IO
  loadTerrainMap(terrain_yaml_);

  if (layered_costmap_ == nullptr) {
    throw std::runtime_error("TerrainLayer '" + name_ + "' 未挂载到 layered costmap");
  }
  if (tf_ == nullptr) {
    throw std::runtime_error("TerrainLayer '" + name_ + "' 无法获取 Nav2 TF buffer");
  }

  logTerrainSummary();
}

void TerrainLayer::logTerrainSummary() const
{
  std::size_t none_count = 0;
  std::size_t slope_count = 0;
  std::size_t tunnel_count = 0;
  for (const uint8_t value : terrain_grid_) {
    if (value == kSemanticNone) {
      ++none_count;
    } else if (value == kSemanticSlope) {
      ++slope_count;
    } else {
      ++tunnel_count;
    }
  }

  RCLCPP_INFO(
    logger_,
    "TerrainLayer 加载完成:\n"
    "  文件=%s\n"
    "  尺寸=%ux%u\n"
    "  分辨率=%g\n"
    "  frame=%s\n"
    "  origin=(%g, %g, %g)\n"
    "  NONE=%zu\n"
    "  SLOPE=%zu\n"
    "  TUNNEL=%zu",
    terrain_yaml_.c_str(), terrain_width_, terrain_height_, terrain_resolution_,
    terrain_frame_id_.c_str(), terrain_origin_x_, terrain_origin_y_, terrain_origin_yaw_,
    none_count, slope_count, tunnel_count);
  RCLCPP_INFO(
    logger_,
    "TerrainLayer '%s': 人工可通行覆盖层，NONE 不修改 master costmap；"
    "SLOPE 与 TUNNEL 均强制写为 FREE_SPACE(0)。",
    name_.c_str());
}

// ============================================================================
// 代价地图回调
// ============================================================================
void TerrainLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || terrain_grid_.empty()) {
    return;
  }
  if (layered_costmap_ == nullptr) {
    current_ = false;
    return;
  }

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();

  // terrain 是静态先验图：需要更新的区域就是它在 world frame 下真实的 AABB，
  // 而不是无穷大范围；LayeredCostmap 会再按当前 costmap 窗口做裁剪。
  double terrain_min_x = 0.0;
  double terrain_min_y = 0.0;
  double terrain_max_x = 0.0;
  double terrain_max_y = 0.0;

  if (terrain_frame_id_ == costmap_frame) {
    computeTerrainWorldBounds(terrain_min_x, terrain_min_y, terrain_max_x, terrain_max_y);
    current_ = true;
  } else {
    geometry_msgs::msg::TransformStamped terrain_to_costmap;
    if (!lookupTransform(costmap_frame, terrain_frame_id_, terrain_to_costmap, "updateBounds")) {
      current_ = false;
      return;
    }

    terrain_min_x = std::numeric_limits<double>::max();
    terrain_min_y = std::numeric_limits<double>::max();
    terrain_max_x = std::numeric_limits<double>::lowest();
    terrain_max_y = std::numeric_limits<double>::lowest();
    for (const auto & corner : computeTerrainWorldCorners()) {
      double costmap_x = 0.0;
      double costmap_y = 0.0;
      transformPoint(terrain_to_costmap, corner[0], corner[1], costmap_x, costmap_y);
      terrain_min_x = std::min(terrain_min_x, costmap_x);
      terrain_min_y = std::min(terrain_min_y, costmap_y);
      terrain_max_x = std::max(terrain_max_x, costmap_x);
      terrain_max_y = std::max(terrain_max_y, costmap_y);
    }
    current_ = true;
  }

  *min_x = std::min(*min_x, terrain_min_x);
  *min_y = std::min(*min_y, terrain_min_y);
  *max_x = std::max(*max_x, terrain_max_x);
  *max_y = std::max(*max_y, terrain_max_y);
}

void TerrainLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || terrain_grid_.empty()) {
    return;
  }
  if (layered_costmap_ == nullptr) {
    current_ = false;
    return;
  }

  const unsigned int master_size_x = master_grid.getSizeInCellsX();
  const unsigned int master_size_y = master_grid.getSizeInCellsY();
  if (master_size_x == 0 || master_size_y == 0) {
    return;
  }

  // 只处理 LayeredCostmap 传进来的更新窗口，并夹到 master 范围内
  const int i_begin = std::max(0, min_i);
  const int j_begin = std::max(0, min_j);
  const int i_end = std::min(static_cast<int>(master_size_x), max_i);
  const int j_end = std::min(static_cast<int>(master_size_y), max_j);

  const std::string costmap_frame = layered_costmap_->getGlobalFrameID();
  const bool same_frame = (terrain_frame_id_ == costmap_frame);
  geometry_msgs::msg::TransformStamped costmap_to_terrain;
  if (!same_frame &&
    !lookupTransform(terrain_frame_id_, costmap_frame, costmap_to_terrain, "updateCosts"))
  {
    current_ = false;
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  for (int j = j_begin; j < j_end; ++j) {
    for (int i = i_begin; i < i_end; ++i) {
      // master 与 terrain 的尺寸/origin/resolution 都可能不同（例如 rolling local costmap），
      // 所以必须 master 格 -> 世界坐标 -> terrain 格，不能直接用 (i, j) 索引 terrain。
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

      if (!same_frame) {
        double terrain_wx = 0.0;
        double terrain_wy = 0.0;
        transformPoint(costmap_to_terrain, wx, wy, terrain_wx, terrain_wy);
        wx = terrain_wx;
        wy = terrain_wy;
      }

      unsigned int terrain_mx = 0;
      unsigned int terrain_my = 0;
      if (!worldToTerrainCell(wx, wy, terrain_mx, terrain_my)) {
        continue;  // 落在 terrain 地图之外，等价于 NONE
      }

      const uint8_t semantic = terrainAt(terrain_mx, terrain_my);
      if (semantic == kSemanticNone) {
        continue;  // NONE：完全不碰 master costmap
      }

      // SLOPE(1) 与 TUNNEL(2) 对 costmap 等价：人工涂过即强制认为可通行，
      // 无条件覆盖为 FREE_SPACE（原来的 LETHAL / INSCRIBED / NO_INFORMATION 都会被覆盖）
      const unsigned int index =
        static_cast<unsigned int>(j) * master_size_x + static_cast<unsigned int>(i);
      master_array[index] = nav2_costmap_2d::FREE_SPACE;
    }
  }
  current_ = true;
}

void TerrainLayer::reset()
{
  // terrain_grid_ 来自静态文件，reset() 不需要重新加载
}

bool TerrainLayer::isClearable()
{
  // 人工可通行覆盖层来自静态文件，不参与 clearing 操作
  return false;
}

// ============================================================================
// 坐标变换与语义覆盖
// ============================================================================
bool TerrainLayer::worldToTerrainCell(
  double wx, double wy, unsigned int & terrain_mx, unsigned int & terrain_my) const
{
  // 1) 世界坐标先减去 terrain origin
  const double dx = wx - terrain_origin_x_;
  const double dy = wy - terrain_origin_y_;

  // 2) 用 R(-yaw) 逆旋转回 terrain 栅格坐标系
  //    [ cos  sin ] [dx]   [local_x]
  //    [-sin  cos ] [dy] = [local_y]
  const double local_x = terrain_cos_yaw_ * dx + terrain_sin_yaw_ * dy;
  const double local_y = -terrain_sin_yaw_ * dx + terrain_cos_yaw_ * dy;

  // 3) floor(local / resolution) 得到 terrain 栅格（PGM 的 Y 翻转已在加载阶段完成）
  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }
  const double cell_x = std::floor(local_x / terrain_resolution_);
  const double cell_y = std::floor(local_y / terrain_resolution_);
  if (cell_x >= static_cast<double>(terrain_width_) ||
    cell_y >= static_cast<double>(terrain_height_))
  {
    return false;
  }

  terrain_mx = static_cast<unsigned int>(cell_x);
  terrain_my = static_cast<unsigned int>(cell_y);
  return true;
}

uint8_t TerrainLayer::terrainAt(unsigned int terrain_mx, unsigned int terrain_my) const
{
  if (terrain_mx >= terrain_width_ || terrain_my >= terrain_height_) {
    return kSemanticNone;
  }
  return terrain_grid_[static_cast<std::size_t>(terrain_my) * terrain_width_ + terrain_mx];
}

void TerrainLayer::computeTerrainWorldBounds(
  double & min_x, double & min_y, double & max_x, double & max_y) const
{
  min_x = std::numeric_limits<double>::max();
  min_y = std::numeric_limits<double>::max();
  max_x = std::numeric_limits<double>::lowest();
  max_y = std::numeric_limits<double>::lowest();

  for (const auto & corner : computeTerrainWorldCorners()) {
    min_x = std::min(min_x, corner[0]);
    min_y = std::min(min_y, corner[1]);
    max_x = std::max(max_x, corner[0]);
    max_y = std::max(max_y, corner[1]);
  }
}

std::array<std::array<double, 2>, 4> TerrainLayer::computeTerrainWorldCorners() const
{
  const double terrain_size_x = static_cast<double>(terrain_width_) * terrain_resolution_;
  const double terrain_size_y = static_cast<double>(terrain_height_) * terrain_resolution_;
  const double corner_x[4] = {0.0, terrain_size_x, terrain_size_x, 0.0};
  const double corner_y[4] = {0.0, 0.0, terrain_size_y, terrain_size_y};
  std::array<std::array<double, 2>, 4> corners{};

  // 四个角经 R(yaw) 旋转后再取 AABB，保证考虑 origin yaw
  for (int k = 0; k < 4; ++k) {
    corners[static_cast<std::size_t>(k)][0] =
      terrain_origin_x_ + terrain_cos_yaw_ * corner_x[k] - terrain_sin_yaw_ * corner_y[k];
    corners[static_cast<std::size_t>(k)][1] =
      terrain_origin_y_ + terrain_sin_yaw_ * corner_x[k] + terrain_cos_yaw_ * corner_y[k];
  }

  return corners;
}

bool TerrainLayer::lookupTransform(
  const std::string & target_frame, const std::string & source_frame,
  geometry_msgs::msg::TransformStamped & transform, const char * context)
{
  if (tf_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "TerrainLayer '%s' %s: TF buffer 为空，跳过本轮 terrain 更新 (source='%s', target='%s')",
      name_.c_str(), context, source_frame.c_str(), target_frame.c_str());
    return false;
  }

  try {
    transform = tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "TerrainLayer '%s' %s: 获取 TF 失败，跳过本轮 terrain 更新 "
      "(source='%s', target='%s'): %s",
      name_.c_str(), context, source_frame.c_str(), target_frame.c_str(), e.what());
    return false;
  }
}

void TerrainLayer::transformPoint(
  const geometry_msgs::msg::TransformStamped & transform,
  double source_x, double source_y, double & target_x, double & target_y) const
{
  geometry_msgs::msg::PointStamped source_point;
  source_point.header.frame_id = transform.child_frame_id;
  source_point.point.x = source_x;
  source_point.point.y = source_y;
  source_point.point.z = 0.0;

  geometry_msgs::msg::PointStamped target_point;
  tf2::doTransform(source_point, target_point, transform);
  target_x = target_point.point.x;
  target_y = target_point.point.y;
}

// ============================================================================
// terrain.yaml + terrain.pgm -> terrain_grid_
// ============================================================================
void TerrainLayer::loadTerrainMap(const std::string & yaml_path)
{
  namespace fs = std::filesystem;

  YAML::Node doc;
  try {
    doc = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("加载 terrain.yaml '" + yaml_path + "' 失败: " + e.what());
  }

  terrain_frame_id_ = requireNode(doc, "frame_id", yaml_path).as<std::string>();
  if (terrain_frame_id_.empty()) {
    throw std::runtime_error("terrain.yaml '" + yaml_path + "' 的 frame_id 为空");
  }

  const YAML::Node resolution_node = requireNode(doc, "resolution", yaml_path);
  try {
    terrain_resolution_ = resolution_node.as<double>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的 resolution 非法: " + e.what());
  }
  if (!(terrain_resolution_ > 0.0)) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的 resolution 必须大于 0，当前为 " +
            std::to_string(terrain_resolution_));
  }

  const int width = parseIntField(doc, "width", yaml_path);
  const int height = parseIntField(doc, "height", yaml_path);
  if (width <= 0 || height <= 0) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的尺寸非法: " + std::to_string(width) + "x" +
            std::to_string(height));
  }
  terrain_width_ = static_cast<unsigned int>(width);
  terrain_height_ = static_cast<unsigned int>(height);

  // origin 支持块序列与行内序列 [-14.0, -7.5, 0.0] 两种写法，yaml-cpp 都能解析
  const YAML::Node origin = doc["origin"];
  if (!origin || !origin.IsSequence() || origin.size() < 3) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的 origin 至少需要 3 个元素");
  }
  try {
    terrain_origin_x_ = origin[0].as<double>();
    terrain_origin_y_ = origin[1].as<double>();
    terrain_origin_yaw_ = origin[2].as<double>();
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的 origin 数值非法: " + e.what());
  }

  // 语义编码必须严格是 none=0 / slope=1 / tunnel=2，不做任何自动转换
  const YAML::Node encoding = doc["encoding"];
  if (!encoding || !encoding.IsMap()) {
    throw std::runtime_error("terrain.yaml '" + yaml_path + "' 缺少 map 类型的 'encoding' 段");
  }
  const int none_value = parseIntField(encoding, "none", yaml_path);
  const int slope_value = parseIntField(encoding, "slope", yaml_path);
  const int tunnel_value = parseIntField(encoding, "tunnel", yaml_path);
  if (none_value != kSemanticNone || slope_value != kSemanticSlope ||
    tunnel_value != kSemanticTunnel)
  {
    throw std::runtime_error(
            "terrain.yaml '" + yaml_path + "' 的 encoding 必须为 none=0, slope=1, tunnel=2，"
            "当前为 none=" + std::to_string(none_value) + ", slope=" + std::to_string(slope_value) +
            ", tunnel=" + std::to_string(tunnel_value));
  }

  // image 通常是相对 terrain.yaml 所在目录的路径；绝对路径则直接使用
  const std::string image_field = requireNode(doc, "image", yaml_path).as<std::string>();
  if (image_field.empty()) {
    throw std::runtime_error("terrain.yaml '" + yaml_path + "' 的 image 字段为空");
  }
  const fs::path yaml_file(yaml_path);
  fs::path image_path(image_field);
  if (image_path.is_relative()) {
    image_path = yaml_file.parent_path() / image_path;
  }
  image_path = image_path.lexically_normal();

  const PgmImage image = readSemanticPgm(image_path.string());

  if (image.width != terrain_width_ || image.height != terrain_height_) {
    throw std::runtime_error(
            "terrain 图像 '" + image_path.string() + "' 的尺寸 " + std::to_string(image.width) +
            "x" + std::to_string(image.height) + " 与 terrain.yaml 的 " +
            std::to_string(terrain_width_) + "x" + std::to_string(terrain_height_) + " 不一致");
  }

  const std::size_t expected_cells =
    static_cast<std::size_t>(terrain_width_) * static_cast<std::size_t>(terrain_height_);
  if (image.pixels.size() != expected_cells) {
    throw std::runtime_error(
            "terrain 图像 '" + image_path.string() + "' 有 " +
            std::to_string(image.pixels.size()) + " 个像素，期望 " +
            std::to_string(expected_cells) + " 个");
  }

  // Y 轴翻转：PGM 第 0 行在图片顶部，而 ROS map 的 my=0 在栅格底部，
  // 因此 image_y 行要放到 map_y = height - 1 - image_y 行。
  // 这样 terrain_grid_[my * width + mx] 才与 Nav2/ROS 地图中同一个 (mx, my) 对应现实中的同一位置。
  terrain_grid_.assign(expected_cells, kSemanticNone);
  for (unsigned int image_y = 0; image_y < terrain_height_; ++image_y) {
    const unsigned int map_y = terrain_height_ - 1u - image_y;
    for (unsigned int mx = 0; mx < terrain_width_; ++mx) {
      const std::size_t image_index =
        static_cast<std::size_t>(image_y) * terrain_width_ + mx;
      const std::size_t map_index = static_cast<std::size_t>(map_y) * terrain_width_ + mx;
      terrain_grid_[map_index] = image.pixels[image_index];
    }
  }

  // yaw 的 sin/cos 预计算，避免 updateCosts 逐格循环里反复调用三角函数
  terrain_cos_yaw_ = std::cos(terrain_origin_yaw_);
  terrain_sin_yaw_ = std::sin(terrain_origin_yaw_);
}
}  // namespace terrain_layer

PLUGINLIB_EXPORT_CLASS(terrain_layer::TerrainLayer, nav2_costmap_2d::Layer)
