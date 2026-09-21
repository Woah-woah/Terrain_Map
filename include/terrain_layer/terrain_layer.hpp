#ifndef TERRAIN_LAYER__TERRAIN_LAYER_HPP_
#define TERRAIN_LAYER__TERRAIN_LAYER_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace terrain_layer
{
class TerrainLayer : public nav2_costmap_2d::Layer
{
public:
  TerrainLayer() = default;
  ~TerrainLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;

private:
  void loadTerrainMap(const std::string & yaml_path);
  void logTerrainSummary() const;

  // 坐标变换：master 世界坐标 -> terrain 栅格坐标
  bool worldToTerrainCell(
    double wx, double wy, unsigned int & terrain_mx, unsigned int & terrain_my) const;
  // 读取 terrain 语义值（越界返回 NONE）
  uint8_t terrainAt(unsigned int terrain_mx, unsigned int terrain_my) const;
  // terrain 栅格在 world frame 下旋转后的包围盒
  void computeTerrainWorldBounds(
    double & min_x, double & min_y, double & max_x, double & max_y) const;

  // terrain.yaml -> terrain.pgm -> terrain_grid_ 只在初始化时读取一次
  std::string terrain_yaml_;
  std::string terrain_frame_id_;
  unsigned int terrain_width_{0};
  unsigned int terrain_height_{0};
  double terrain_resolution_{0.05};
  double terrain_origin_x_{0.0};
  double terrain_origin_y_{0.0};
  double terrain_origin_yaw_{0.0};
  // yaw 的 sin/cos 在加载时算好，避免 updateCosts 的逐格循环里反复调用三角函数
  double terrain_cos_yaw_{1.0};
  double terrain_sin_yaw_{0.0};
  std::vector<uint8_t> terrain_grid_;
};
} // namespace terrain_layer

#endif // TERRAIN_LAYER__TERRAIN_LAYER_HPP_
