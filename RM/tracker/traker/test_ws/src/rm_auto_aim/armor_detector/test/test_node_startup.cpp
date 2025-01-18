// 测试ArmorDetectorNode节点，确保 ArmorDetectorNode 节点能够正确初始化并启动，没有抛出异常或错误

#include <gtest/gtest.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

// STD
#include <memory>

#include "armor_detector/detector_node.hpp"

TEST(ArmorDetectorNodeTest, NodeStartupTest)
{
  rclcpp::NodeOptions options;//配置节点选项
  auto node = std::make_shared<rm_auto_aim::ArmorDetectorNode>(options);//节点实例
  node.reset();//释放节点资源
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
  /*
  执行器，SingleThreadedExecutor管理和运行节点，以确保节点能够正常处理回调和事件
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->spin_some();
*/
}
