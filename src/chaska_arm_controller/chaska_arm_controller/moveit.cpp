#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Función auxiliar para planificar y ejecutar
void plan_and_execute(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const geometry_msgs::msg::Pose& pose,
  const rclcpp::Logger& logger)
{
  move_group.setPoseTarget(pose);

  auto const [success, plan] = [&move_group] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success) {
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "¡Planificación fallida!");
  }
}

int main(int argc, char * argv[])
{
  // Inicializar ROS2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit_xarm6",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("hello_moveit_xarm6");

  // IMPORTANTE: El planning group del xArm 6 es "xarm6"
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "xarm6");

  // Velocidad y aceleración (opcional, valores entre 0.0 y 1.0)
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  // Pose 1: posición frontal (en metros, frame base del xArm)
  geometry_msgs::msg::Pose pose1;
  pose1.orientation.w = 1.0;
  pose1.position.x = 0.3;
  pose1.position.y = 0.0;
  pose1.position.z = 0.4;

  // Pose 2: movimiento lateral izquierdo
  geometry_msgs::msg::Pose pose2 = pose1;
  pose2.position.y = -0.25;

  // Ejecutar movimientos secuenciales
  plan_and_execute(move_group_interface, pose1, logger);
  plan_and_execute(move_group_interface, pose2, logger);

  // Pose 3: con orientación personalizada (usando RPY)
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    double roll  = 3.1415;
    double pitch = 0.0;
    double yaw   = 0.0;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    msg.orientation = tf2::toMsg(q);
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();

  plan_and_execute(move_group_interface, target_pose, logger);

  rclcpp::shutdown();
  return 0;
}