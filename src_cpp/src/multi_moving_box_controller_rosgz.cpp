#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <cmath>
using namespace std::chrono_literals;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

class MovingBoxesGz : public rclcpp::Node {
public:
  MovingBoxesGz() : Node("multi_moving_box_controller_gz")
  {
    // Tham số tên world trong gz (mặc định "default")
    declare_parameter<std::string>("world_name", "default");
    get_parameter("world_name", world_);

    // Service: /world/<world>/set_entity_pose
    std::string srv = "/world/" + world_ + "/set_entity_pose";
    client_ = create_client<SetEntityPose>(srv);

    // Timer 10 Hz giống bản Python
    timer_ = create_wall_timer(100ms, std::bind(&MovingBoxesGz::tick, this));
  }

private:
  // Tạo pose nhanh
  static geometry_msgs::msg::Pose pose(double x, double y, double z)
  { geometry_msgs::msg::Pose p; p.position.x=x; p.position.y=y; p.position.z=z; p.orientation.w=1.0; return p; }

  // Gửi 1 yêu cầu đặt pose cho entity tên 'name'
  void sendPose(const std::string &name, const geometry_msgs::msg::Pose &p)
  {
    auto req = std::make_shared<SetEntityPose::Request>();
    req->entity.name = name;
    req->entity.type = ros_gz_interfaces::msg::Entity::MODEL; // đặt loại là MODEL
    req->pose = p;
    (void)client_->async_send_request(req); // fire-and-forget
  }

  void tick()
  {
    if (!ready_) {
      if (!client_->wait_for_service(0s)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Waiting for /world/%s/set_entity_pose ...", world_.c_str());
        return;
      }
      ready_ = true;
    }

    // 3 box: ngang, dọc, tròn
    sendPose("moving_box",   pose(x_, 0.7, 0.0));                                  // ngang theo X
    sendPose("moving_box_0", pose(-4.0, y_, 0.0));                                  // dọc theo Y
    sendPose("moving_box_1", pose(-1.5 + std::cos(t_), 1.0 + std::sin(t_), 0.0));  // quỹ đạo tròn R≈1

    // Cập nhật tham số chuyển động
    x_ += dir_*step_;  y_ += dir_*step_;
    if (x_ > x_max_ || x_ < x_min_) dir_ *= -1.0;
    if (y_ > y_max_ || y_ < y_min_) dir_ *= -1.0;
    t_ += 0.05;
  }

  // ROS
  rclcpp::Client<SetEntityPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool ready_{false};
  std::string world_{"default"};

  // Tham số chuyển động (giống Python)
  double dir_{1.0}, x_{0.85}, y_{2.0}, t_{0.0};
  const double step_{0.05}, x_min_{-3.0}, x_max_{3.0}, y_min_{0.5}, y_max_{4.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovingBoxesGz>());
  rclcpp::shutdown();
  return 0;
}
