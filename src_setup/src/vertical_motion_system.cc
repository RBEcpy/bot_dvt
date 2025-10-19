// vertical_motion_system.cc
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>
#include <chrono>
#include <cmath>
#include <string>

using namespace gz;
using namespace gz::sim;

class VerticalMotionSystem
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate
{
public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "[VerticalMotionSystem] Invalid model entity\n";
      return;
    }

    // Lấy pose gốc của MODEL (world frame)
    if (auto poseComp = _ecm.Component<components::Pose>(this->model.Entity()))
      this->basePose = poseComp->Data();
    else
      this->basePose = math::Pose3d::Zero;

    // Tham số từ SDF
    if (_sdf)
    {
      this->mode   = _sdf->Get<std::string>("mode", "sine").first; // "sine" | "bounce"
      this->axis   = _sdf->Get<std::string>("axis", "x").first;    // "x" | "y" | "z" (mặc định: x)

      // sine
      this->amp    = _sdf->Get<double>("amplitude", 1.0).first;    // m
      this->omega  = _sdf->Get<double>("omega",     0.8).first;    // rad/s
      this->phase  = _sdf->Get<double>("phase",     0.0).first;    // rad

      // bounce (min/max tương đối so với base theo trục đã chọn)
      this->minVal = _sdf->Get<double>("min", -1.0).first;         // m
      this->maxVal = _sdf->Get<double>("max",  1.0).first;         // m
      this->speed  = _sdf->Get<double>("speed", 0.5).first;        // m/s
    }

    if (this->axis != "x" && this->axis != "y" && this->axis != "z")
    {
      gzwarn << "[VerticalMotionSystem] Invalid axis '" << this->axis
             << "' -> default to 'x'\n";
      this->axis = "x";
    }

    this->configured = true;

    gzdbg << "[VerticalMotionSystem] mode=" << this->mode
          << " axis=" << this->axis
          << " amp=" << this->amp
          << " omega=" << this->omega
          << " min=" << this->minVal
          << " max=" << this->maxVal
          << " speed=" << this->speed << std::endl;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    // Tính offset dọc theo trục đã chọn (WORLD frame)
    double offset = 0.0;

    if (this->mode == "sine")
    {
      const double t = std::chrono::duration<double>(_info.simTime).count();
      offset = this->amp * std::sin(this->omega * t + this->phase);
    }
    else // "bounce"
    {
      const double dt = std::chrono::duration<double>(_info.dt).count();
      this->curVal += (this->goingUp ? +this->speed : -this->speed) * dt;

      if (this->curVal > this->maxVal) { this->curVal = this->maxVal; this->goingUp = false; }
      if (this->curVal < this->minVal) { this->curVal = this->minVal; this->goingUp = true;  }

      offset = this->curVal;
    }

    // Pose mục tiêu
    math::Vector3d p = this->basePose.Pos();
    if (this->axis == "x")      p.X() = this->basePose.Pos().X() + offset;
    else if (this->axis == "y") p.Y() = this->basePose.Pos().Y() + offset;
    else                        p.Z() = this->basePose.Pos().Z() + offset;

    math::Pose3d target(p, this->basePose.Rot());

    // Gửi lệnh pose cho MODEL (mượt, chuẩn physics)
    this->model.SetWorldPoseCmd(_ecm, target);
  }

private:
  Model model{kNullEntity};
  math::Pose3d basePose{math::Pose3d::Zero};
  bool configured{false};

  // Params
  std::string mode{"sine"};
  std::string axis{"x"};   // chạy ngang theo X (map)
  double amp{1.0};
  double omega{0.8};
  double phase{0.0};
  double minVal{-1.0};
  double maxVal{ 1.0};
  double speed{0.5};
  bool goingUp{true};
  double curVal{0.0};
};

// Đăng ký plugin
GZ_ADD_PLUGIN(VerticalMotionSystem,
              gz::sim::System,
              VerticalMotionSystem::ISystemConfigure,
              VerticalMotionSystem::ISystemPreUpdate)
// Nếu muốn dùng name="VerticalMotion" trong SDF, bật dòng dưới:
// GZ_ADD_PLUGIN_ALIAS(VerticalMotionSystem, "VerticalMotion")
