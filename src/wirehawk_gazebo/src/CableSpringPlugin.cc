#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <vector>
#include <mutex>
#include <algorithm>
#include <string>
#include <functional>

namespace wirehawk {
  class CableSpringPlugin
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate 
  {
  private:
    gz::sim::Entity payload_link_entity_;
    std::vector<gz::sim::Entity> cable_models_;
    std::vector<gz::math::Vector3d> anchors_;
    
    size_t num_cables_ = 0;
    std::vector<double> prev_L_act_;
    std::vector<double> L0_;
    std::vector<double> prev_L0_;
    std::vector<bool> received_first_cmd_;

    double K_ = 5000.0;
    double C_ = 800.0;

    gz::transport::Node node_;
    std::mutex msg_mutex_;

  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override 
    {
      // 1. Read optional configurable parameters from SDF
      if (_sdf) {
        if (_sdf->HasElement("stiffness")) {
          K_ = _sdf->Get<double>("stiffness");
        }
        if (_sdf->HasElement("damping")) {
          C_ = _sdf->Get<double>("damping");
        }
      }

      gz::sim::Model model(_entity);
      payload_link_entity_ = model.LinkByName(_ecm, "mass_link");
      
      if (payload_link_entity_ == gz::sim::kNullEntity) {
        gzerr << "CableSpringPlugin: Could not find payload link named [mass_link]\n";
        return;
      }

      // Discover 4 single visual cable models
      for (int i = 0; i < 4; ++i) {
        std::string c_name = "cable_vis_" + std::to_string(i);
        gz::sim::Entity c_ent = gz::sim::kNullEntity;
        _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_ent, const gz::sim::components::Model *, const gz::sim::components::Name *_nameComp) -> bool {
            if (_nameComp->Data() == c_name) {
              c_ent = _ent;
              return false;
            }
            return true;
          });
        cable_models_.push_back(c_ent);
      }

      // Discover pillars
      std::vector<std::string> pillar_names = {"pillar_1", "pillar_2", "pillar_3", "pillar_4"};
      for (const auto &name : pillar_names) {
        gz::sim::Entity pillar_entity = gz::sim::kNullEntity;
        _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_ent, const gz::sim::components::Model *, const gz::sim::components::Name *_nameComp) -> bool {
            if (_nameComp->Data() == name) {
              pillar_entity = _ent;
              return false; 
            }
            return true;
          });

        if (pillar_entity != gz::sim::kNullEntity) {
          gz::math::Pose3d pose = gz::sim::worldPose(pillar_entity, _ecm);
          gz::math::Vector3d pos = pose.Pos();
          pos.Z() += 1.5; // Top of 3m pillar
          anchors_.push_back(pos);
        }
      }

      num_cables_ = anchors_.size();
      if (num_cables_ != 4) {
        gzerr << "CableSpringPlugin: Expected 4 pillars, found " << num_cables_ << ". Aborting.\n";
        return;
      }

      // 2. Compute initial cable lengths dynamically from actual spawn positions
      gz::sim::Link payload(payload_link_entity_);
      gz::math::Vector3d p0 = gz::sim::worldPose(payload_link_entity_, _ecm).Pos();

      prev_L_act_.assign(num_cables_, 0.0);
      L0_.resize(num_cables_);
      prev_L0_.resize(num_cables_);
      received_first_cmd_.assign(num_cables_, false);

      for (size_t i = 0; i < num_cables_; ++i) {
        double dist0 = (anchors_[i] - p0).Length();
        L0_[i] = dist0;
        prev_L0_[i] = dist0;
      }

      if (!_ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(payload_link_entity_)) {
        _ecm.CreateComponent(payload_link_entity_, gz::sim::components::ExternalWorldWrenchCmd());
      }

      for (size_t i = 0; i < num_cables_; ++i) {
        std::string topic = "/cdpr/l" + std::to_string(i);
        std::function<void(const gz::msgs::Double &)> cb = [this, i](const gz::msgs::Double &_msg) {
          std::lock_guard<std::mutex> lock(this->msg_mutex_);
          if (i < this->L0_.size()) {
            double val = std::max(1.0, _msg.data());
            if (!this->received_first_cmd_[i]) {
              this->prev_L0_[i] = val;
              this->received_first_cmd_[i] = true;
            }
            this->L0_[i] = val;
          }
        };
        node_.Subscribe(topic, cb);
      }
      
      gzmsg << "CableSpringPlugin configured: K=" << K_ << ", C=" << C_ 
            << ", initial lengths automatically computed from spawn pose.\n";
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override 
    {
      if (_info.paused || payload_link_entity_ == gz::sim::kNullEntity) return; 

      gz::sim::Link payload(payload_link_entity_);
      auto poseOpt = payload.WorldPose(_ecm);
      if (!poseOpt.has_value()) return;

      gz::math::Vector3d p = poseOpt.value().Pos();
      gz::math::Vector3d net_force(0.0, 0.0, 0.0);
      double dt = std::chrono::duration<double>(_info.dt).count();

      std::vector<double> current_L0(num_cables_);
      {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        current_L0 = L0_;
      }

      for (size_t i = 0; i < num_cables_; ++i) {
        gz::math::Vector3d l_vec = anchors_[i] - p;
        double L_act = l_vec.Length();
        
        double L_dot = 0.0;
        double L0_dot = 0.0;
        
        if (dt > 0.0001 && prev_L_act_[i] > 0) {
            L_dot = (L_act - prev_L_act_[i]) / dt;
            L0_dot = (current_L0[i] - prev_L0_[i]) / dt;
        }
        
        L_dot = std::clamp(L_dot, -20.0, 20.0);
        L0_dot = std::clamp(L0_dot, -20.0, 20.0);
        
        prev_L_act_[i] = L_act;
        prev_L0_[i] = current_L0[i];

        double tension = K_ * (L_act - current_L0[i]) + C_ * (L_dot - L0_dot);
        if (tension < 0.0) tension = 0.0;
        if (tension > 20000.0) tension = 20000.0;

        net_force += l_vec.Normalize() * tension;

        // Visual cable tracking
        if (i < cable_models_.size() && cable_models_[i] != gz::sim::kNullEntity) {
          gz::math::Vector3d dir = l_vec.Normalize();
          gz::math::Quaterniond rot;
          rot.SetFrom2Axes(gz::math::Vector3d::UnitZ, dir);

          gz::sim::Model cable_model(cable_models_[i]);
          cable_model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(p, rot));
        }
      }

      gz::msgs::Wrench wrench_msg;
      gz::msgs::Set(wrench_msg.mutable_force(), net_force);
      gz::msgs::Set(wrench_msg.mutable_torque(), gz::math::Vector3d::Zero);

      _ecm.SetComponentData<gz::sim::components::ExternalWorldWrenchCmd>(
          payload_link_entity_, wrench_msg);
    }
  };
}

GZ_ADD_PLUGIN(wirehawk::CableSpringPlugin, gz::sim::System,
              wirehawk::CableSpringPlugin::ISystemConfigure,
              wirehawk::CableSpringPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(wirehawk::CableSpringPlugin, "wirehawk::CableSpringPlugin")