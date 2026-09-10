#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
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
    gz::sim::Entity payload_link_entity_{gz::sim::kNullEntity};
    std::vector<gz::sim::Entity> cable_models_;
    std::vector<gz::math::Vector3d> anchors_;
    
    size_t num_cables_ = 0;
    std::vector<double> L0_;

    double K_ = 5000.0;  // Spring coefficient (single source of truth: wirehawk_world.sdf)
    double C_ = 2500.0;  // Damping coefficient (single source of truth: wirehawk_world.sdf)
    double drag_ = 150.0;  // Payload viscous drag [N/(m/s)] — damps soft modes near workspace edge
    double max_tension_ = 5000.0;  // Tension clamp [N] — caps violent cable pulls that destabilize the sim

    gz::transport::Node node_;
    std::mutex msg_mutex_;

  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override 
    {
      if (_sdf) {
        if (_sdf->HasElement("stiffness")) {
          K_ = _sdf->Get<double>("stiffness");
        }
        if (_sdf->HasElement("damping")) {
          C_ = _sdf->Get<double>("damping");
        }
        if (_sdf->HasElement("drag")) {
          drag_ = _sdf->Get<double>("drag");
        }
        if (_sdf->HasElement("max_tension")) {
          max_tension_ = _sdf->Get<double>("max_tension");
        }
      }

      gz::sim::Model model(_entity);
      payload_link_entity_ = model.LinkByName(_ecm, "mass_link");
      
      if (payload_link_entity_ == gz::sim::kNullEntity) {
        gzerr << "CableSpringPlugin: Could not find payload link named [mass_link]\n";
        return;
      }

      // Discover cable visual markers
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

      // Compute initial resting lengths from spawn pose
      gz::math::Vector3d p0 = gz::sim::worldPose(payload_link_entity_, _ecm).Pos();
      L0_.resize(num_cables_);

      for (size_t i = 0; i < num_cables_; ++i) {
        L0_[i] = (anchors_[i] - p0).Length();
      }

      if (!_ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(payload_link_entity_)) {
        _ecm.CreateComponent(payload_link_entity_, gz::sim::components::ExternalWorldWrenchCmd());
      }

      // Enable linear velocity component tracking if absent
      gz::sim::enableComponent<gz::sim::components::LinearVelocity>(_ecm, payload_link_entity_);

      for (size_t i = 0; i < num_cables_; ++i) {
        std::string topic = "/cdpr/l" + std::to_string(i);
        std::function<void(const gz::msgs::Double &)> cb = [this, i](const gz::msgs::Double &_msg) {
          std::lock_guard<std::mutex> lock(this->msg_mutex_);
          if (i < this->L0_.size()) {
            this->L0_[i] = std::max(0.1, _msg.data());
          }
        };
        node_.Subscribe(topic, cb);
      }
      
      gzmsg << "CableSpringPlugin configured: K=" << K_ << ", C=" << C_ << ", drag=" << drag_ << "\n";
    }

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override 
    {
      if (_info.paused || payload_link_entity_ == gz::sim::kNullEntity) return; 

      gz::sim::Link payload(payload_link_entity_);
      auto poseOpt = payload.WorldPose(_ecm);
      if (!poseOpt.has_value()) return;

      gz::math::Vector3d p = poseOpt.value().Pos();
      
      // Get physical velocity of payload directly from physics engine
      auto velOpt = payload.WorldLinearVelocity(_ecm);
      gz::math::Vector3d v = velOpt.has_value() ? velOpt.value() : gz::math::Vector3d::Zero;

      gz::math::Vector3d net_force(0.0, 0.0, 0.0);

      std::vector<double> current_L0(num_cables_);
      {
        std::lock_guard<std::mutex> lock(msg_mutex_);
        current_L0 = L0_;
      }

      for (size_t i = 0; i < num_cables_; ++i) {
        gz::math::Vector3d l_vec = anchors_[i] - p;
        double L_act = l_vec.Length();
        if (L_act < 1e-4) continue;

        // Normalized unit vector pointing from payload TO anchor
        gz::math::Vector3d u_vec = l_vec / L_act;

        // Cable rate of change: projection of velocity along the cable line
        // Moving TOWARDS anchor -> L_act shrinks (negative rate)
        // Moving AWAY from anchor -> L_act grows (positive rate)
        double L_dot = -u_vec.Dot(v);

        // Spring-damper tension (Kelvin-Voigt)
        double stretch = L_act - current_L0[i];
        double tension = (K_ * stretch) + (C_ * L_dot);

        // Cables only pull, never push
        if (tension < 0.0) tension = 0.0;
        if (tension > max_tension_) tension = max_tension_;

        net_force += u_vec * tension;

        // Visual cable pose update. Skip when the cable is near-vertical: the
        // payload above an anchor makes SetFrom2Axes degenerate (NaN).
        if (i < cable_models_.size() && cable_models_[i] != gz::sim::kNullEntity) {
          double uz = u_vec.Dot(gz::math::Vector3d::UnitZ);
          if (uz < 0.999 && uz > -0.999) {
            gz::math::Quaterniond rot;
            rot.SetFrom2Axes(gz::math::Vector3d::UnitZ, u_vec);
            gz::sim::Model cable_model(cable_models_[i]);
            cable_model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(p, rot));
          }
        }
      }

      // Payload viscous drag: cable-axis damping alone under-damps the soft
      // tangential mode near the workspace edge (zeta ~0.1 at the top corner),
      // so this damps the payload directly in all directions.
      net_force -= drag_ * v;

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