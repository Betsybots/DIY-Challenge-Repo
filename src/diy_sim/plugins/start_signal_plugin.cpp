#include <gz/msgs/material.pb.h>

#include <cmath>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Visual.hh>
#include <vector>

class StartSignalPlugin : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
public:
    /// \brief Configure the plugin
    /// \param[in] _entity The entity this plugin is attached to.
    /// \param[in] _sdf SDF element containing configuration for this plugin.
    /// \param[in] _ecm Entity component manager.
    /// \param[in] _eventMgr The event manager.
    void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override {
        modelEntity = _entity;
        targetReached = false;
        targetAngle = 1.57;       // 90 degrees in radians
        rotationVelocity = 1.57;  // rad/s

        if (_sdf->HasElement("start_time")) {
            startTime = _sdf->Get<int>("start_time");
            gzmsg << "Using start time: " << startTime << " seconds" << std::endl;
        }

        // Find the revolute joint
        if (jointEntity == gz::sim::kNullEntity) {
            // Find all joints in the model
            auto jointEntities =
                _ecm.ChildrenByComponents(modelEntity, gz::sim::components::Joint());

            // Find the revolute joint by name
            for (const auto &entity : jointEntities) {
                auto nameComp = _ecm.Component<gz::sim::components::Name>(entity);
                if (nameComp && nameComp->Data() == "revolute_joint") {
                    jointEntity = entity;
                    gzmsg << "Found revolute joint with entity ID: " << entity << std::endl;
                    break;
                }
            }

            if (jointEntity == gz::sim::kNullEntity) {
                gzerr << "Revolute joint not found in model" << std::endl;
                return;
            }
        }
    }

    /// \brief Update the plugin
    /// \param[in] _info System update information.
    /// \param[in] _ecm Entity component manager.
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) override {
        // Skip if simulation is paused
        if (_info.paused)
            return;

        // Stop if the target has been reached
        if (targetReached)
            return;

        if (jointEntity == gz::sim::kNullEntity)
            return;

        // Create JointPosition component if it doesn't exist
        if (!_ecm.Component<gz::sim::components::JointPosition>(jointEntity)) {
            _ecm.CreateComponent(jointEntity, gz::sim::components::JointPosition({0}));
            gzmsg << "Created JointPosition component for revolute joint" << std::endl;
        }

        // Create JointVelocityCmd component if it doesn't exist
        if (!_ecm.Component<gz::sim::components::JointVelocityCmd>(jointEntity)) {
            _ecm.CreateComponent(jointEntity, gz::sim::components::JointVelocityCmd({0}));
            gzmsg << "Created JointVelocityCmd component for revolute joint" << std::endl;
        }

        auto vel = _ecm.Component<gz::sim::components::JointVelocityCmd>(jointEntity);

        // Don't  move the start signal until 10 seconds from the start of the sim.
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count();
        if (duration < startTime) {
            *vel = gz::sim::components::JointVelocityCmd({0.0});
            return;
        }

        // Check current position
        auto jointPosComp = _ecm.Component<gz::sim::components::JointPosition>(jointEntity);
        if (jointPosComp) {
            const auto &jointPos = jointPosComp->Data();
            if (!jointPos.empty()) {
                double currentPos = jointPos[0];
                gzmsg << "Current joint position: " << currentPos << std::endl;

                // If we're close enough to the target, stop
                if (std::abs(currentPos - targetAngle) < 0.05) {
                    // Set velocity to zero to stop movement
                    std::vector<double> zeroVel = {0.0};
                    *vel = gz::sim::components::JointVelocityCmd(zeroVel);

                    gzmsg << "Target position reached, stopping joint" << std::endl;
                    targetReached = true;
                    return;
                }
            }
        }

        // If target not reached, set velocity command to rotate
        std::vector<double> velocity = {rotationVelocity};
        *vel = gz::sim::components::JointVelocityCmd(velocity);

        if (_info.iterations % 100 == 0)
            gzmsg << "Rotating joint with velocity: " << rotationVelocity << " rad/s" << std::endl;
    }

private:
    gz::sim::Entity modelEntity;
    gz::sim::Entity jointEntity = gz::sim::kNullEntity;
    int startTime = 10;  // seconds. Time from the start of the simulation to move the start signal.
    bool targetReached = false;
    double targetAngle;
    double rotationVelocity;
};

GZ_ADD_PLUGIN(StartSignalPlugin, gz::sim::System, StartSignalPlugin::ISystemConfigure,
              StartSignalPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(StartSignalPlugin, "gz::sim::systems::StartSignalPlugin")
