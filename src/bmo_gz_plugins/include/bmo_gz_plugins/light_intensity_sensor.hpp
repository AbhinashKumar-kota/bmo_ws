/*
 * LightIntensitySensor — Gazebo Harmonic World Plugin
 *
 * Purpose:
 *   Simulates omnidirectional light intensity sensors on Crazyflie drones.
 *   Models tungsten lamp signal sources using the inverse-square law.
 *   Each drone gets a sensor reading published on a Gazebo transport topic.
 *
 * How it works:
 *   - Attached as a WORLD plugin (one instance for the entire simulation)
 *   - On Configure: reads lamp positions/powers and drone model names from SDF
 *   - On PostUpdate (every physics step, throttled to update_rate):
 *       1. Gets each drone's world pose from the Entity Component Manager
 *       2. Computes total illuminance from all lamps: I = sum(P*eta / 4*pi*r^2)
 *       3. Adds Gaussian noise to simulate real LDR sensor behavior
 *       4. Publishes gz.msgs.Double on /model/{drone_name}/light_intensity
 *
 * Physics reference:
 *   Inverse-square law for isotropic point source:
 *     I(r) = (P * eta) / (4 * pi * r^2)
 *   where P = lamp power (W), eta = luminous efficacy (lm/W), r = distance (m)
 *   For tungsten at ~2700K, eta ≈ 15 lm/W.
 *
 * SDF configuration example:
 *   <plugin filename="LightIntensitySensor"
 *          name="bmo_gz_plugins::LightIntensitySensor">
 *     <drone_models>cf0 cf1 cf2 cf3</drone_models>
 *     <source_positions>3.0 0.0 0.3</source_positions>
 *     <source_powers>60.0</source_powers>
 *     <luminous_efficacy>15.0</luminous_efficacy>
 *     <noise_stddev>0.5</noise_stddev>
 *     <max_reading>10000.0</max_reading>
 *     <update_rate>10.0</update_rate>
 *   </plugin>
 */

#ifndef BMO_GZ_PLUGINS_LIGHT_INTENSITY_SENSOR_HPP
#define BMO_GZ_PLUGINS_LIGHT_INTENSITY_SENSOR_HPP

#include <memory>
#include <string>
#include <vector>
#include <random>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>

namespace bmo_gz_plugins
{

/// \brief Represents a single tungsten lamp signal source in the world.
struct LampSource
{
  gz::math::Vector3d position;  ///< World position of the lamp (meters)
  double power_watts;            ///< Electrical power (watts), e.g. 60W
};

/// \brief Per-drone state: entity ID, name, and Gazebo publisher.
struct DroneState
{
  gz::sim::Entity entity;                        ///< Gazebo entity ID
  std::string name;                               ///< Model name (e.g. "cf0")
  gz::transport::Node::Publisher publisher;       ///< Publishes light intensity
  bool entity_found;                              ///< Whether entity was resolved
};

/// \brief World plugin that computes light intensity for each drone.
class LightIntensitySensor
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate
{
public:
  LightIntensitySensor() = default;
  ~LightIntensitySensor() override = default;

  /// \brief Called once when the plugin is loaded. Reads SDF parameters.
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  /// \brief Called every physics step (after physics update).
  /// Computes illuminance for each drone and publishes the reading.
  void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

private:
  /// \brief Compute total illuminance at a point from all lamp sources.
  /// Uses inverse-square law: I = (P * eta) / (4 * pi * r^2)
  /// \param _drone_pos World position of the drone
  /// \return Total illuminance in lux (before noise)
  double ComputeIlluminance(const gz::math::Vector3d &_drone_pos) const;

  // --- Configuration (read from SDF) ---
  std::vector<LampSource> lamps_;          ///< All tungsten lamp sources
  std::vector<DroneState> drones_;         ///< All drone models to track
  double luminous_efficacy_ = 15.0;        ///< lm/W, 15 for tungsten at 2700K
  double noise_stddev_ = 0.5;             ///< Gaussian noise std dev (lux)
  double max_reading_ = 10000.0;          ///< Sensor saturation limit (lux)
  double update_rate_ = 10.0;            ///< Publish rate (Hz)

  // --- Runtime state ---
  gz::transport::Node node_;               ///< Gazebo transport node
  std::default_random_engine rng_;         ///< RNG for sensor noise
  std::normal_distribution<double> noise_dist_;  ///< Gaussian noise distribution
  std::chrono::steady_clock::duration update_period_;  ///< Period between updates
  std::chrono::steady_clock::duration last_update_time_{0};  ///< Last publish time
};

}  // namespace bmo_gz_plugins

#endif  // BMO_GZ_PLUGINS_LIGHT_INTENSITY_SENSOR_HPP
