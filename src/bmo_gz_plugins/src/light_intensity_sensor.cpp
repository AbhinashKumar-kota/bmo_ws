/*
 * LightIntensitySensor — Implementation
 *
 * This file implements the world plugin that simulates tungsten lamp
 * illuminance sensors on Crazyflie drones. See the header file for
 * full documentation of the design and physics model.
 *
 * Key implementation details:
 * - Entity resolution is deferred: drone entities may not exist at Configure
 *   time (Gazebo spawns models asynchronously), so we resolve them lazily
 *   in PostUpdate on the first call.
 * - The plugin publishes at a fixed rate (default 10 Hz), not every physics
 *   step, to match real LDR sensor behavior and reduce topic bandwidth.
 */

#include "bmo_gz_plugins/light_intensity_sensor.hpp"

#include <cmath>
#include <sstream>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/World.hh>
#include <gz/common/Console.hh>

namespace bmo_gz_plugins
{

// ---------------------------------------------------------------------------
// Configure — called once when the plugin is loaded from SDF
// ---------------------------------------------------------------------------
void LightIntensitySensor::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  // --- Read drone model names ---
  // These are the model names used in <include><name>cf0</name></include>
  if (_sdf->HasElement("drone_models"))
  {
    std::string drone_names_str = _sdf->Get<std::string>("drone_models");
    std::istringstream iss(drone_names_str);
    std::string name;
    while (iss >> name)
    {
      DroneState ds;
      ds.name = name;
      ds.entity = gz::sim::kNullEntity;
      ds.entity_found = false;
      drones_.push_back(ds);
    }
    gzmsg << "[LightIntensitySensor] Tracking " << drones_.size()
          << " drones: " << drone_names_str << std::endl;
  }
  else
  {
    gzerr << "[LightIntensitySensor] Missing <drone_models> in SDF. "
          << "Plugin will not publish any data." << std::endl;
    return;
  }

  // --- Read lamp source positions ---
  // Format: "x1 y1 z1" for single source, or multiple <source_positions> elements
  // For single source world: one entry like "3.0 0.0 0.3"
  if (_sdf->HasElement("source_positions"))
  {
    auto elem = _sdf->FindElement("source_positions");
    while (elem)
    {
      std::string pos_str = elem->Get<std::string>();
      std::istringstream iss(pos_str);
      double x, y, z;
      if (iss >> x >> y >> z)
      {
        LampSource lamp;
        lamp.position = gz::math::Vector3d(x, y, z);
        lamp.power_watts = 60.0;  // default, overridden below if provided
        lamps_.push_back(lamp);
      }
      elem = elem->GetNextElement("source_positions");
    }
  }
  else
  {
    gzerr << "[LightIntensitySensor] Missing <source_positions> in SDF." << std::endl;
    return;
  }

  // --- Read lamp powers (one per source, same order) ---
  // Format: "60.0" or "60.0 40.0 100.0" for multiple sources
  if (_sdf->HasElement("source_powers"))
  {
    std::string powers_str = _sdf->Get<std::string>("source_powers");
    std::istringstream iss(powers_str);
    double power;
    size_t idx = 0;
    while (iss >> power && idx < lamps_.size())
    {
      lamps_[idx].power_watts = power;
      idx++;
    }
  }

  // --- Read optional parameters with defaults ---
  if (_sdf->HasElement("luminous_efficacy"))
    luminous_efficacy_ = _sdf->Get<double>("luminous_efficacy");

  if (_sdf->HasElement("noise_stddev"))
    noise_stddev_ = _sdf->Get<double>("noise_stddev");

  if (_sdf->HasElement("max_reading"))
    max_reading_ = _sdf->Get<double>("max_reading");

  if (_sdf->HasElement("update_rate"))
    update_rate_ = _sdf->Get<double>("update_rate");

  // --- Compute update period from rate ---
  double period_sec = 1.0 / update_rate_;
  update_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(period_sec));

  // --- Initialize noise generator ---
  rng_ = std::default_random_engine(std::random_device{}());
  noise_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);

  // --- Create Gazebo transport publishers for each drone ---
  for (auto &drone : drones_)
  {
    std::string topic = "/model/" + drone.name + "/light_intensity";
    drone.publisher = node_.Advertise<gz::msgs::Double>(topic);
    gzmsg << "[LightIntensitySensor] Publishing on: " << topic << std::endl;
  }

  // --- Log lamp configuration ---
  for (size_t i = 0; i < lamps_.size(); i++)
  {
    gzmsg << "[LightIntensitySensor] Lamp " << i
          << ": pos=(" << lamps_[i].position.X()
          << ", " << lamps_[i].position.Y()
          << ", " << lamps_[i].position.Z()
          << "), power=" << lamps_[i].power_watts << "W" << std::endl;
  }

  gzmsg << "[LightIntensitySensor] Config: efficacy=" << luminous_efficacy_
        << " lm/W, noise_stddev=" << noise_stddev_
        << " lux, max_reading=" << max_reading_
        << " lux, update_rate=" << update_rate_ << " Hz" << std::endl;
}

// ---------------------------------------------------------------------------
// PostUpdate — called every physics step (after physics)
// ---------------------------------------------------------------------------
void LightIntensitySensor::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // Don't run if simulation is paused
  if (_info.paused)
    return;

  // --- Throttle to update_rate ---
  auto elapsed = _info.simTime - last_update_time_;
  if (elapsed < update_period_)
    return;
  last_update_time_ = _info.simTime;

  // --- Lazy entity resolution ---
  // Drone models might not exist yet at Configure time because Gazebo
  // spawns <include> models asynchronously. We resolve them here on the
  // first PostUpdate calls until all are found.
  for (auto &drone : drones_)
  {
    if (drone.entity_found)
      continue;

    // Search for a model entity with matching name
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool
        {
          if (_name->Data() == drone.name)
          {
            drone.entity = _entity;
            drone.entity_found = true;
            gzmsg << "[LightIntensitySensor] Resolved entity for drone '"
                  << drone.name << "' (id=" << _entity << ")" << std::endl;
            return false;  // stop iterating
          }
          return true;  // continue
        });
  }

  // --- Compute and publish intensity for each drone ---
  for (auto &drone : drones_)
  {
    if (!drone.entity_found)
      continue;

    // Get the drone's world pose from the ECM
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(drone.entity);
    if (!poseComp)
      continue;

    gz::math::Vector3d drone_pos = poseComp->Data().Pos();

    // Compute total illuminance from all lamp sources
    double illuminance = ComputeIlluminance(drone_pos);

    // Add Gaussian noise to simulate real LDR sensor
    double noisy_reading = illuminance + noise_dist_(rng_);

    // Clamp to valid range [0, max_reading]
    noisy_reading = std::max(0.0, std::min(noisy_reading, max_reading_));

    // Publish the reading
    gz::msgs::Double msg;
    msg.set_data(noisy_reading);
    drone.publisher.Publish(msg);
  }
}

// ---------------------------------------------------------------------------
// ComputeIlluminance — inverse-square law for all lamp sources
//
// Physics: For an isotropic point source (tungsten lamp), illuminance at
// distance r is:
//   I(r) = (P * eta) / (4 * pi * r^2)       [BMO Paper context]
//
// For multiple sources, total illuminance is the superposition:
//   I_total = sum_k [ (P_k * eta) / (4 * pi * ||p_k - p_drone||^2) ]
//
// Parameters:
//   _drone_pos : world position of the drone (meters)
// Returns:
//   Total illuminance in lux (before noise)
// ---------------------------------------------------------------------------
double LightIntensitySensor::ComputeIlluminance(
    const gz::math::Vector3d &_drone_pos) const
{
  // Minimum distance to prevent division by zero when drone is at lamp position.
  // 0.01m = 1cm, smaller than any real approach distance.
  constexpr double EPSILON = 0.01;
  constexpr double FOUR_PI = 4.0 * GZ_PI;

  double total = 0.0;

  for (const auto &lamp : lamps_)
  {
    double dist = lamp.position.Distance(_drone_pos);

    // Clamp distance to avoid singularity at r=0
    if (dist < EPSILON)
      dist = EPSILON;

    // Inverse-square law: I = (P * eta) / (4 * pi * r^2)
    double illuminance = (lamp.power_watts * luminous_efficacy_) / (FOUR_PI * dist * dist);
    total += illuminance;
  }

  return total;
}

}  // namespace bmo_gz_plugins

// ---------------------------------------------------------------------------
// Register the plugin with Gazebo so it can be loaded from SDF.
// The first argument is the class, the second is the base class (System).
// ---------------------------------------------------------------------------
GZ_ADD_PLUGIN(
    bmo_gz_plugins::LightIntensitySensor,
    gz::sim::System,
    bmo_gz_plugins::LightIntensitySensor::ISystemConfigure,
    bmo_gz_plugins::LightIntensitySensor::ISystemPostUpdate)
