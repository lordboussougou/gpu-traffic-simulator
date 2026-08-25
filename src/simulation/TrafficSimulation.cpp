#include "TrafficSimulation.hpp"

#include <algorithm>
#include <limits>

TrafficSimulation::TrafficSimulation()
{
    constexpr int vehicleCount = 12;
    vehicles_.reserve(vehicleCount);

    for (int i = 0; i < vehicleCount; ++i)
    {
        Vehicle vehicle;
        vehicle.id = i;
        vehicle.position = (roadLength_ / static_cast<float>(vehicleCount)) * static_cast<float>(i);
        vehicle.speed = 0.0f;
        vehicle.acceleration = 0.0f;
        vehicle.lane = 0;
        vehicle.desiredSpeed = (i == 6) ? 7.0f : 15.0f;
        vehicles_.push_back(vehicle);
    }
}

void TrafficSimulation::update(float deltaTime)
{
    if (vehicles_.empty()) return;

    std::vector<float> accelerations(vehicles_.size(), 0.0f);

    // Phase 1 : calcul IDM à partir de l'état courant.
    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        const Vehicle& vehicle = vehicles_[i];

        if (vehicles_.size() == 1)
        {
            accelerations[i] = idm_.computeAcceleration(vehicle.speed, vehicle.desiredSpeed, vehicle.speed, roadLength_);
            continue;
        }

        const std::size_t leaderIndex = findLeaderIndex(i);
        const Vehicle& leader = vehicles_[leaderIndex];
        const float centerDistance = distanceAhead(vehicle, leader);
        const float gap = std::max(centerDistance - vehicleLength_, 0.1f);

        accelerations[i] = idm_.computeAcceleration(vehicle.speed, vehicle.desiredSpeed, leader.speed, gap);
    }

    // Phase 2 : application des accélérations et mise à jour de l'état.
    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        Vehicle& vehicle = vehicles_[i];
        vehicle.acceleration = accelerations[i];
        vehicle.speed += vehicle.acceleration * deltaTime;
        vehicle.speed = std::max(vehicle.speed, 0.0f);
        vehicle.position += vehicle.speed * deltaTime;

        if (vehicle.position >= roadLength_) vehicle.position -= roadLength_;
    }
}

std::size_t TrafficSimulation::findLeaderIndex(std::size_t vehicleIndex) const
{
    const Vehicle& vehicle = vehicles_[vehicleIndex];
    std::size_t closestIndex = vehicleIndex;
    float closestDistance = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        if (i == vehicleIndex) continue;

        const float distance = distanceAhead(vehicle, vehicles_[i]);
        if (distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    return closestIndex;
}

float TrafficSimulation::distanceAhead(const Vehicle& vehicle, const Vehicle& leader) const
{
    float distance = leader.position - vehicle.position;
    if (distance <= 0.0f) distance += roadLength_;
    return distance;
}

const std::vector<Vehicle>& TrafficSimulation::getVehicles() const
{
    return vehicles_;
}

VehicleTelemetry TrafficSimulation::getVehicleTelemetry(std::size_t vehicleIndex) const
{
    VehicleTelemetry telemetry{};
    if (vehicleIndex >= vehicles_.size()) return telemetry;

    const Vehicle& vehicle = vehicles_[vehicleIndex];
    telemetry.vehicleId = vehicle.id;
    telemetry.speed = vehicle.speed;
    telemetry.desiredSpeed = vehicle.desiredSpeed;
    telemetry.acceleration = vehicle.acceleration;

    if (vehicles_.size() == 1)
    {
        telemetry.leaderId = -1;
        telemetry.gap = roadLength_;
        return telemetry;
    }

    const std::size_t leaderIndex = findLeaderIndex(vehicleIndex);
    const Vehicle& leader = vehicles_[leaderIndex];
    telemetry.leaderId = leader.id;
    telemetry.gap = std::max(distanceAhead(vehicle, leader) - vehicleLength_, 0.1f);

    return telemetry;
}