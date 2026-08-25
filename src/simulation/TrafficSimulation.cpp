#include "TrafficSimulation.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

TrafficSimulation::TrafficSimulation()
{
    constexpr int vehicleCount = 12;

    vehicles_.reserve(vehicleCount);
    leaderIndices_.resize(vehicleCount);

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

    if (leaderIndices_.size() != vehicles_.size()) leaderIndices_.resize(vehicles_.size());

    // Phase CPU : pour l'instant, la recherche du leader reste séquentielle.
    if (vehicles_.size() == 1)
    {
        leaderIndices_[0] = 0;
    }
    else
    {
        for (std::size_t i = 0; i < vehicles_.size(); ++i)
            leaderIndices_[i] = static_cast<int>(findLeaderIndex(i));
    }

    // Phase GPU : IDM + accélération + vitesse + position, un thread CUDA par véhicule.
    const bool success = cudaVehicleUpdater_.update(
        vehicles_, leaderIndices_, deltaTime, roadLength_, vehicleLength_, idm_.getParameters()
    );

    if (!success) throw std::runtime_error("CUDA vehicle update failed.");
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
