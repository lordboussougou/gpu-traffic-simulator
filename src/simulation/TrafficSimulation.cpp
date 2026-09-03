#include "TrafficSimulation.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace
{
float calculateRoadLength(std::size_t vehicleCount, float metersPerVehicle)
{
    constexpr float vehicleLength = 4.0f;

    const float spacing = std::max(metersPerVehicle, vehicleLength + 0.1f);
    const std::size_t safeVehicleCount = std::max<std::size_t>(vehicleCount, 1);

    return static_cast<float>(safeVehicleCount) * spacing;
}
}

TrafficSimulation::TrafficSimulation(std::size_t vehicleCount, float metersPerVehicle)
    : roadNetwork_(calculateRoadLength(vehicleCount, metersPerVehicle))
{
    const float spacing = std::max(metersPerVehicle, vehicleLength_ + 0.1f);

    roadLength_ = roadNetwork_.getRouteLength();

    vehicles_.reserve(vehicleCount);

    for (std::size_t i = 0; i < vehicleCount; ++i)
    {
        Vehicle vehicle;

        vehicle.id = static_cast<int>(i);
        vehicle.position = static_cast<float>(i) * spacing;
        vehicle.speed = 0.0f;
        vehicle.acceleration = 0.0f;
        vehicle.lane = 0;
        vehicle.desiredSpeed = (i == vehicleCount / 2) ? 7.0f : 15.0f;

        vehicles_.push_back(vehicle);
    }
}

void TrafficSimulation::update(float deltaTime)
{
    if (vehicles_.empty()) return;

    const auto start = std::chrono::high_resolution_clock::now();

    const bool success =
        cudaVehicleUpdater_.update(vehicles_, deltaTime, roadLength_, vehicleLength_, idm_.getParameters());

    const auto end = std::chrono::high_resolution_clock::now();

    lastCudaUpdateTimeMs_ =
        std::chrono::duration<float, std::milli>(end - start).count();

    if (!success) throw std::runtime_error("CUDA vehicle update failed.");
}

std::size_t TrafficSimulation::findLeaderIndexForTelemetry(std::size_t vehicleIndex) const
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

const RoadNetwork& TrafficSimulation::getRoadNetwork() const
{
    return roadNetwork_;
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

    const std::size_t leaderIndex = findLeaderIndexForTelemetry(vehicleIndex);
    const Vehicle& leader = vehicles_[leaderIndex];

    telemetry.leaderId = leader.id;
    telemetry.gap = std::max(distanceAhead(vehicle, leader) - vehicleLength_, 0.1f);

    return telemetry;
}

float TrafficSimulation::getRoadLength() const
{
    return roadLength_;
}

float TrafficSimulation::getLastKernelTimeMs() const
{
    return cudaVehicleUpdater_.getLastKernelTimeMs();
}

float TrafficSimulation::getLastCudaUpdateTimeMs() const
{
    return lastCudaUpdateTimeMs_;
}