#include "TrafficSimulation.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace
{
float calculateNetworkLength(std::size_t vehicleCount, float metersPerVehicle)
{
    constexpr float vehicleLength = 4.0f;

    const float spacing = std::max(metersPerVehicle, vehicleLength + 0.1f);
    const std::size_t safeVehicleCount = std::max<std::size_t>(vehicleCount, 1);

    return static_cast<float>(safeVehicleCount) * spacing;
}
}

TrafficSimulation::TrafficSimulation(std::size_t vehicleCount, float metersPerVehicle)
    : roadNetwork_(calculateNetworkLength(vehicleCount, metersPerVehicle))
{
    const auto& edges = roadNetwork_.getEdges();
    edgeLengths_.reserve(edges.size());

    for (const RoadEdge& edge : edges)
        edgeLengths_.push_back(edge.length);
    
    const std::size_t edgeCount = edges.size();

    vehicles_.reserve(vehicleCount);

    for (std::size_t i = 0; i < vehicleCount; ++i)
    {
        const std::size_t edgeIndex = i % edgeCount;
        const std::size_t slotIndex = i / edgeCount;

        const std::size_t vehiclesOnEdge =
            (vehicleCount + edgeCount - 1 - edgeIndex) / edgeCount;

        Vehicle vehicle;

        vehicle.id = static_cast<int>(i);
        vehicle.currentEdgeId = static_cast<int>(edgeIndex);
        vehicle.nextEdgeId = roadNetwork_.chooseNextEdge(vehicle.currentEdgeId, vehicle.id);

        vehicle.position =
            edges[edgeIndex].length *
            static_cast<float>(slotIndex + 1) /
            static_cast<float>(vehiclesOnEdge + 1);

        vehicle.speed = 0.0f;
        vehicle.acceleration = 0.0f;
        vehicle.lane = 0;
        vehicle.desiredSpeed = (i == vehicleCount / 2) ? 7.0f : 15.0f;

        vehicles_.push_back(vehicle);
    }
}

void TrafficSimulation::updateRoutingState()
{
    for (Vehicle& vehicle : vehicles_)
        vehicle.nextEdgeId = roadNetwork_.chooseNextEdge(vehicle.currentEdgeId, vehicle.id);
}

void TrafficSimulation::update(float deltaTime)
{
    if (vehicles_.empty()) return;

    updateRoutingState();

    const auto start = std::chrono::high_resolution_clock::now();

    const bool success =
        cudaVehicleUpdater_.update(vehicles_, edgeLengths_, deltaTime, vehicleLength_, idm_.getParameters());

    const auto end = std::chrono::high_resolution_clock::now();

    lastCudaUpdateTimeMs_ =
        std::chrono::duration<float, std::milli>(end - start).count();

    if (!success) throw std::runtime_error("CUDA vehicle update failed.");

    updateRoadTransitions();
}

void TrafficSimulation::updateRoadTransitions()
{
    const auto& edges = roadNetwork_.getEdges();

    for (Vehicle& vehicle : vehicles_)
    {
        while (true)
        {
            const RoadEdge& currentEdge = edges[vehicle.currentEdgeId];

            if (vehicle.position < currentEdge.length) break;

            vehicle.position -= currentEdge.length;

            const int nextEdgeId = roadNetwork_.chooseNextEdge(vehicle.currentEdgeId, vehicle.id);

            if (nextEdgeId < 0)
            {
                vehicle.position = currentEdge.length;
                vehicle.speed = 0.0f;
                vehicle.acceleration = 0.0f;
                break;
            }

            vehicle.currentEdgeId = nextEdgeId;
            vehicle.nextEdgeId = roadNetwork_.chooseNextEdge(vehicle.currentEdgeId, vehicle.id);
        }
    }
}

std::size_t TrafficSimulation::findLeaderIndexForTelemetry(std::size_t vehicleIndex) const
{
    const Vehicle& vehicle = vehicles_[vehicleIndex];

    std::size_t closestIndex = vehicles_.size();
    float closestDistance = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        if (i == vehicleIndex) continue;

        const Vehicle& candidate = vehicles_[i];

        if (candidate.currentEdgeId != vehicle.currentEdgeId) continue;

        const float distance = candidate.position - vehicle.position;

        if (distance > 0.0f && distance < closestDistance)
        {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    if (closestIndex != vehicles_.size()) return closestIndex;

    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        if (i == vehicleIndex) continue;

        const Vehicle& candidate = vehicles_[i];

        if (candidate.currentEdgeId != vehicle.nextEdgeId) continue;

        if (candidate.position < closestDistance)
        {
            closestDistance = candidate.position;
            closestIndex = i;
        }
    }

    return closestIndex;
}

const std::vector<Vehicle>& TrafficSimulation::getVehicles() const
{
    return vehicles_;
}

const RoadNetwork& TrafficSimulation::getRoadNetwork() const
{
    return roadNetwork_;
}

VehicleTelemetry TrafficSimulation::getVehicleTelemetry(std::size_t vehicleIndex) const
{
    VehicleTelemetry telemetry{};

    if (vehicleIndex >= vehicles_.size()) return telemetry;

    const Vehicle& vehicle = vehicles_[vehicleIndex];

    telemetry.vehicleId = vehicle.id;
    telemetry.edgeId = vehicle.currentEdgeId;
    telemetry.speed = vehicle.speed;
    telemetry.desiredSpeed = vehicle.desiredSpeed;
    telemetry.acceleration = vehicle.acceleration;

    const std::size_t leaderIndex = findLeaderIndexForTelemetry(vehicleIndex);

    if (leaderIndex == vehicles_.size()) return telemetry;

    const Vehicle& leader = vehicles_[leaderIndex];

    telemetry.leaderId = leader.id;

    float centerDistance = 0.0f;

    if (leader.currentEdgeId == vehicle.currentEdgeId)
    {
        centerDistance = leader.position - vehicle.position;
    }
    else
    {
        const RoadEdge& currentEdge = roadNetwork_.getEdges()[vehicle.currentEdgeId];

        centerDistance =
            (currentEdge.length - vehicle.position)
            + leader.position;
    }

    telemetry.gap = std::max(centerDistance - vehicleLength_, 0.1f);

    return telemetry;
}

float TrafficSimulation::getRoadLength() const
{
    return roadNetwork_.getReferenceLoopLength();
}

float TrafficSimulation::getLastKernelTimeMs() const
{
    return cudaVehicleUpdater_.getLastKernelTimeMs();
}

float TrafficSimulation::getLastCudaUpdateTimeMs() const
{
    return lastCudaUpdateTimeMs_;
}