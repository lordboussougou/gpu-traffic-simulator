#include "TrafficSimulation.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>


TrafficSimulation::TrafficSimulation(std::size_t vehicleCount, float metersPerVehicle)
    : roadNetwork_(RoadNetworkPresets::choose(vehicleCount))
{
    const auto& edges = roadNetwork_.getEdges();

    const std::size_t edgeCount = edges.size();
    const int laneCount = roadNetwork_.getConfig().lanesPerDirection;

    const std::size_t laneEdgeCount =
        edgeCount * static_cast<std::size_t>(laneCount);

    edgeLengths_.reserve(edgeCount);

    for (const RoadEdge& edge : edges)
        edgeLengths_.push_back(edge.length);

    vehicles_.reserve(vehicleCount);
    vehicleRoutes_.resize(vehicleCount);

    for (std::size_t i = 0; i < vehicleCount; ++i)
    {
        const std::size_t laneEdgeIndex = i % laneEdgeCount;
        const std::size_t slotIndex = i / laneEdgeCount;

        const std::size_t edgeIndex =
            laneEdgeIndex / static_cast<std::size_t>(laneCount);

        const int lane =
            static_cast<int>(laneEdgeIndex % static_cast<std::size_t>(laneCount));

        const std::size_t vehiclesOnLane =
            (vehicleCount + laneEdgeCount - 1 - laneEdgeIndex) / laneEdgeCount;

        Vehicle vehicle;

        vehicle.id = static_cast<int>(i);
        vehicle.currentEdgeId = static_cast<int>(edgeIndex);
        vehicle.nextEdgeId = -1;

        vehicle.lane = lane;

        vehicle.position =
            edges[edgeIndex].length *
            static_cast<float>(slotIndex + 1) /
            static_cast<float>(vehiclesOnLane + 1);

        vehicle.speed = 0.0f;
        vehicle.acceleration = 0.0f;

        vehicle.desiredSpeed =
            (i == vehicleCount / 2) ? 7.0f : 15.0f;

        vehicles_.push_back(vehicle);
    }

    for (std::size_t i = 0; i < vehicles_.size(); ++i)
        initializeRoute(i);

    updateRoutingState();
}

int TrafficSimulation::chooseDestinationNode(int vehicleId, int startNodeIndex, int tripNumber) const
{
    const int nodeCount = static_cast<int>(roadNetwork_.getNodes().size());

    if (nodeCount <= 1) return startNodeIndex;

    int destination =
        (vehicleId * 3 + tripNumber * 5 + 2) % nodeCount;

    if (destination == startNodeIndex)
        destination = (destination + 1) % nodeCount;

    return destination;
}

void TrafficSimulation::initializeRoute(std::size_t vehicleIndex)
{
    Vehicle& vehicle = vehicles_[vehicleIndex];
    VehicleRoute& route = vehicleRoutes_[vehicleIndex];

    const auto& edges = roadNetwork_.getEdges();
    const RoadEdge& currentEdge = edges[vehicle.currentEdgeId];

    const int startNodeIndex = currentEdge.endNodeIndex;
    const int destinationNodeIndex =
        chooseDestinationNode(vehicle.id, startNodeIndex, route.completedTrips);

    route.destinationNodeIndex = destinationNodeIndex;
    route.currentEdgePathIndex = 0;

    route.edgePath.clear();
    route.edgePath.push_back(vehicle.currentEdgeId);

    const std::vector<int> continuation =
        roadNetwork_.findShortestPath(startNodeIndex, destinationNodeIndex);

    route.edgePath.insert(route.edgePath.end(), continuation.begin(), continuation.end());

    preparePendingRoute(vehicleIndex);
}

void TrafficSimulation::preparePendingRoute(std::size_t vehicleIndex)
{
    Vehicle& vehicle = vehicles_[vehicleIndex];
    VehicleRoute& route = vehicleRoutes_[vehicleIndex];

    if (!route.pendingEdgePath.empty()) return;
    if (route.destinationNodeIndex < 0) return;

    const int nextTripNumber = route.completedTrips + 1;

    route.pendingDestinationNodeIndex =
        chooseDestinationNode(vehicle.id, route.destinationNodeIndex, nextTripNumber);

    route.pendingEdgePath =
        roadNetwork_.findShortestPath(route.destinationNodeIndex, route.pendingDestinationNodeIndex);
}

void TrafficSimulation::updateRoutingState()
{
    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        Vehicle& vehicle = vehicles_[i];
        VehicleRoute& route = vehicleRoutes_[i];

        if (route.currentEdgePathIndex + 1 < route.edgePath.size())
        {
            vehicle.nextEdgeId = route.edgePath[route.currentEdgePathIndex + 1];
            continue;
        }

        preparePendingRoute(i);

        if (!route.pendingEdgePath.empty())
            vehicle.nextEdgeId = route.pendingEdgePath.front();
        else
            vehicle.nextEdgeId = -1;
    }
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
    updateRoutingState();
}

void TrafficSimulation::updateRoadTransitions()
{
    const auto& edges = roadNetwork_.getEdges();

    for (std::size_t i = 0; i < vehicles_.size(); ++i)
    {
        Vehicle& vehicle = vehicles_[i];
        VehicleRoute& route = vehicleRoutes_[i];

        while (true)
        {
            const RoadEdge& currentEdge = edges[vehicle.currentEdgeId];

            if (vehicle.position < currentEdge.length) break;

            vehicle.position -= currentEdge.length;

            if (route.currentEdgePathIndex + 1 < route.edgePath.size())
            {
                ++route.currentEdgePathIndex;
                vehicle.currentEdgeId = route.edgePath[route.currentEdgePathIndex];
                continue;
            }

            ++route.completedTrips;

            if (route.pendingEdgePath.empty())
                preparePendingRoute(i);

            if (route.pendingEdgePath.empty())
            {
                vehicle.position = 0.0f;
                vehicle.speed = 0.0f;
                vehicle.acceleration = 0.0f;
                break;
            }

            route.edgePath = std::move(route.pendingEdgePath);
            route.destinationNodeIndex = route.pendingDestinationNodeIndex;

            route.pendingEdgePath.clear();
            route.pendingDestinationNodeIndex = -1;
            route.currentEdgePathIndex = 0;

            vehicle.currentEdgeId = route.edgePath.front();

            preparePendingRoute(i);
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
        if (candidate.lane != vehicle.lane) continue;

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
        if (candidate.lane != vehicle.lane) continue;

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
    const VehicleRoute& route = vehicleRoutes_[vehicleIndex];

    telemetry.vehicleId = vehicle.id;
    telemetry.edgeId = vehicle.currentEdgeId;
    telemetry.nextEdgeId = vehicle.nextEdgeId;
    telemetry.destinationNodeId = route.destinationNodeIndex;

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
    return roadNetwork_.getTotalRoadLength();
}

float TrafficSimulation::getLastKernelTimeMs() const
{
    return cudaVehicleUpdater_.getLastKernelTimeMs();
}

float TrafficSimulation::getLastCudaUpdateTimeMs() const
{
    return lastCudaUpdateTimeMs_;
}