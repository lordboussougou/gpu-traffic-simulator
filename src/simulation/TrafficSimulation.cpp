#include "TrafficSimulation.hpp"

#include <algorithm>

TrafficSimulation::TrafficSimulation()
{
    constexpr int vehicleCount = 20;

    vehicles_.reserve(vehicleCount);

    for (int i = 0; i < vehicleCount; ++i)
    {
        Vehicle vehicle;

        vehicle.id = i;
        vehicle.position = static_cast<float>(i) * 8.0f;
        vehicle.speed = 0.0f;
        vehicle.acceleration = 2.0f;
        vehicle.lane = 0;
        vehicle.desiredSpeed = 15.0f;

        vehicles_.push_back(vehicle);
    }
}

void TrafficSimulation::update(float deltaTime)
{
    for (Vehicle& vehicle : vehicles_)
    {
        if (vehicle.speed < vehicle.desiredSpeed)
        {
            vehicle.speed += vehicle.acceleration * deltaTime;

            vehicle.speed =
                std::min(vehicle.speed, vehicle.desiredSpeed);
        }

        vehicle.position += vehicle.speed * deltaTime;

        if (vehicle.position > roadLength_)
        {
            vehicle.position -= roadLength_;
        }
    }
}

const std::vector<Vehicle>& TrafficSimulation::getVehicles() const
{
    return vehicles_;
}