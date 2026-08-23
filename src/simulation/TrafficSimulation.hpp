#pragma once

#include "Vehicle.hpp"

#include <vector>

class TrafficSimulation
{
public:
    TrafficSimulation();

    void update(float deltaTime);

    const std::vector<Vehicle>& getVehicles() const;

private:
    std::vector<Vehicle> vehicles_;

    static constexpr float roadLength_ = 200.0f;
};