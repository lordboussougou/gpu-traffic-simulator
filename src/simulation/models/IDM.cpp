#include "IDM.hpp"

#include <algorithm>
#include <cmath>

IDM::IDM(const IDMParameters& parameters)
    : parameters_(parameters)
{
}

float IDM::computeAcceleration(float speed, float desiredSpeed, float leaderSpeed, float gap) const
{
    const float deltaSpeed = speed - leaderSpeed;
    const float brakingTerm =
        (speed * deltaSpeed) / (2.0f * std::sqrt(parameters_.maxAcceleration * parameters_.comfortableBraking));

    const float desiredGap =
        parameters_.minimumGap + std::max(0.0f, speed * parameters_.timeHeadway + brakingTerm);

    const float safeGap = std::max(gap, 0.1f);
    const float safeDesiredSpeed = std::max(desiredSpeed, 0.1f);
    const float freeRoadTerm = std::pow(speed / safeDesiredSpeed, parameters_.accelerationExponent);
    const float interactionTerm = std::pow(desiredGap / safeGap, 2.0f);

    return parameters_.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

const IDMParameters& IDM::getParameters() const
{
    return parameters_;
}
