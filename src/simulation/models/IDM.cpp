#include "IDM.hpp"

#include <algorithm>
#include <cmath>

IDM::IDM(const IDMParameters& parameters)
    : parameters_(parameters)
{
}

float IDM::computeAcceleration(
    float speed,
    float desiredSpeed,
    float leaderSpeed,
    float gap
) const
{
    // Δv > 0 signifie que notre véhicule
    // se rapproche du leader.
    const float deltaSpeed =
        speed - leaderSpeed;

    const float brakingTerm =
        (speed * deltaSpeed)
        /
        (
            2.0f *
            std::sqrt(
                parameters_.maxAcceleration *
                parameters_.comfortableBraking
            )
        );

    const float desiredGap =
        parameters_.minimumGap
        +
        std::max(
            0.0f,
            speed * parameters_.timeHeadway
            + brakingTerm
        );

    // Évite une division par zéro si deux véhicules
    // deviennent exceptionnellement très proches.
    const float safeGap =
        std::max(gap, 0.1f);

    const float freeRoadTerm =
        std::pow(
            speed / desiredSpeed,
            parameters_.accelerationExponent
        );

    const float interactionTerm =
        std::pow(
            desiredGap / safeGap,
            2.0f
        );

    return parameters_.maxAcceleration *
        (
            1.0f
            - freeRoadTerm
            - interactionTerm
        );
}