#pragma once

struct IDMParameters
{
    float maxAcceleration = 2.0f;       // a   (m/s²)
    float comfortableBraking = 2.0f;    // b   (m/s²)
    float minimumGap = 2.0f;            // s0  (m)
    float timeHeadway = 1.5f;           // T   (s)
    float accelerationExponent = 4.0f;  // delta
};

class IDM
{
public:
    explicit IDM(
        const IDMParameters& parameters = {}
    );

    float computeAcceleration(
        float speed,
        float desiredSpeed,
        float leaderSpeed,
        float gap
    ) const;

private:
    IDMParameters parameters_;
};