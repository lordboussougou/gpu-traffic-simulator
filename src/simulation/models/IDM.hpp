#pragma once

struct IDMParameters
{
    float maxAcceleration = 2.0f;
    float comfortableBraking = 2.0f;
    float minimumGap = 2.0f;
    float timeHeadway = 1.5f;
    float accelerationExponent = 4.0f;
};

class IDM
{
public:
    explicit IDM(const IDMParameters& parameters = {});

    float computeAcceleration(float speed, float desiredSpeed, float leaderSpeed, float gap) const;
    const IDMParameters& getParameters() const;

private:
    IDMParameters parameters_;
};
