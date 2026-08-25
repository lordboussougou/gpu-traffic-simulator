#pragma once

#include <raylib.h>

class CameraController
{
public:
    CameraController();

    void update(float deltaTime);

    const Camera3D& getCamera() const;
    const Vector3& getTarget() const;

    float getDistance() const;
    float getRenderDistance() const;

    void setTarget(const Vector3& target);
    void setHorizontalBounds(float minX, float maxX);

private:
    void updateCameraPosition();
    void clampTarget();

    Camera3D camera_{};
    Vector3 target_{};

    float yaw_ = -2.35f;
    float pitch_ = 0.65f;
    float distance_ = 60.0f;

    float minX_ = 0.0f;
    float maxX_ = 0.0f;
    bool horizontalBoundsEnabled_ = false;

    static constexpr float minDistance_ = 10.0f;
    static constexpr float maxDistance_ = 50000.0f;
};