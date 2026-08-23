#pragma once

#include <raylib.h>

class CameraController
{
public:
    CameraController();

    void update(float deltaTime);

    const Camera3D& getCamera() const;

private:
    void updateCameraPosition();

    Camera3D camera_{};

    Vector3 target_{100.0f, 0.0f, 0.0f};

    float yaw_ = 1.5708f;
    float pitch_ = 0.70f;
    float distance_ = 78.0f;

    float moveSpeed_ = 30.0f;
    float verticalSpeed_ = 20.0f;
    float rotationSpeed_ = 1.2f;
    float pitchSpeed_ = 0.8f;
    float zoomSpeed_ = 8.0f;

    float minDistance_ = 5.0f;
    float maxDistance_ = 250.0f;

    float minPitch_ = 0.15f;
    float maxPitch_ = 1.40f;
};