#include "CameraController.hpp"

#include <algorithm>
#include <cmath>

#include <raymath.h>

CameraController::CameraController()
{
    camera_.target = target_;
    camera_.up = {0.0f, 1.0f, 0.0f};
    camera_.fovy = 45.0f;
    camera_.projection = CAMERA_PERSPECTIVE;

    updateCameraPosition();
}

void CameraController::update(float deltaTime)
{
    float currentMoveSpeed = moveSpeed_;

    if (IsKeyDown(KEY_LEFT_SHIFT))
    {
        currentMoveSpeed *= 3.0f;
    }

    // Direction vers laquelle la caméra regarde,
    // projetée sur le sol XZ.
    Vector3 forward{
        -std::cos(yaw_),
        0.0f,
        -std::sin(yaw_)
    };

    forward = Vector3Normalize(forward);

    const Vector3 right =
        Vector3Normalize(
            Vector3CrossProduct(
                forward,
                {0.0f, 1.0f, 0.0f}
            )
        );

    // Translation sur le plan horizontal
    if (IsKeyDown(KEY_W))
    {
        target_ = Vector3Add(
            target_,
            Vector3Scale(forward, currentMoveSpeed * deltaTime)
        );
    }

    if (IsKeyDown(KEY_S))
    {
        target_ = Vector3Subtract(
            target_,
            Vector3Scale(forward, currentMoveSpeed * deltaTime)
        );
    }

    if (IsKeyDown(KEY_D))
    {
        target_ = Vector3Add(
            target_,
            Vector3Scale(right, currentMoveSpeed * deltaTime)
        );
    }

    if (IsKeyDown(KEY_A))
    {
        target_ = Vector3Subtract(
            target_,
            Vector3Scale(right, currentMoveSpeed * deltaTime)
        );
    }

    // Rotation horizontale
    if (IsKeyDown(KEY_Q))
    {
        yaw_ -= rotationSpeed_ * deltaTime;
    }

    if (IsKeyDown(KEY_E))
    {
        yaw_ += rotationSpeed_ * deltaTime;
    }

    // Inclinaison
    if (IsKeyDown(KEY_T))
    {
        pitch_ += pitchSpeed_ * deltaTime;
    }

    if (IsKeyDown(KEY_G))
    {
        pitch_ -= pitchSpeed_ * deltaTime;
    }

    pitch_ = std::clamp(
        pitch_,
        minPitch_,
        maxPitch_
    );

    // Déplacement vertical
    if (IsKeyDown(KEY_R))
    {
        target_.y += verticalSpeed_ * deltaTime;
    }

    if (IsKeyDown(KEY_F))
    {
        target_.y -= verticalSpeed_ * deltaTime;
    }

    // Zoom souris
    const float wheel = GetMouseWheelMove();

    if (wheel != 0.0f)
    {
        distance_ -= wheel * zoomSpeed_;

        distance_ = std::clamp(
            distance_,
            minDistance_,
            maxDistance_
        );
    }

    updateCameraPosition();
}

void CameraController::updateCameraPosition()
{
    const float horizontalDistance =
        distance_ * std::cos(pitch_);

    camera_.position.x =
        target_.x +
        horizontalDistance * std::cos(yaw_);

    camera_.position.y =
        target_.y +
        distance_ * std::sin(pitch_);

    camera_.position.z =
        target_.z +
        horizontalDistance * std::sin(yaw_);

    camera_.target = target_;
}

const Camera3D& CameraController::getCamera() const
{
    return camera_;
}