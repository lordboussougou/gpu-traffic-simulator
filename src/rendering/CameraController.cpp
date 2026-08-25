#include "CameraController.hpp"

#include <algorithm>
#include <cmath>

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
    const float adaptiveMoveSpeed = std::clamp(distance_ * 1.5f, 20.0f, 8000.0f);
    const float speedMultiplier = IsKeyDown(KEY_LEFT_SHIFT) ? 6.0f : 1.0f;
    const float moveSpeed = adaptiveMoveSpeed * speedMultiplier * deltaTime;

    const Vector3 forward{-std::cos(yaw_), 0.0f, -std::sin(yaw_)};
    const Vector3 right{-forward.z, 0.0f, forward.x};

    if (IsKeyDown(KEY_W))
    {
        target_.x += forward.x * moveSpeed;
        target_.z += forward.z * moveSpeed;
    }

    if (IsKeyDown(KEY_S))
    {
        target_.x -= forward.x * moveSpeed;
        target_.z -= forward.z * moveSpeed;
    }

    if (IsKeyDown(KEY_D))
    {
        target_.x += right.x * moveSpeed;
        target_.z += right.z * moveSpeed;
    }

    if (IsKeyDown(KEY_A))
    {
        target_.x -= right.x * moveSpeed;
        target_.z -= right.z * moveSpeed;
    }

    const float rotationSpeed = 1.25f * deltaTime;

    if (IsKeyDown(KEY_Q)) yaw_ += rotationSpeed;
    if (IsKeyDown(KEY_E)) yaw_ -= rotationSpeed;

    if (IsKeyDown(KEY_T)) pitch_ += rotationSpeed;
    if (IsKeyDown(KEY_G)) pitch_ -= rotationSpeed;

    pitch_ = std::clamp(pitch_, 0.15f, 1.35f);

    const float verticalSpeed = adaptiveMoveSpeed * 0.5f * deltaTime;

    if (IsKeyDown(KEY_R)) target_.y += verticalSpeed;
    if (IsKeyDown(KEY_F)) target_.y -= verticalSpeed;

    target_.y = std::max(target_.y, 0.0f);

    const float wheel = GetMouseWheelMove();

    if (wheel != 0.0f)
    {
        distance_ *= std::pow(0.78f, wheel);
        distance_ = std::clamp(distance_, minDistance_, maxDistance_);
    }

    clampTarget();
    updateCameraPosition();
}

void CameraController::updateCameraPosition()
{
    const float horizontalDistance = std::cos(pitch_) * distance_;

    camera_.position.x = target_.x + std::cos(yaw_) * horizontalDistance;
    camera_.position.y = target_.y + std::sin(pitch_) * distance_;
    camera_.position.z = target_.z + std::sin(yaw_) * horizontalDistance;

    camera_.target = target_;
}

void CameraController::clampTarget()
{
    if (!horizontalBoundsEnabled_) return;

    target_.x = std::clamp(target_.x, minX_, maxX_);
}

const Camera3D& CameraController::getCamera() const
{
    return camera_;
}

const Vector3& CameraController::getTarget() const
{
    return target_;
}

float CameraController::getDistance() const
{
    return distance_;
}

float CameraController::getRenderDistance() const
{
    return std::clamp(distance_ * 2.5f, 150.0f, 20000.0f);
}

void CameraController::setTarget(const Vector3& target)
{
    target_ = target;
    clampTarget();
    updateCameraPosition();
}

void CameraController::setHorizontalBounds(float minX, float maxX)
{
    minX_ = std::min(minX, maxX);
    maxX_ = std::max(minX, maxX);
    horizontalBoundsEnabled_ = true;

    clampTarget();
    updateCameraPosition();
}