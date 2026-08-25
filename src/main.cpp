#include <cstddef>
#include <iostream>
#include <raylib.h>

#include "rendering/CameraController.hpp"
#include "simulation/TrafficSimulation.hpp"

int main()
{
    std::cout << "GPU Traffic Simulator\n";
    std::cout << "=====================\n\n";

    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "GPU Traffic Simulator - Step 3");
    SetTargetFPS(60);

    CameraController cameraController;
    TrafficSimulation simulation;
    std::size_t selectedVehicleIndex = 0;

    while (!WindowShouldClose())
    {
        const float deltaTime = GetFrameTime();

        cameraController.update(deltaTime);
        simulation.update(deltaTime);

        const std::size_t vehicleCount = simulation.getVehicles().size();

        if (vehicleCount > 0)
        {
            if (IsKeyPressed(KEY_RIGHT)) selectedVehicleIndex = (selectedVehicleIndex + 1) % vehicleCount;
            if (IsKeyPressed(KEY_LEFT)) selectedVehicleIndex = (selectedVehicleIndex + vehicleCount - 1) % vehicleCount;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(cameraController.getCamera());

        DrawPlane({100.0f, 0.0f, 0.0f}, {200.0f, 8.0f}, DARKGRAY);
        DrawLine3D({0.0f, 0.02f, -4.0f}, {200.0f, 0.02f, -4.0f}, WHITE);
        DrawLine3D({0.0f, 0.02f, 4.0f}, {200.0f, 0.02f, 4.0f}, WHITE);

        const auto& vehicles = simulation.getVehicles();

        for (std::size_t i = 0; i < vehicles.size(); ++i)
        {
            const Vehicle& vehicle = vehicles[i];
            Color vehicleColor = vehicle.desiredSpeed < 10.0f ? RED : BLUE;
            if (i == selectedVehicleIndex) vehicleColor = YELLOW;

            const Vector3 vehiclePosition{vehicle.position, 0.5f, 0.0f};
            DrawCube(vehiclePosition, 4.0f, 1.0f, 2.0f, vehicleColor);
            DrawCubeWires(vehiclePosition, 4.0f, 1.0f, 2.0f, DARKBLUE);
        }

        EndMode3D();

        const VehicleTelemetry telemetry = simulation.getVehicleTelemetry(selectedVehicleIndex);

        DrawText("GPU Traffic Simulator", 20, 20, 28, DARKGRAY);
        DrawText("Step 3: CUDA vehicle update + IDM", 20, 58, 20, GRAY);
        DrawText("WASD Move | Q/E Rotate | T/G Tilt | R/F Height | Wheel Zoom | Shift Fast", 20, 88, 18, DARKGRAY);
        DrawText("Left/Right Arrows: select vehicle", 20, 118, 18, DARKGRAY);
        DrawText("Vehicle update: GPU (CUDA) | Leader search: CPU", 20, 148, 18, DARKGREEN);

        DrawRectangle(screenWidth - 300, 20, 280, 190, Fade(BLACK, 0.75f));
        DrawText(TextFormat("Vehicle #%d", telemetry.vehicleId), screenWidth - 280, 35, 22, WHITE);
        DrawText(TextFormat("Speed: %.2f m/s", telemetry.speed), screenWidth - 280, 70, 18, WHITE);
        DrawText(TextFormat("Desired: %.2f m/s", telemetry.desiredSpeed), screenWidth - 280, 95, 18, WHITE);
        DrawText(TextFormat("Acceleration: %.2f m/s2", telemetry.acceleration), screenWidth - 280, 120, 18, WHITE);
        DrawText(TextFormat("Leader: #%d", telemetry.leaderId), screenWidth - 280, 145, 18, WHITE);
        DrawText(TextFormat("Gap: %.2f m", telemetry.gap), screenWidth - 280, 170, 18, WHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
