#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <raylib.h>
#include <string>

#include "rendering/CameraController.hpp"
#include "simulation/TrafficSimulation.hpp"

int main(int argc, char* argv[])
{
    std::cout << "GPU Traffic Simulator\n";
    std::cout << "=====================\n\n";

    std::size_t vehicleCount = 30;
    float metersPerVehicle = 30.0f;

    if (argc > 1)
    {
        try
        {
            vehicleCount = std::stoull(argv[1]);
        }
        catch (const std::exception&)
        {
            std::cerr << "Invalid vehicle count: " << argv[1] << '\n';
            return 1;
        }
    }

    if (argc > 2)
    {
        try
        {
            metersPerVehicle = std::stof(argv[2]);
        }
        catch (const std::exception&)
        {
            std::cerr << "Invalid spacing: " << argv[2] << '\n';
            return 1;
        }
    }

    if (vehicleCount == 0)
    {
        std::cerr << "Vehicle count must be greater than 0.\n";
        return 1;
    }

    if (metersPerVehicle <= 4.0f)
    {
        std::cerr << "Spacing must be greater than vehicle length (4 m).\n";
        return 1;
    }

    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "GPU Traffic Simulator - CUDA Benchmark");
    SetTargetFPS(60);

    CameraController cameraController;
    TrafficSimulation simulation(vehicleCount, metersPerVehicle);

    const float roadLength = simulation.getRoadLength();

    cameraController.setHorizontalBounds(0.0f, roadLength);
    cameraController.setTarget({std::min(100.0f, roadLength * 0.5f), 0.0f, 0.0f});

    std::size_t selectedVehicleIndex = 0;

    while (!WindowShouldClose())
    {
        const float deltaTime = GetFrameTime();

        cameraController.update(deltaTime);
        simulation.update(deltaTime);

        const auto& vehicles = simulation.getVehicles();

        const RoadNetwork& roadNetwork = simulation.getRoadNetwork();
        const auto& nodes = roadNetwork.getNodes();
        const auto& edges = roadNetwork.getEdges();

        if (!vehicles.empty())
        {
            if (IsKeyPressed(KEY_RIGHT))
                selectedVehicleIndex = (selectedVehicleIndex + 1) % vehicles.size();

            if (IsKeyPressed(KEY_LEFT))
                selectedVehicleIndex = (selectedVehicleIndex + vehicles.size() - 1) % vehicles.size();
        }


        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(cameraController.getCamera());

        constexpr float roadWidth = 8.0f;

        for (const RoadEdge& edge : edges)
        {
            const RoadNode& startNode = nodes[edge.startNodeIndex];
            const RoadNode& endNode = nodes[edge.endNodeIndex];

            const float centerX = (startNode.x + endNode.x) * 0.5f;
            const float centerZ = (startNode.z + endNode.z) * 0.5f;

            const float deltaX = std::abs(endNode.x - startNode.x);
            const float deltaZ = std::abs(endNode.z - startNode.z);

            const float width = deltaX > 0.0f ? deltaX : roadWidth;
            const float depth = deltaZ > 0.0f ? deltaZ : roadWidth;

            DrawCube({centerX, -0.05f, centerZ}, width, 0.1f, depth, DARKGRAY);
        }


        for (const RoadNode& node : nodes)
        {
            DrawCube({node.x, 0.0f, node.z}, roadWidth, 0.12f, roadWidth, GRAY);
        }


        std::size_t renderedVehicleCount = 0;

        const Vector3 cameraTarget = cameraController.getTarget();
        const float renderDistance = cameraController.getRenderDistance();
        const float renderDistanceSquared = renderDistance * renderDistance;

        for (std::size_t i = 0; i < vehicles.size(); ++i)
        {
            const Vehicle& vehicle = vehicles[i];

            const RoadPoint roadPoint = roadNetwork.getPointOnRoute(vehicle.position);

            const float deltaX = roadPoint.x - cameraTarget.x;
            const float deltaZ = roadPoint.z - cameraTarget.z;
            const float distanceSquared = deltaX * deltaX + deltaZ * deltaZ;

            if (distanceSquared > renderDistanceSquared)
                continue;

            Color vehicleColor = vehicle.desiredSpeed < 10.0f ? RED : BLUE;

            if (i == selectedVehicleIndex)
                vehicleColor = YELLOW;

            const Vector3 vehiclePosition{
                roadPoint.x,
                0.55f,
                roadPoint.z
            };

            DrawCube(vehiclePosition, 4.0f, 1.0f, 2.0f, vehicleColor);

            if (i == selectedVehicleIndex)
                DrawCubeWires(vehiclePosition, 4.0f, 1.0f, 2.0f, BLACK);

            ++renderedVehicleCount;
        }


        EndMode3D();

        const VehicleTelemetry telemetry = simulation.getVehicleTelemetry(selectedVehicleIndex);

        DrawText("GPU Traffic Simulator", 20, 20, 28, DARKGRAY);
        DrawText("CUDA leader search + IDM + vehicle update", 20, 58, 20, GRAY);

        DrawText(TextFormat("Vehicles: %zu", vehicles.size()), 20, 95, 18, DARKGRAY);
        DrawText(TextFormat("Visible vehicles: %zu / %zu", renderedVehicleCount, vehicles.size()), 20, 120, 18, DARKGRAY);        DrawText(TextFormat("Road length: %.0f m", roadLength), 20, 145, 18, DARKGRAY);
        DrawText(TextFormat("Spacing: %.1f m", metersPerVehicle), 20, 170, 18, DARKGRAY);

        DrawText(TextFormat("Kernel: %.4f ms", simulation.getLastKernelTimeMs()), 20, 205, 18, DARKGREEN);
        DrawText(TextFormat("CUDA update total: %.4f ms", simulation.getLastCudaUpdateTimeMs()), 20, 230, 18, DARKGREEN);
        
        DrawText(TextFormat("Camera X: %.0f m", cameraController.getTarget().x), 20, 285, 18, GRAY);
        DrawText(TextFormat("Camera distance: %.0f m", cameraController.getDistance()), 20, 310, 18, GRAY);
        DrawText("WASD Move | Shift Fast | Q/E Rotate | T/G Tilt | R/F Height | Wheel Zoom", 20, 680, 16, GRAY);

        DrawFPS(20, 260);

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