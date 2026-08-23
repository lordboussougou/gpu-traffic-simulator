#include "cuda/cuda_test.cuh"

#include <iostream>
#include <raylib.h>

#include "simulation/TrafficSimulation.hpp"

int main()
{
    std::cout << "GPU Traffic Simulator\n";
    std::cout << "=====================\n\n";

    if (!runCudaTest())
    {
        std::cerr << "\nCUDA test failed. The 3D window will not start.\n";
        return 1;
    }

    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "GPU Traffic Simulator - Step 1");
    SetTargetFPS(60);

    Camera3D camera{};
    camera.position = {100.0f, 50.0f, 60.0f};
    camera.target = {100.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    TrafficSimulation simulation;

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);
        
        const float deltaTime = GetFrameTime();
        simulation.update(deltaTime);
        
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        DrawGrid(20, 1.0f);
        DrawPlane(
            {100.0f, 0.0f, 0.0f},
            {200.0f, 8.0f},
            DARKGRAY
        );
        DrawLine3D(
            {0.0f, 0.02f, -4.0f},
            {200.0f, 0.02f, -4.0f},
            WHITE
        );

        DrawLine3D(
            {0.0f, 0.02f, 4.0f},
            {200.0f, 0.02f, 4.0f},
            WHITE
        );

        for (const Vehicle& vehicle : simulation.getVehicles())
        {
            const Vector3 vehiclePosition{
                vehicle.position,
                0.5f,
                0.0f
            };

            DrawCube(
                vehiclePosition,
                4.0f,
                1.0f,
                2.0f,
                BLUE
            );

            DrawCubeWires(
                vehiclePosition,
                4.0f,
                1.0f,
                2.0f,
                DARKBLUE
            );
        }

        EndMode3D();

        DrawText("GPU Traffic Simulator", 20, 20, 28, DARKGRAY);
        DrawText("Step 2: CPU traffic simulation", 20, 58, 20, GRAY);
        DrawText("Mouse wheel: zoom | Middle mouse: move camera.", 20, 88, 18, DARKGRAY);
        DrawText("CUDA test passed. ESC to quit.", 20, 118, 18, DARKGREEN);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
