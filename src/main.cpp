#include "cuda/cuda_test.cuh"

#include <iostream>
#include <raylib.h>

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
    camera.position = {10.0f, 10.0f, 10.0f};
    camera.target = {0.0f, 0.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        DrawGrid(20, 1.0f);
        DrawCube({0.0f, 0.5f, 0.0f}, 1.0f, 1.0f, 1.0f, BLUE);
        DrawCubeWires({0.0f, 0.5f, 0.0f}, 1.0f, 1.0f, 1.0f, DARKBLUE);

        EndMode3D();

        DrawText("GPU Traffic Simulator", 20, 20, 28, DARKGRAY);
        DrawText("Step 1: C++ + CUDA + raylib", 20, 58, 20, GRAY);
        DrawText("CUDA test passed. ESC to quit.", 20, 88, 18, DARKGREEN);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
