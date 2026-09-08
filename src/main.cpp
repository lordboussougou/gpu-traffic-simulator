#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <raylib.h>
#include <string>

#include "rendering/CameraController.hpp"
#include "simulation/TrafficSimulation.hpp"
#include "simulation/road/RoadNetwork.hpp"

void drawOffsetLine(const RoadNode& startNode, const RoadNode& endNode, float offset, Color color)
{
    const float deltaX = endNode.x - startNode.x;
    const float deltaZ = endNode.z - startNode.z;
    const float length = std::sqrt(deltaX * deltaX + deltaZ * deltaZ);

    if (length <= 0.0f) return;

    const float directionX = deltaX / length;
    const float directionZ = deltaZ / length;

    const float perpendicularX = -directionZ;
    const float perpendicularZ = directionX;

    const Vector3 start{
        startNode.x + perpendicularX * offset,
        0.07f,
        startNode.z + perpendicularZ * offset
    };

    const Vector3 end{
        endNode.x + perpendicularX * offset,
        0.07f,
        endNode.z + perpendicularZ * offset
    };

    DrawLine3D(start, end, color);
}

void drawDirectionArrow(const RoadPoint& point, float size, Color color)
{
    const Vector3 start{
        point.x - point.directionX * size * 0.5f,
        0.1f,
        point.z - point.directionZ * size * 0.5f
    };

    const Vector3 end{
        point.x + point.directionX * size * 0.5f,
        0.1f,
        point.z + point.directionZ * size * 0.5f
    };

    DrawLine3D(start, end, color);

    const float rightX = point.directionZ;
    const float rightZ = -point.directionX;

    const Vector3 leftTip{
        end.x - point.directionX * size * 0.35f + rightX * size * 0.2f,
        0.1f,
        end.z - point.directionZ * size * 0.35f + rightZ * size * 0.2f
    };

    const Vector3 rightTip{
        end.x - point.directionX * size * 0.35f - rightX * size * 0.2f,
        0.1f,
        end.z - point.directionZ * size * 0.35f - rightZ * size * 0.2f
    };

    DrawLine3D(end, leftTip, color);
    DrawLine3D(end, rightTip, color);
}

int main(int argc, char* argv[])
{
    std::cout << "GPU Traffic Simulator\n";
    std::cout << "=====================\n\n";

    std::size_t vehicleCount = 30;

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

    if (vehicleCount == 0)
    {
        std::cerr << "Vehicle count must be greater than 0.\n";
        return 1;
    }

    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "GPU Traffic Simulator");
    SetTargetFPS(60);

    CameraController cameraController;
    TrafficSimulation simulation(vehicleCount);

    const RoadNetwork& initialRoadNetwork = simulation.getRoadNetwork();

    cameraController.setHorizontalBounds(0.0f, initialRoadNetwork.getMaxX());
    cameraController.setTarget({
        initialRoadNetwork.getMaxX() * 0.5f,
        0.0f,
        initialRoadNetwork.getMaxZ() * 0.5f
    });

    std::size_t selectedVehicleIndex = 0;

    while (!WindowShouldClose())
    {
        const float deltaTime = GetFrameTime();

        cameraController.update(deltaTime);
        simulation.update(deltaTime);

        const auto& vehicles = simulation.getVehicles();

        const RoadNetwork& roadNetwork = simulation.getRoadNetwork();
        const RoadNetworkConfig& networkConfig = roadNetwork.getConfig();

        const auto& nodes = roadNetwork.getNodes();
        const auto& roads = roadNetwork.getRoads();
        const auto& edges = roadNetwork.getEdges();

        if (!vehicles.empty())
        {
            if (IsKeyPressed(KEY_RIGHT))
                selectedVehicleIndex = (selectedVehicleIndex + 1) % vehicles.size();

            if (IsKeyPressed(KEY_LEFT))
                selectedVehicleIndex = (selectedVehicleIndex + vehicles.size() - 1) % vehicles.size();
        }

        const Vector3 cameraTarget = cameraController.getTarget();
        const float renderDistance = cameraController.getRenderDistance();
        const float renderDistanceSquared = renderDistance * renderDistance;

        const float laneWidth = networkConfig.laneWidth;
        const int lanesPerDirection = networkConfig.lanesPerDirection;

        const float roadWidth =
            laneWidth * static_cast<float>(lanesPerDirection * 2);

        std::size_t renderedVehicleCount = 0;

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(cameraController.getCamera());

        // Physical roads
        for (const Road& road : roads)
        {
            const RoadNode& nodeA = nodes[road.nodeAIndex];
            const RoadNode& nodeB = nodes[road.nodeBIndex];

            const float centerX = (nodeA.x + nodeB.x) * 0.5f;
            const float centerZ = (nodeA.z + nodeB.z) * 0.5f;

            const float cameraDeltaX = centerX - cameraTarget.x;
            const float cameraDeltaZ = centerZ - cameraTarget.z;
            const float visibilityDistance = renderDistance + road.length * 0.5f;

            if (cameraDeltaX * cameraDeltaX + cameraDeltaZ * cameraDeltaZ >
                visibilityDistance * visibilityDistance)
                continue;

            const float deltaX = std::abs(nodeB.x - nodeA.x);
            const float deltaZ = std::abs(nodeB.z - nodeA.z);

            const float width = deltaX > 0.0f ? road.length : roadWidth;
            const float depth = deltaZ > 0.0f ? road.length : roadWidth;

            DrawCube({centerX, -0.05f, centerZ}, width, 0.1f, depth, DARKGRAY);

            // Central line separating both directions
            drawOffsetLine(nodeA, nodeB, 0.0f, YELLOW);

            // Lane separators
            for (int lane = 1; lane < lanesPerDirection; ++lane)
            {
                const float offset = static_cast<float>(lane) * laneWidth;

                drawOffsetLine(nodeA, nodeB, offset, WHITE);
                drawOffsetLine(nodeA, nodeB, -offset, WHITE);
            }
        }

        // Intersections
        for (const RoadNode& node : nodes)
        {
            const float deltaX = node.x - cameraTarget.x;
            const float deltaZ = node.z - cameraTarget.z;

            if (deltaX * deltaX + deltaZ * deltaZ > renderDistanceSquared)
                continue;

            DrawCube({node.x, 0.0f, node.z}, roadWidth, 0.12f, roadWidth, GRAY);
        }

        // Direction arrows
        if (cameraController.getDistance() <= 1200.0f)
        {
            for (const RoadEdge& edge : edges)
            {
                const RoadPoint middle =
                    roadNetwork.getPointOnEdge(edge.id, edge.length * 0.5f);

                const float deltaX = middle.x - cameraTarget.x;
                const float deltaZ = middle.z - cameraTarget.z;

                if (deltaX * deltaX + deltaZ * deltaZ > renderDistanceSquared)
                    continue;

                for (int lane = 0; lane < edge.laneCount; ++lane)
                {
                    for (const float factor : {0.3f, 0.7f})
                    {
                        const RoadPoint point =
                            roadNetwork.getLanePointOnEdge(
                                edge.id,
                                edge.length * factor,
                                lane
                            );

                        drawDirectionArrow(point, 8.0f, YELLOW);
                    }
                }
            }
        }

        // Vehicles
        for (std::size_t i = 0; i < vehicles.size(); ++i)
        {
            const Vehicle& vehicle = vehicles[i];

            const RoadPoint roadPoint =
                roadNetwork.getLanePointOnEdge(
                    vehicle.currentEdgeId,
                    vehicle.position,
                    vehicle.lane
                );

            const float deltaX = roadPoint.x - cameraTarget.x;
            const float deltaZ = roadPoint.z - cameraTarget.z;
            const float distanceSquared = deltaX * deltaX + deltaZ * deltaZ;

            if (distanceSquared > renderDistanceSquared)
                continue;

            Color vehicleColor =
                vehicle.desiredSpeed < 10.0f ? RED : BLUE;

            if (i == selectedVehicleIndex)
                vehicleColor = YELLOW;

            const Vector3 vehiclePosition{
                roadPoint.x,
                0.55f,
                roadPoint.z
            };

            const bool horizontal =
                std::abs(roadPoint.directionX) >
                std::abs(roadPoint.directionZ);

            if (horizontal)
                DrawCube(vehiclePosition, 4.0f, 1.0f, 2.0f, vehicleColor);
            else
                DrawCube(vehiclePosition, 2.0f, 1.0f, 4.0f, vehicleColor);

            if (i == selectedVehicleIndex)
            {
                if (horizontal)
                    DrawCubeWires(vehiclePosition, 4.0f, 1.0f, 2.0f, BLACK);
                else
                    DrawCubeWires(vehiclePosition, 2.0f, 1.0f, 4.0f, BLACK);
            }

            ++renderedVehicleCount;
        }

        EndMode3D();

        // Debug labels for nodes and directed edges
        if (cameraController.getDistance() <= 500.0f)
        {

            for (const RoadNode& node : nodes)
            {
                const float deltaX = node.x - cameraTarget.x;
                const float deltaZ = node.z - cameraTarget.z;

                if (deltaX * deltaX + deltaZ * deltaZ > renderDistanceSquared)
                    continue;

                const Vector2 screenPosition =
                    GetWorldToScreen(
                        {node.x, 3.0f, node.z},
                        cameraController.getCamera()
                    );

                DrawText(
                    TextFormat("N%d", node.id),
                    static_cast<int>(screenPosition.x),
                    static_cast<int>(screenPosition.y),
                    16,
                    RED
                );
            }
        }

        const VehicleTelemetry telemetry =
            simulation.getVehicleTelemetry(selectedVehicleIndex);

        const int selectedLane =
            vehicles.empty()
                ? -1
                : vehicles[selectedVehicleIndex].lane;

        // Simulation statistics
        DrawText("GPU Traffic Simulator", 20, 20, 28, DARKGRAY);
        DrawText("Bidirectional graph + CUDA IDM", 20, 58, 20, GRAY);

        DrawText(TextFormat("Vehicles: %zu", vehicles.size()), 20, 95, 18, DARKGRAY);
        DrawText(TextFormat("Visible vehicles: %zu / %zu", renderedVehicleCount, vehicles.size()), 20, 120, 18, DARKGRAY);

        DrawText(TextFormat("Network: %dx%d", networkConfig.rows, networkConfig.columns), 20, 145, 18, DARKGRAY);
        DrawText(TextFormat("Roads: %zu", roads.size()), 20, 170, 18, DARKGRAY);
        DrawText(TextFormat("Directed edges: %zu", edges.size()), 20, 195, 18, DARKGRAY);
        DrawText(TextFormat("Lanes / direction: %d", lanesPerDirection), 20, 220, 18, DARKGRAY);
        DrawText(TextFormat("Total road length: %.0f m", simulation.getRoadLength()), 20, 245, 18, DARKGRAY);

        DrawText(TextFormat("Kernel: %.4f ms", simulation.getLastKernelTimeMs()), 20, 280, 18, DARKGREEN);
        DrawText(TextFormat("CUDA update total: %.4f ms", simulation.getLastCudaUpdateTimeMs()), 20, 305, 18, DARKGREEN);

        DrawFPS(20, 335);

        DrawText(TextFormat("Camera X: %.0f m", cameraController.getTarget().x), 20, 365, 18, GRAY);
        DrawText(TextFormat("Camera distance: %.0f m", cameraController.getDistance()), 20, 390, 18, GRAY);

        DrawText(
            "WASD Move | Shift Fast | Q/E Rotate | T/G Tilt | R/F Height | Wheel Zoom",
            20,
            680,
            16,
            GRAY
        );

        // Selected vehicle telemetry
        DrawRectangle(
            screenWidth - 310,
            20,
            290,
            270,
            Fade(BLACK, 0.75f)
        );

        DrawText(TextFormat("Vehicle #%d", telemetry.vehicleId), screenWidth - 290, 35, 22, WHITE);
        DrawText(TextFormat("Edge: #%d -> #%d", telemetry.edgeId, telemetry.nextEdgeId), screenWidth - 290, 70, 18, WHITE);
        DrawText(TextFormat("Lane: #%d", selectedLane), screenWidth - 290, 95, 18, WHITE);
        DrawText(TextFormat("Destination: N%d", telemetry.destinationNodeId), screenWidth - 290, 120, 18, WHITE);
        DrawText(TextFormat("Speed: %.2f m/s", telemetry.speed), screenWidth - 290, 145, 18, WHITE);
        DrawText(TextFormat("Desired: %.2f m/s", telemetry.desiredSpeed), screenWidth - 290, 170, 18, WHITE);
        DrawText(TextFormat("Acceleration: %.2f m/s2", telemetry.acceleration), screenWidth - 290, 195, 18, WHITE);

        if (telemetry.leaderId >= 0)
        {
            DrawText(TextFormat("Leader: #%d", telemetry.leaderId), screenWidth - 290, 220, 18, WHITE);
            DrawText(TextFormat("Gap: %.2f m", telemetry.gap), screenWidth - 290, 245, 18, WHITE);
        }
        else
        {
            DrawText("Leader: none", screenWidth - 290, 220, 18, WHITE);
            DrawText("Gap: --", screenWidth - 290, 245, 18, WHITE);
        }

        EndDrawing();
    }

    CloseWindow();

    return 0;
}