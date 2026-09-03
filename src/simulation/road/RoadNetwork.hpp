#pragma once

#include <vector>

struct RoadNode
{
    int id = 0;
    float x = 0.0f;
    float z = 0.0f;
};

struct RoadEdge
{
    int id = 0;
    int startNodeIndex = 0;
    int endNodeIndex = 0;
    float length = 0.0f;
};

struct RoadPoint
{
    float x = 0.0f;
    float z = 0.0f;

    float directionX = 1.0f;
    float directionZ = 0.0f;
};

class RoadNetwork
{
public:
    explicit RoadNetwork(float routeLength);

    const std::vector<RoadNode>& getNodes() const;
    const std::vector<RoadEdge>& getEdges() const;

    float getRouteLength() const;
    RoadPoint getPointOnRoute(float routePosition) const;

private:
    std::vector<RoadNode> nodes_;
    std::vector<RoadEdge> edges_;
    std::vector<int> routeEdgeIndices_;

    float routeLength_ = 0.0f;
};