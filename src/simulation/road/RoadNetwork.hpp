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
    explicit RoadNetwork(float targetLoopLength);

    const std::vector<RoadNode>& getNodes() const;
    const std::vector<RoadEdge>& getEdges() const;

    RoadPoint getPointOnEdge(int edgeIndex, float edgePosition) const;
    int chooseNextEdge(int currentEdgeIndex, int vehicleId) const;

    float getReferenceLoopLength() const;
    float getMaxX() const;
    float getMaxZ() const;

private:
    void addEdge(int startNodeIndex, int endNodeIndex);

    std::vector<RoadNode> nodes_;
    std::vector<RoadEdge> edges_;
    std::vector<std::vector<int>> outgoingEdgeIndices_;

    float referenceLoopLength_ = 0.0f;
    float maxX_ = 0.0f;
    float maxZ_ = 0.0f;
};