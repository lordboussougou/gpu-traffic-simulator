#pragma once

#include "config/RoadNetworkConfig.hpp"

#include <vector>

struct RoadNode
{
    int id = 0;

    float x = 0.0f;
    float z = 0.0f;
};

struct Road
{
    int id = 0;

    int nodeAIndex = 0;
    int nodeBIndex = 0;

    int forwardEdgeId = -1;
    int reverseEdgeId = -1;

    float length = 0.0f;
};

struct RoadEdge
{
    int id = 0;
    int roadId = 0;

    int startNodeIndex = 0;
    int endNodeIndex = 0;

    float length = 0.0f;

    int laneCount = 2;
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
    explicit RoadNetwork(const RoadNetworkConfig& config);

    const RoadNetworkConfig& getConfig() const;

    const std::vector<RoadNode>& getNodes() const;
    const std::vector<Road>& getRoads() const;
    const std::vector<RoadEdge>& getEdges() const;

    RoadPoint getPointOnEdge(int edgeIndex, float edgePosition) const;
    RoadPoint getLanePointOnEdge(int edgeIndex, float edgePosition, int lane) const;

    std::vector<int> findShortestPath(int startNodeIndex, int destinationNodeIndex) const;

    float getMaxX() const;
    float getMaxZ() const;
    float getTotalRoadLength() const;

private:
    void generateGrid();
    void addBidirectionalRoad(int nodeAIndex, int nodeBIndex);
    void buildShortestPathTree(int startNodeIndex) const;

    RoadNetworkConfig config_;

    std::vector<RoadNode> nodes_;
    std::vector<Road> roads_;
    std::vector<RoadEdge> edges_;

    std::vector<std::vector<int>> outgoingEdgeIndices_;

    mutable std::vector<std::vector<int>> previousEdgeCache_;
    mutable std::vector<bool> shortestPathTreeReady_;

    float maxX_ = 0.0f;
    float maxZ_ = 0.0f;
    float totalRoadLength_ = 0.0f;
};