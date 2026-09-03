#include "RoadNetwork.hpp"

#include <algorithm>
#include <cmath>

RoadNetwork::RoadNetwork(float routeLength)
{
    routeLength_ = std::max(routeLength, 60.0f);

    const float height = routeLength_ / 6.0f;
    const float width = routeLength_ / 3.0f;

    nodes_ = {
        {0, 0.0f, 0.0f},
        {1, width, 0.0f},
        {2, width, height},
        {3, 0.0f, height}
    };

    edges_ = {
        {0, 0, 1, width},
        {1, 1, 2, height},
        {2, 2, 3, width},
        {3, 3, 0, height}
    };

    routeEdgeIndices_ = {0, 1, 2, 3};
}

const std::vector<RoadNode>& RoadNetwork::getNodes() const
{
    return nodes_;
}

const std::vector<RoadEdge>& RoadNetwork::getEdges() const
{
    return edges_;
}

float RoadNetwork::getRouteLength() const
{
    return routeLength_;
}

RoadPoint RoadNetwork::getPointOnRoute(float routePosition) const
{
    if (routeLength_ <= 0.0f || edges_.empty()) return {};

    float normalizedPosition = std::fmod(routePosition, routeLength_);

    if (normalizedPosition < 0.0f)
        normalizedPosition += routeLength_;

    for (const int edgeIndex : routeEdgeIndices_)
    {
        const RoadEdge& edge = edges_[edgeIndex];

        if (normalizedPosition <= edge.length)
        {
            const RoadNode& startNode = nodes_[edge.startNodeIndex];
            const RoadNode& endNode = nodes_[edge.endNodeIndex];

            const float t = edge.length > 0.0f ? normalizedPosition / edge.length : 0.0f;

            RoadPoint point;

            point.x = startNode.x + (endNode.x - startNode.x) * t;
            point.z = startNode.z + (endNode.z - startNode.z) * t;

            point.directionX = (endNode.x - startNode.x) / edge.length;
            point.directionZ = (endNode.z - startNode.z) / edge.length;

            return point;
        }

        normalizedPosition -= edge.length;
    }

    const RoadNode& firstNode = nodes_[0];

    return {
        firstNode.x,
        firstNode.z,
        1.0f,
        0.0f
    };
}