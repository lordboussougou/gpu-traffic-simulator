#include "RoadNetwork.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

RoadNetwork::RoadNetwork(float targetLoopLength)
{
    constexpr float baseLoopLength = 1400.0f;

    const float safeLoopLength = std::max(targetLoopLength, 300.0f);
    const float scale = safeLoopLength / baseLoopLength;

    nodes_ = {
        {0,   0.0f * scale, 100.0f * scale}, // A
        {1, 200.0f * scale, 100.0f * scale}, // B
        {2, 200.0f * scale,   0.0f * scale}, // C
        {3, 400.0f * scale,   0.0f * scale}, // D
        {4, 200.0f * scale, 200.0f * scale}, // E
        {5, 400.0f * scale, 200.0f * scale}, // F
        {6, 400.0f * scale, 100.0f * scale}, // G
        {7, 400.0f * scale, 300.0f * scale}, // H
        {8,   0.0f * scale, 300.0f * scale}  // I
    };

    outgoingEdgeIndices_.resize(nodes_.size());

    addEdge(0, 1); // 0 : A -> B

    addEdge(1, 2); // 1 : B -> C
    addEdge(2, 3); // 2 : C -> D
    addEdge(3, 6); // 3 : D -> G

    addEdge(1, 4); // 4 : B -> E
    addEdge(4, 5); // 5 : E -> F
    addEdge(5, 6); // 6 : F -> G

    addEdge(6, 7); // 7 : G -> H
    addEdge(7, 8); // 8 : H -> I
    addEdge(8, 0); // 9 : I -> A

    referenceLoopLength_ = safeLoopLength;

    for (const RoadNode& node : nodes_)
    {
        maxX_ = std::max(maxX_, node.x);
        maxZ_ = std::max(maxZ_, node.z);
    }
}

void RoadNetwork::addEdge(int startNodeIndex, int endNodeIndex)
{
    const RoadNode& start = nodes_[startNodeIndex];
    const RoadNode& end = nodes_[endNodeIndex];

    const float deltaX = end.x - start.x;
    const float deltaZ = end.z - start.z;
    const float length = std::sqrt(deltaX * deltaX + deltaZ * deltaZ);

    const int edgeIndex = static_cast<int>(edges_.size());

    edges_.push_back({
        edgeIndex,
        startNodeIndex,
        endNodeIndex,
        length
    });

    outgoingEdgeIndices_[startNodeIndex].push_back(edgeIndex);
}

const std::vector<RoadNode>& RoadNetwork::getNodes() const
{
    return nodes_;
}

const std::vector<RoadEdge>& RoadNetwork::getEdges() const
{
    return edges_;
}

RoadPoint RoadNetwork::getPointOnEdge(int edgeIndex, float edgePosition) const
{
    if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges_.size())) return {};

    const RoadEdge& edge = edges_[edgeIndex];
    const RoadNode& start = nodes_[edge.startNodeIndex];
    const RoadNode& end = nodes_[edge.endNodeIndex];

    const float safePosition = std::clamp(edgePosition, 0.0f, edge.length);
    const float t = edge.length > 0.0f ? safePosition / edge.length : 0.0f;

    RoadPoint point;

    point.x = start.x + (end.x - start.x) * t;
    point.z = start.z + (end.z - start.z) * t;

    point.directionX = (end.x - start.x) / edge.length;
    point.directionZ = (end.z - start.z) / edge.length;

    return point;
}

std::vector<int> RoadNetwork::findShortestPath(int startNodeIndex, int destinationNodeIndex) const
{
    if (startNodeIndex < 0 || startNodeIndex >= static_cast<int>(nodes_.size())) return {};
    if (destinationNodeIndex < 0 || destinationNodeIndex >= static_cast<int>(nodes_.size())) return {};
    if (startNodeIndex == destinationNodeIndex) return {};

    const float infinity = std::numeric_limits<float>::max();

    std::vector<float> distances(nodes_.size(), infinity);
    std::vector<int> previousEdge(nodes_.size(), -1);

    using QueueEntry = std::pair<float, int>;

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        std::greater<QueueEntry>
    > queue;

    distances[startNodeIndex] = 0.0f;
    queue.push({0.0f, startNodeIndex});

    while (!queue.empty())
    {
        const auto [currentDistance, currentNode] = queue.top();
        queue.pop();

        if (currentDistance > distances[currentNode]) continue;
        if (currentNode == destinationNodeIndex) break;

        for (const int edgeIndex : outgoingEdgeIndices_[currentNode])
        {
            const RoadEdge& edge = edges_[edgeIndex];
            const int nextNode = edge.endNodeIndex;

            const float newDistance = currentDistance + edge.length;

            if (newDistance < distances[nextNode])
            {
                distances[nextNode] = newDistance;
                previousEdge[nextNode] = edgeIndex;

                queue.push({newDistance, nextNode});
            }
        }
    }

    if (previousEdge[destinationNodeIndex] < 0) return {};

    std::vector<int> path;

    int currentNode = destinationNodeIndex;

    while (currentNode != startNodeIndex)
    {
        const int edgeIndex = previousEdge[currentNode];

        if (edgeIndex < 0) return {};

        path.push_back(edgeIndex);
        currentNode = edges_[edgeIndex].startNodeIndex;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

float RoadNetwork::getReferenceLoopLength() const
{
    return referenceLoopLength_;
}

float RoadNetwork::getMaxX() const
{
    return maxX_;
}

float RoadNetwork::getMaxZ() const
{
    return maxZ_;
}