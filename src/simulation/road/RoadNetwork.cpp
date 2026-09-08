#include "RoadNetwork.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

RoadNetwork::RoadNetwork(const RoadNetworkConfig& config)
    : config_(config)
{
    generateGrid();

    previousEdgeCache_.resize(nodes_.size());
    shortestPathTreeReady_.resize(nodes_.size(), false);
}

void RoadNetwork::generateGrid()
{
    nodes_.clear();
    roads_.clear();
    edges_.clear();

    const int rows = std::max(config_.rows, 2);
    const int columns = std::max(config_.columns, 2);

    nodes_.reserve(static_cast<std::size_t>(rows * columns));

    for (int row = 0; row < rows; ++row)
    {
        for (int column = 0; column < columns; ++column)
        {
            RoadNode node;

            node.id = row * columns + column;
            node.x = static_cast<float>(column) * config_.blockLength;
            node.z = static_cast<float>(row) * config_.blockLength;

            nodes_.push_back(node);
        }
    }

    outgoingEdgeIndices_.resize(nodes_.size());

    for (int row = 0; row < rows; ++row)
    {
        for (int column = 0; column < columns; ++column)
        {
            const int currentNode = row * columns + column;

            if (column + 1 < columns)
                addBidirectionalRoad(currentNode, currentNode + 1);

            if (row + 1 < rows)
                addBidirectionalRoad(currentNode, currentNode + columns);
        }
    }

    maxX_ = static_cast<float>(columns - 1) * config_.blockLength;
    maxZ_ = static_cast<float>(rows - 1) * config_.blockLength;
}

void RoadNetwork::addBidirectionalRoad(int nodeAIndex, int nodeBIndex)
{
    const RoadNode& nodeA = nodes_[nodeAIndex];
    const RoadNode& nodeB = nodes_[nodeBIndex];

    const float deltaX = nodeB.x - nodeA.x;
    const float deltaZ = nodeB.z - nodeA.z;
    const float length = std::sqrt(deltaX * deltaX + deltaZ * deltaZ);

    const int roadId = static_cast<int>(roads_.size());
    const int forwardEdgeId = static_cast<int>(edges_.size());
    const int reverseEdgeId = forwardEdgeId + 1;

    Road road;

    road.id = roadId;
    road.nodeAIndex = nodeAIndex;
    road.nodeBIndex = nodeBIndex;
    road.forwardEdgeId = forwardEdgeId;
    road.reverseEdgeId = reverseEdgeId;
    road.length = length;

    roads_.push_back(road);

    RoadEdge forwardEdge;

    forwardEdge.id = forwardEdgeId;
    forwardEdge.roadId = roadId;
    forwardEdge.startNodeIndex = nodeAIndex;
    forwardEdge.endNodeIndex = nodeBIndex;
    forwardEdge.length = length;
    forwardEdge.laneCount = config_.lanesPerDirection;

    edges_.push_back(forwardEdge);

    RoadEdge reverseEdge;

    reverseEdge.id = reverseEdgeId;
    reverseEdge.roadId = roadId;
    reverseEdge.startNodeIndex = nodeBIndex;
    reverseEdge.endNodeIndex = nodeAIndex;
    reverseEdge.length = length;
    reverseEdge.laneCount = config_.lanesPerDirection;

    edges_.push_back(reverseEdge);

    outgoingEdgeIndices_[nodeAIndex].push_back(forwardEdgeId);
    outgoingEdgeIndices_[nodeBIndex].push_back(reverseEdgeId);

    totalRoadLength_ += length;
}

const RoadNetworkConfig& RoadNetwork::getConfig() const
{
    return config_;
}

const std::vector<RoadNode>& RoadNetwork::getNodes() const
{
    return nodes_;
}

const std::vector<Road>& RoadNetwork::getRoads() const
{
    return roads_;
}

const std::vector<RoadEdge>& RoadNetwork::getEdges() const
{
    return edges_;
}

RoadPoint RoadNetwork::getPointOnEdge(int edgeIndex, float edgePosition) const
{
    if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges_.size())) return {};

    const RoadEdge& edge = edges_[edgeIndex];

    const RoadNode& startNode = nodes_[edge.startNodeIndex];
    const RoadNode& endNode = nodes_[edge.endNodeIndex];

    const float safePosition = std::clamp(edgePosition, 0.0f, edge.length);
    const float t = edge.length > 0.0f ? safePosition / edge.length : 0.0f;

    RoadPoint point;

    point.x = startNode.x + (endNode.x - startNode.x) * t;
    point.z = startNode.z + (endNode.z - startNode.z) * t;

    point.directionX = (endNode.x - startNode.x) / edge.length;
    point.directionZ = (endNode.z - startNode.z) / edge.length;

    return point;
}

RoadPoint RoadNetwork::getLanePointOnEdge(int edgeIndex, float edgePosition, int lane) const
{
    RoadPoint point = getPointOnEdge(edgeIndex, edgePosition);

    if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges_.size())) return point;

    const RoadEdge& edge = edges_[edgeIndex];
    const int safeLane = std::clamp(lane, 0, edge.laneCount - 1);

    const float rightX = point.directionZ;
    const float rightZ = -point.directionX;

    const float laneOffset =
        (static_cast<float>(safeLane) + 0.5f) * config_.laneWidth;

    point.x += rightX * laneOffset;
    point.z += rightZ * laneOffset;

    return point;
}

void RoadNetwork::buildShortestPathTree(int startNodeIndex) const
{
    if (startNodeIndex < 0 || startNodeIndex >= static_cast<int>(nodes_.size())) return;
    if (shortestPathTreeReady_[startNodeIndex]) return;

    const float infinity = std::numeric_limits<float>::max();

    std::vector<float> distances(nodes_.size(), infinity);
    std::vector<int>& previousEdge = previousEdgeCache_[startNodeIndex];

    previousEdge.assign(nodes_.size(), -1);

    using QueueEntry = std::pair<float, int>;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

    distances[startNodeIndex] = 0.0f;
    queue.push({0.0f, startNodeIndex});

    while (!queue.empty())
    {
        const auto [currentDistance, currentNode] = queue.top();
        queue.pop();

        if (currentDistance > distances[currentNode]) continue;

        for (const int edgeIndex : outgoingEdgeIndices_[currentNode])
        {
            const RoadEdge& edge = edges_[edgeIndex];

            const int nextNode = edge.endNodeIndex;
            const float newDistance = currentDistance + edge.length;

            if (newDistance >= distances[nextNode]) continue;

            distances[nextNode] = newDistance;
            previousEdge[nextNode] = edgeIndex;

            queue.push({newDistance, nextNode});
        }
    }

    shortestPathTreeReady_[startNodeIndex] = true;
}

std::vector<int> RoadNetwork::findShortestPath(int startNodeIndex, int destinationNodeIndex) const
{
    if (startNodeIndex < 0 || startNodeIndex >= static_cast<int>(nodes_.size())) return {};
    if (destinationNodeIndex < 0 || destinationNodeIndex >= static_cast<int>(nodes_.size())) return {};
    if (startNodeIndex == destinationNodeIndex) return {};

    buildShortestPathTree(startNodeIndex);

    const std::vector<int>& previousEdge = previousEdgeCache_[startNodeIndex];

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

float RoadNetwork::getMaxX() const
{
    return maxX_;
}

float RoadNetwork::getMaxZ() const
{
    return maxZ_;
}

float RoadNetwork::getTotalRoadLength() const
{
    return totalRoadLength_;
}