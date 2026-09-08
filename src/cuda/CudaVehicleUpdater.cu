#include "CudaVehicleUpdater.cuh"

#include <cuda_runtime.h>

#include <cmath>
#include <iostream>
#include <type_traits>

namespace
{
bool checkCuda(cudaError_t error, const char* operation)
{
    if (error == cudaSuccess) return true;

    std::cerr << "[CUDA ERROR] " << operation << ": " << cudaGetErrorString(error) << '\n';
    return false;
}

__device__ int findLeaderIndex(const Vehicle* vehicles, const float* edgeLengths, int vehicleCount, int edgeCount,
                               int vehicleIndex, float& leaderDistance)
{
    const Vehicle& vehicle = vehicles[vehicleIndex];

    int closestIndex = -1;
    leaderDistance = 1.0e30f;

    for (int i = 0; i < vehicleCount; ++i)
    {
        if (i == vehicleIndex) continue;

        const Vehicle& candidate = vehicles[i];

        if (candidate.currentEdgeId != vehicle.currentEdgeId) continue;

        const float distance = candidate.position - vehicle.position;

        if (distance > 0.0f && distance < leaderDistance)
        {
            leaderDistance = distance;
            closestIndex = i;
        }
    }

    if (closestIndex >= 0) return closestIndex;

    if (vehicle.nextEdgeId < 0 || vehicle.nextEdgeId >= edgeCount) return -1;
    if (vehicle.currentEdgeId < 0 || vehicle.currentEdgeId >= edgeCount) return -1;

    const float distanceToIntersection = edgeLengths[vehicle.currentEdgeId] - vehicle.position;

    for (int i = 0; i < vehicleCount; ++i)
    {
        if (i == vehicleIndex) continue;

        const Vehicle& candidate = vehicles[i];

        if (candidate.currentEdgeId != vehicle.nextEdgeId) continue;

        const float distance = distanceToIntersection + candidate.position;

        if (distance > 0.0f && distance < leaderDistance)
        {
            leaderDistance = distance;
            closestIndex = i;
        }
    }

    return closestIndex;
}

__device__ float computeIdmAcceleration(float speed, float desiredSpeed, float leaderSpeed, float gap,
                                        const IDMParameters& parameters)
{
    const float deltaSpeed = speed - leaderSpeed;

    const float brakingDenominator =
        2.0f * sqrtf(parameters.maxAcceleration * parameters.comfortableBraking);

    const float brakingTerm = (speed * deltaSpeed) / brakingDenominator;

    const float desiredGap =
        parameters.minimumGap + fmaxf(0.0f, speed * parameters.timeHeadway + brakingTerm);

    const float safeGap = fmaxf(gap, 0.1f);
    const float safeDesiredSpeed = fmaxf(desiredSpeed, 0.1f);

    const float freeRoadTerm = powf(speed / safeDesiredSpeed, parameters.accelerationExponent);
    const float interactionTerm = powf(desiredGap / safeGap, 2.0f);

    return parameters.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

__global__ void updateVehiclesKernel(const Vehicle* inputVehicles, Vehicle* outputVehicles,
                                     const float* edgeLengths, int vehicleCount, int edgeCount,
                                     float deltaTime, float vehicleLength, IDMParameters idmParameters)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= vehicleCount) return;

    const Vehicle vehicle = inputVehicles[index];

    float leaderSpeed = vehicle.speed;
    float leaderDistance = 1.0e30f;

    const int leaderIndex =
        findLeaderIndex(inputVehicles, edgeLengths, vehicleCount, edgeCount, index, leaderDistance);

    float gap = 1000000.0f;

    if (leaderIndex >= 0)
    {
        leaderSpeed = inputVehicles[leaderIndex].speed;
        gap = fmaxf(leaderDistance - vehicleLength, 0.1f);
    }

    const float acceleration =
        computeIdmAcceleration(vehicle.speed, vehicle.desiredSpeed, leaderSpeed, gap, idmParameters);

    Vehicle updatedVehicle = vehicle;

    updatedVehicle.acceleration = acceleration;
    updatedVehicle.speed = fmaxf(vehicle.speed + acceleration * deltaTime, 0.0f);
    updatedVehicle.position = vehicle.position + updatedVehicle.speed * deltaTime;

    outputVehicles[index] = updatedVehicle;
}
}

static_assert(std::is_trivially_copyable_v<Vehicle>,
              "Vehicle must remain trivially copyable to transfer it directly between CPU and GPU.");

CudaVehicleUpdater::CudaVehicleUpdater()
{
    checkCuda(cudaEventCreate(&kernelStartEvent_), "cudaEventCreate start");
    checkCuda(cudaEventCreate(&kernelStopEvent_), "cudaEventCreate stop");
}

CudaVehicleUpdater::~CudaVehicleUpdater()
{
    release();

    if (kernelStartEvent_) cudaEventDestroy(kernelStartEvent_);
    if (kernelStopEvent_) cudaEventDestroy(kernelStopEvent_);
}

bool CudaVehicleUpdater::ensureCapacity(std::size_t vehicleCount, std::size_t edgeCount)
{
    if (vehicleCount <= vehicleCapacity_ && edgeCount <= edgeCapacity_) return true;

    release();

    const std::size_t vehicleBytes = vehicleCount * sizeof(Vehicle);
    const std::size_t edgeBytes = edgeCount * sizeof(float);

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceVehiclesInput_), vehicleBytes),
                   "cudaMalloc input vehicles"))
        return false;

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceVehiclesOutput_), vehicleBytes),
                   "cudaMalloc output vehicles"))
    {
        release();
        return false;
    }

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceEdgeLengths_), edgeBytes),
                   "cudaMalloc edge lengths"))
    {
        release();
        return false;
    }

    vehicleCapacity_ = vehicleCount;
    edgeCapacity_ = edgeCount;

    return true;
}

bool CudaVehicleUpdater::update(std::vector<Vehicle>& vehicles, const std::vector<float>& edgeLengths,
                                float deltaTime, float vehicleLength, const IDMParameters& idmParameters)
{
    if (vehicles.empty()) return true;
    if (edgeLengths.empty()) return false;

    if (!ensureCapacity(vehicles.size(), edgeLengths.size())) return false;

    const std::size_t vehicleBytes = vehicles.size() * sizeof(Vehicle);
    const std::size_t edgeBytes = edgeLengths.size() * sizeof(float);

    if (!checkCuda(cudaMemcpy(deviceVehiclesInput_, vehicles.data(), vehicleBytes, cudaMemcpyHostToDevice),
                   "cudaMemcpy vehicles HostToDevice"))
        return false;

    if (!checkCuda(cudaMemcpy(deviceEdgeLengths_, edgeLengths.data(), edgeBytes, cudaMemcpyHostToDevice),
                   "cudaMemcpy edge lengths HostToDevice"))
        return false;

    constexpr int threadsPerBlock = 256;

    const int vehicleCount = static_cast<int>(vehicles.size());
    const int edgeCount = static_cast<int>(edgeLengths.size());

    const int blockCount = (vehicleCount + threadsPerBlock - 1) / threadsPerBlock;

    if (!checkCuda(cudaEventRecord(kernelStartEvent_), "cudaEventRecord start")) return false;

    updateVehiclesKernel<<<blockCount, threadsPerBlock>>>(
        deviceVehiclesInput_,
        deviceVehiclesOutput_,
        deviceEdgeLengths_,
        vehicleCount,
        edgeCount,
        deltaTime,
        vehicleLength,
        idmParameters
    );

    if (!checkCuda(cudaGetLastError(), "updateVehiclesKernel launch")) return false;

    if (!checkCuda(cudaEventRecord(kernelStopEvent_), "cudaEventRecord stop")) return false;
    if (!checkCuda(cudaEventSynchronize(kernelStopEvent_), "cudaEventSynchronize stop")) return false;

    if (!checkCuda(cudaEventElapsedTime(&lastKernelTimeMs_, kernelStartEvent_, kernelStopEvent_),
                   "cudaEventElapsedTime"))
        return false;

    if (!checkCuda(cudaMemcpy(vehicles.data(), deviceVehiclesOutput_, vehicleBytes, cudaMemcpyDeviceToHost),
                   "cudaMemcpy vehicles DeviceToHost"))
        return false;

    return true;
}

float CudaVehicleUpdater::getLastKernelTimeMs() const
{
    return lastKernelTimeMs_;
}

void CudaVehicleUpdater::release()
{
    if (deviceVehiclesInput_) cudaFree(deviceVehiclesInput_);
    if (deviceVehiclesOutput_) cudaFree(deviceVehiclesOutput_);
    if (deviceEdgeLengths_) cudaFree(deviceEdgeLengths_);

    deviceVehiclesInput_ = nullptr;
    deviceVehiclesOutput_ = nullptr;
    deviceEdgeLengths_ = nullptr;

    vehicleCapacity_ = 0;
    edgeCapacity_ = 0;
}