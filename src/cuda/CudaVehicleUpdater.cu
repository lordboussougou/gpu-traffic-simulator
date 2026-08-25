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

__device__ float distanceAhead(float vehiclePosition, float leaderPosition, float roadLength)
{
    float distance = leaderPosition - vehiclePosition;
    if (distance <= 0.0f) distance += roadLength;

    return distance;
}

__device__ int findLeaderIndex(const Vehicle* vehicles, int vehicleCount, int vehicleIndex, float roadLength)
{
    if (vehicleCount <= 1) return vehicleIndex;

    const Vehicle& vehicle = vehicles[vehicleIndex];

    int closestIndex = vehicleIndex;
    float closestDistance = roadLength;

    for (int i = 0; i < vehicleCount; ++i)
    {
        if (i == vehicleIndex) continue;

        const float distance = distanceAhead(vehicle.position, vehicles[i].position, roadLength);

        if (distance < closestDistance)
        {
            closestDistance = distance;
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

__global__ void updateVehiclesKernel(const Vehicle* inputVehicles, Vehicle* outputVehicles, int vehicleCount,
                                     float deltaTime, float roadLength, float vehicleLength,
                                     IDMParameters idmParameters)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= vehicleCount) return;

    const Vehicle vehicle = inputVehicles[index];

    float leaderSpeed = vehicle.speed;
    float gap = roadLength - vehicleLength;

    if (vehicleCount > 1)
    {
        const int leaderIndex = findLeaderIndex(inputVehicles, vehicleCount, index, roadLength);
        const Vehicle leader = inputVehicles[leaderIndex];

        const float centerDistance = distanceAhead(vehicle.position, leader.position, roadLength);

        leaderSpeed = leader.speed;
        gap = fmaxf(centerDistance - vehicleLength, 0.1f);
    }

    const float acceleration =
        computeIdmAcceleration(vehicle.speed, vehicle.desiredSpeed, leaderSpeed, gap, idmParameters);

    Vehicle updatedVehicle = vehicle;

    updatedVehicle.acceleration = acceleration;
    updatedVehicle.speed = fmaxf(vehicle.speed + acceleration * deltaTime, 0.0f);
    updatedVehicle.position = vehicle.position + updatedVehicle.speed * deltaTime;

    if (updatedVehicle.position >= roadLength)
        updatedVehicle.position = fmodf(updatedVehicle.position, roadLength);

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

bool CudaVehicleUpdater::ensureCapacity(std::size_t count)
{
    if (count <= capacity_) return true;

    release();

    const std::size_t vehicleBytes = count * sizeof(Vehicle);

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceVehiclesInput_), vehicleBytes),
                   "cudaMalloc input vehicles"))
        return false;

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceVehiclesOutput_), vehicleBytes),
                   "cudaMalloc output vehicles"))
    {
        release();
        return false;
    }

    capacity_ = count;

    return true;
}

bool CudaVehicleUpdater::update(std::vector<Vehicle>& vehicles, float deltaTime, float roadLength,
                                float vehicleLength, const IDMParameters& idmParameters)
{
    if (vehicles.empty()) return true;
    if (!ensureCapacity(vehicles.size())) return false;

    const std::size_t vehicleBytes = vehicles.size() * sizeof(Vehicle);

    if (!checkCuda(cudaMemcpy(deviceVehiclesInput_, vehicles.data(), vehicleBytes, cudaMemcpyHostToDevice),
                   "cudaMemcpy vehicles HostToDevice"))
        return false;

    constexpr int threadsPerBlock = 256;

    const int vehicleCount = static_cast<int>(vehicles.size());
    const int blockCount = (vehicleCount + threadsPerBlock - 1) / threadsPerBlock;

    if (!checkCuda(cudaEventRecord(kernelStartEvent_), "cudaEventRecord start")) return false;

    updateVehiclesKernel<<<blockCount, threadsPerBlock>>>(
        deviceVehiclesInput_,
        deviceVehiclesOutput_,
        vehicleCount,
        deltaTime,
        roadLength,
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

    deviceVehiclesInput_ = nullptr;
    deviceVehiclesOutput_ = nullptr;
    capacity_ = 0;
}