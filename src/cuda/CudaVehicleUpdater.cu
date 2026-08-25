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

__device__ float computeIdmAcceleration(float speed, float desiredSpeed, float leaderSpeed, float gap,
                                        const IDMParameters& parameters)
{
    const float deltaSpeed = speed - leaderSpeed;
    const float brakingDenominator = 2.0f * sqrtf(parameters.maxAcceleration * parameters.comfortableBraking);
    const float brakingTerm = (speed * deltaSpeed) / brakingDenominator;
    const float desiredGap = parameters.minimumGap + fmaxf(0.0f, speed * parameters.timeHeadway + brakingTerm);
    const float safeGap = fmaxf(gap, 0.1f);
    const float safeDesiredSpeed = fmaxf(desiredSpeed, 0.1f);

    const float freeRoadTerm = powf(speed / safeDesiredSpeed, parameters.accelerationExponent);
    const float interactionTerm = powf(desiredGap / safeGap, 2.0f);

    return parameters.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

__global__ void updateVehiclesKernel(const Vehicle* inputVehicles, Vehicle* outputVehicles, const int* leaderIndices,
                                     int vehicleCount, float deltaTime, float roadLength, float vehicleLength,
                                     IDMParameters idmParameters)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= vehicleCount) return;

    const Vehicle vehicle = inputVehicles[index];
    const Vehicle leader = inputVehicles[leaderIndices[index]];

    float centerDistance = leader.position - vehicle.position;
    if (centerDistance <= 0.0f) centerDistance += roadLength;

    const float gap = fmaxf(centerDistance - vehicleLength, 0.1f);
    const float acceleration = computeIdmAcceleration(vehicle.speed, vehicle.desiredSpeed, leader.speed, gap, idmParameters);

    Vehicle updatedVehicle = vehicle;
    updatedVehicle.acceleration = acceleration;
    updatedVehicle.speed = fmaxf(vehicle.speed + acceleration * deltaTime, 0.0f);
    updatedVehicle.position = vehicle.position + updatedVehicle.speed * deltaTime;

    if (updatedVehicle.position >= roadLength) updatedVehicle.position = fmodf(updatedVehicle.position, roadLength);

    outputVehicles[index] = updatedVehicle;
}
}

static_assert(std::is_trivially_copyable_v<Vehicle>,
              "Vehicle must remain trivially copyable to transfer it directly between CPU and GPU.");

CudaVehicleUpdater::~CudaVehicleUpdater()
{
    release();
}

bool CudaVehicleUpdater::ensureCapacity(std::size_t count)
{
    if (count <= capacity_) return true;

    release();

    const std::size_t vehicleBytes = count * sizeof(Vehicle);
    const std::size_t leaderBytes = count * sizeof(int);

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceVehiclesInput_), vehicleBytes), "cudaMalloc input vehicles")) return false;

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceVehiclesOutput_), vehicleBytes), "cudaMalloc output vehicles"))
    {
        release();
        return false;
    }

    if (!checkCuda(cudaMalloc(reinterpret_cast<void**>(&deviceLeaderIndices_), leaderBytes), "cudaMalloc leader indices"))
    {
        release();
        return false;
    }

    capacity_ = count;
    return true;
}

bool CudaVehicleUpdater::update(std::vector<Vehicle>& vehicles, const std::vector<int>& leaderIndices, float deltaTime,
                                float roadLength, float vehicleLength, const IDMParameters& idmParameters)
{
    if (vehicles.empty()) return true;

    if (vehicles.size() != leaderIndices.size())
    {
        std::cerr << "[CUDA ERROR] Vehicle and leader arrays have different sizes.\n";
        return false;
    }

    if (!ensureCapacity(vehicles.size())) return false;

    const std::size_t vehicleBytes = vehicles.size() * sizeof(Vehicle);
    const std::size_t leaderBytes = leaderIndices.size() * sizeof(int);

    if (!checkCuda(cudaMemcpy(deviceVehiclesInput_, vehicles.data(), vehicleBytes, cudaMemcpyHostToDevice),
                   "cudaMemcpy vehicles HostToDevice")) return false;

    if (!checkCuda(cudaMemcpy(deviceLeaderIndices_, leaderIndices.data(), leaderBytes, cudaMemcpyHostToDevice),
                   "cudaMemcpy leaders HostToDevice")) return false;

    constexpr int threadsPerBlock = 256;
    const int vehicleCount = static_cast<int>(vehicles.size());
    const int blockCount = (vehicleCount + threadsPerBlock - 1) / threadsPerBlock;

    updateVehiclesKernel<<<blockCount, threadsPerBlock>>>(
        deviceVehiclesInput_, deviceVehiclesOutput_, deviceLeaderIndices_, vehicleCount, deltaTime,
        roadLength, vehicleLength, idmParameters
    );

    if (!checkCuda(cudaGetLastError(), "updateVehiclesKernel launch")) return false;
    if (!checkCuda(cudaDeviceSynchronize(), "updateVehiclesKernel execution")) return false;

    if (!checkCuda(cudaMemcpy(vehicles.data(), deviceVehiclesOutput_, vehicleBytes, cudaMemcpyDeviceToHost),
                   "cudaMemcpy vehicles DeviceToHost")) return false;

    return true;
}

void CudaVehicleUpdater::release()
{
    if (deviceVehiclesInput_) cudaFree(deviceVehiclesInput_);
    if (deviceVehiclesOutput_) cudaFree(deviceVehiclesOutput_);
    if (deviceLeaderIndices_) cudaFree(deviceLeaderIndices_);

    deviceVehiclesInput_ = nullptr;
    deviceVehiclesOutput_ = nullptr;
    deviceLeaderIndices_ = nullptr;
    capacity_ = 0;
}
