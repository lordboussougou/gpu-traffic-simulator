#include "cuda_test.cuh"

#include <cuda_runtime.h>

#include <iostream>
#include <vector>

namespace
{
bool checkCuda(cudaError_t error, const char* operation)
{
    if (error == cudaSuccess)
    {
        return true;
    }

    std::cerr
        << "[CUDA ERROR] "
        << operation
        << ": "
        << cudaGetErrorString(error)
        << '\n';

    return false;
}

__global__ void incrementKernel(int* values, int count)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index < count)
    {
        values[index] += 1;
    }
}
}

bool runCudaTest()
{
    std::cout << "=== CUDA test ===\n";

    int deviceCount = 0;

    if (!checkCuda(cudaGetDeviceCount(&deviceCount), "cudaGetDeviceCount"))
    {
        return false;
    }

    if (deviceCount == 0)
    {
        std::cerr << "No CUDA-compatible GPU detected.\n";
        return false;
    }

    cudaDeviceProp deviceProperties{};

    if (!checkCuda(
            cudaGetDeviceProperties(&deviceProperties, 0),
            "cudaGetDeviceProperties"))
    {
        return false;
    }

    std::cout << "CUDA device: " << deviceProperties.name << '\n';

    constexpr int elementCount = 1024;
    constexpr int threadsPerBlock = 256;
    constexpr int blockCount =
        (elementCount + threadsPerBlock - 1) / threadsPerBlock;

    std::vector<int> hostValues(elementCount, 0);

    int* deviceValues = nullptr;
    const std::size_t bytes = hostValues.size() * sizeof(int);

    if (!checkCuda(
            cudaMalloc(
                reinterpret_cast<void**>(&deviceValues),
                bytes),
            "cudaMalloc"))
    {
        return false;
    }

    if (!checkCuda(
            cudaMemcpy(
                deviceValues,
                hostValues.data(),
                bytes,
                cudaMemcpyHostToDevice),
            "cudaMemcpy HostToDevice"))
    {
        cudaFree(deviceValues);
        return false;
    }

    incrementKernel<<<blockCount, threadsPerBlock>>>(
        deviceValues,
        elementCount);

    if (!checkCuda(cudaGetLastError(), "incrementKernel launch"))
    {
        cudaFree(deviceValues);
        return false;
    }

    if (!checkCuda(cudaDeviceSynchronize(), "cudaDeviceSynchronize"))
    {
        cudaFree(deviceValues);
        return false;
    }

    if (!checkCuda(
            cudaMemcpy(
                hostValues.data(),
                deviceValues,
                bytes,
                cudaMemcpyDeviceToHost),
            "cudaMemcpy DeviceToHost"))
    {
        cudaFree(deviceValues);
        return false;
    }

    if (!checkCuda(cudaFree(deviceValues), "cudaFree"))
    {
        return false;
    }

    for (const int value : hostValues)
    {
        if (value != 1)
        {
            std::cerr << "Result: FAILURE\n";
            return false;
        }
    }

    std::cout << "Elements: " << elementCount << '\n';
    std::cout << "Threads per block: " << threadsPerBlock << '\n';
    std::cout << "Blocks: " << blockCount << '\n';
    std::cout << "Result: SUCCESS\n\n";

    return true;
}
