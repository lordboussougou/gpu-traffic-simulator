#pragma once

// Runs the first CUDA learning test.
//
// Returns true when:
// - a CUDA device is available,
// - memory allocations/transfers succeed,
// - the kernel executes,
// - and every result is correct.
bool runCudaTest();
