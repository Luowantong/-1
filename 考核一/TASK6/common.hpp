#pragma once
#include <chrono>
#include<cstdint>
struct Image {
    std::chrono::steady_clock::time_point timestamp;
    uint64_t sequence_id =0;
};

struct InferResult {
    std::chrono::steady_clock::time_point timestamp;
    uint64_t sequence_id =0;
};
