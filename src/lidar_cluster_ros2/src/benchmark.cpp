#include "benchmark.hpp"
#include <iostream>
#include <limits>

namespace benchmark {

void Timer::start(const std::string &name, bool verbose) {
    this->verbose = verbose;
    if (!this->verbose) return; // if verbose is false, don't start the timer.
    this->start_time = std::chrono::high_resolution_clock::now();
    this->name = name;
}

void Timer::finish() {
    if (!verbose) return; // if verbose is false, don't finish the timer.
    this->end_time = std::chrono::high_resolution_clock::now();
    this->elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    this->times.push_back(elapsed_time);

    if (elapsed_time < min) min = elapsed_time;
    if (elapsed_time > max) max = elapsed_time;
    avg = (avg * (times.size() - 1) + elapsed_time) / times.size();

    std::cout << name << " frametime: " << elapsed_time << " ms, Min: " << min 
              << " ms, Max: " << max << " ms, Avg: " << avg << " ms" << std::endl;
}

} // namespace benchmark
