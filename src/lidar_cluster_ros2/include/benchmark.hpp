#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include <chrono>
#include <vector>
#include <string>

// Benchmarking namespace, for various benchmarking tools
namespace benchmark {

// Class to initialize and finish a timer, for benchmarking purposes
class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;
    long elapsed_time = 0;
    long min = std::numeric_limits<long>::max();
    long max = std::numeric_limits<long>::min();
    double avg = 0.0;
    std::vector<long> times;
    std::string name;
    bool verbose = false;

public:
    // Start the timer. First argument is the name of the timer, second argument is an optional bool, and can stop the timer from starting if false
    void start(const std::string &name, bool verbose = false);

    // Finish the timer and print the results. 
    void finish();
};

} // namespace benchmark

#endif // BENCHMARK_HPP
