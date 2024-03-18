#pragma once
#include <chrono>
#include <iostream>
namespace common
{
  class Timer
  {
  private:
    std::chrono::high_resolution_clock::time_point start, end;
    std::chrono::duration<double, std::milli>      time_consumed_ms;

  public:
    double time_consumed_ms_double;
    Timer() = default;

    ~Timer() = default;

    void tic()
    {
      start = std::chrono::high_resolution_clock::now();
    }

    void toc()
    {
      end                     = std::chrono::high_resolution_clock::now();
      time_consumed_ms        = end - start;
      time_consumed_ms_double = time_consumed_ms.count();
    }

    void show()
    {
      std::cout << "Time: " << time_consumed_ms.count() << "\t\tms" << std::endl;
    }
  };
}  // namespace common