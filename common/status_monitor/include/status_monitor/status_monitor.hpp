// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef STATUS_MONITOR__STATUS_MONITOR_HPP_
#define STATUS_MONITOR__STATUS_MONITOR_HPP_

#include <cerrno>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <unordered_map>

namespace common
{
class StatusMonitor
{
  struct Status
  {
    std::chrono::high_resolution_clock::time_point last_access;

    bool exited = false;

    template <typename Duration = void>
    auto elapsed_time_since_last_access() const
    {
      if constexpr (std::is_void_v<Duration>) {
        return std::chrono::high_resolution_clock::now() - last_access;
      } else {
        return std::chrono::duration_cast<Duration>(
          std::chrono::high_resolution_clock::now() - last_access);
      }
    }

    auto good() const { return exited or elapsed_time_since_last_access() < threshold; }

    explicit operator bool() const { return good(); }
  };

  static inline std::ofstream file;

  static inline std::unordered_map<std::thread::id, Status> statuses;

  static inline std::thread watchdog;

  static inline std::atomic_bool terminating;

  static inline std::chrono::milliseconds threshold = std::chrono::milliseconds(1000);

public:
  explicit StatusMonitor();

  ~StatusMonitor();

  auto good() const
  {
    return std::all_of(std::begin(statuses), std::end(statuses), [](auto && id_and_status) {
      return std::get<1>(id_and_status).good();
    });
  }

  static auto name() -> std::string
  {
#if _GNU_SOURCE
    return std::filesystem::path(program_invocation_name).filename();
#else
    return boost::lexical_cast<std::string>(std::this_thread::get_id())
#endif
  }

  auto touch(std::thread::id id)
  {
    // TODO MUTEX LOCK
    statuses[id].last_access = std::chrono::high_resolution_clock::now();
  }

  auto mark_as_exited(std::thread::id id)
  {
    if (auto iter = statuses.find(id); iter != std::end(statuses)) {
      std::get<1>(*iter).exited = true;
    }
  }
} static status_monitor;
}  // namespace common

#endif  // STATUS_MONITOR__STATUS_MONITOR_HPP_
