/**
 * @file src/platform/windows/local_input_monitor.h
 * @brief Tracks whether a person is physically using the host, as distinct
 *        from input the host itself delivers for a connected client.
 *
 * This answers one question: "is someone at this machine right now?" so that
 * @ref nvhttp::launch can decline a new or competing connection instead of
 * silently taking over their display and input.
 *
 * Design: no input hooks, nothing that sees what was typed or clicked.
 *
 *  - Keyboard and mouse: only the timestamp of the most recent input in the
 *    current session is read, via GetLastInputInfo(). It carries no key codes,
 *    positions, or any content. Windows cannot say who produced that input, so
 *    the host reports its own deliveries with @ref note_host_input() (called
 *    at the points where remote keyboard/mouse/pen input is sent: SendInput,
 *    synthetic pointer injection, and the VHF keyboard/mouse driver). Input
 *    that is newer than the host's most recent delivery by more than a small
 *    tolerance is treated as physical.
 *
 *  - Controllers: XInput state is polled and counted only while no streaming
 *    session exists, since no virtual pad can then be attached. Detection uses
 *    the documented XInput deadzone/threshold constants to ignore analog noise.
 *
 * Known limits:
 *  - While remote input is flowing, physical keyboard/mouse input within the
 *    tolerance window after each delivery cannot be told apart from it.
 *  - Input injected by other software (macros, other remote-desktop tools) in
 *    the same session counts as local, because Windows reports it identically.
 *  - Controller activity is ignored while a session is live.
 */
#pragma once

#ifdef _WIN32

  #include "src/platform/common.h"

  #include <atomic>
  #include <chrono>
  #include <cstdint>
  #include <memory>
  #include <windows.h>

namespace local_input_monitor {

  namespace detail {
    /// GetTickCount64() value of the host's most recent keyboard/mouse/pen delivery; 0 = none yet.
    inline std::atomic<std::uint64_t> last_host_input_tick {0};
  }  // namespace detail

  /**
   * @brief Record that the host just delivered keyboard/mouse/pen input on behalf of a client.
   * @details Cheap and lock-free; safe to call from any thread on every event.
   */
  inline void
    note_host_input() noexcept {
    detail::last_host_input_tick.store(GetTickCount64(), std::memory_order_relaxed);
  }

  /**
   * @brief Start the local input monitor.
   *
   * Spawns a small polling thread (GetLastInputInfo plus XInput controllers).
   * Safe to call once per process lifetime; a second call while already
   * running logs a warning and returns a no-op guard.
   *
   * @return RAII guard that stops the thread when destroyed.
   */
  std::unique_ptr<platf::deinit_t>
    start();

  /**
   * @brief Time elapsed since the most recent confirmed physical input.
   *
   * @return Milliseconds since the last such input, or
   *         std::chrono::milliseconds::max() if none has been observed.
   */
  std::chrono::milliseconds
    time_since_local_input();

  /**
   * @brief Convenience wrapper: true if local input was seen within @p threshold.
   */
  bool
    locally_active(std::chrono::milliseconds threshold);

}  // namespace local_input_monitor

#endif  // _WIN32
