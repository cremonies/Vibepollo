/**
 * @file src/platform/windows/local_input_monitor.cpp
 * @brief See local_input_monitor.h for the design rationale.
 */
#ifdef _WIN32

  #include "local_input_monitor.h"

  #include "src/logging.h"
  #include "src/rtsp.h"

  #include <algorithm>
  #include <atomic>
  #include <chrono>
  #include <condition_variable>
  #include <cstdlib>
  #include <mutex>
  #include <thread>
  #include <windows.h>
  #include <xinput.h>

  #ifdef _MSC_VER
    // MinGW links against xinput9_1_0 via CMake (windows.cmake); MSVC builds
    // need the equivalent pulled in here.
    #pragma comment(lib, "xinput9_1_0.lib")
  #endif

namespace local_input_monitor {

  namespace {

    constexpr auto POLL_INTERVAL = std::chrono::milliseconds(150);

    // Input newer than the host's last delivery by no more than this is
    // attributed to the host. Covers the delay between the host sending an
    // event and Windows stamping it as the last input (driver/HID latency and
    // the coarse timer granularity of GetLastInputInfo).
    constexpr std::uint64_t HOST_INPUT_TOLERANCE_MS = 500;

    // GetTickCount64() of the most recent input attributed to a person; 0 = none.
    std::atomic<std::uint64_t> g_last_physical_tick {0};
    std::atomic<bool> g_running {false};
    std::atomic<bool> g_warned_no_input_info {false};

    void
      mark_physical_input(std::uint64_t tick) {
      auto current = g_last_physical_tick.load(std::memory_order_relaxed);
      while (tick > current && !g_last_physical_tick.compare_exchange_weak(current, tick, std::memory_order_relaxed)) {
      }
    }

    // -- Keyboard / mouse ------------------------------------------------------
    //
    // GetLastInputInfo() gives only the tick count of the last input in this
    // session. It does not say what the input was or who produced it, so it is
    // attributed by comparison with the host's own deliveries (note_host_input).
    void
      sample_keyboard_mouse() {
      LASTINPUTINFO info {};
      info.cbSize = sizeof(info);
      if (!GetLastInputInfo(&info)) {
        if (!g_warned_no_input_info.exchange(true, std::memory_order_relaxed)) {
          BOOST_LOG(warning) << "local_input_monitor: GetLastInputInfo failed (err=" << GetLastError()
                             << "); keyboard/mouse activity will not be detected";
        }
        return;
      }

      // dwTime is a 32-bit tick count that wraps; the subtraction below is
      // wrap-safe, and widening back to 64 bits puts it on the same scale as
      // note_host_input().
      const std::uint64_t now = GetTickCount64();
      const DWORD idle_ms = static_cast<DWORD>(now) - info.dwTime;
      const std::uint64_t input_tick = now - idle_ms;

      const auto host_tick = detail::last_host_input_tick.load(std::memory_order_relaxed);
      if (host_tick != 0 && input_tick <= host_tick + HOST_INPUT_TOLERANCE_MS) {
        return;  // Attributable to the host's own delivery.
      }

      mark_physical_input(input_tick);
    }

    // -- Controller (XInput) polling -------------------------------------------
    //
    // There is no way to tell a ViGEm/VHF virtual pad from a physical one when
    // polling XInput, and no supported way to learn which slot a virtual pad
    // was given. So controller activity only counts while no streaming session
    // exists, i.e. nothing host-managed could be occupying a slot. This is
    // checked on every poll since a session can start or end at any time.
    //
    // Detection compares the CURRENT raw state with the deadzone/threshold
    // constants Microsoft documents for XInput, not whether dwPacketNumber
    // changed: the packet number is bumped by analog noise and wireless
    // telemetry even when nobody touches the pad.
    void
      poll_controllers_once() {
      if (rtsp_stream::session_count() != 0) {
        return;
      }

      for (DWORD i = 0; i < XUSER_MAX_COUNT; ++i) {
        XINPUT_STATE state {};
        if (XInputGetState(i, &state) != ERROR_SUCCESS) {
          continue;
        }

        const XINPUT_GAMEPAD &pad = state.Gamepad;

        // Buttons are discrete; any bit set is a genuine press.
        bool active = pad.wButtons != 0;

        if (!active) {
          auto beyond_deadzone = [](SHORT axis, SHORT deadzone) {
            return std::abs(static_cast<int>(axis)) > deadzone;
          };
          active = beyond_deadzone(pad.sThumbLX, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) ||
                   beyond_deadzone(pad.sThumbLY, XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) ||
                   beyond_deadzone(pad.sThumbRX, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE) ||
                   beyond_deadzone(pad.sThumbRY, XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE) ||
                   pad.bLeftTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD ||
                   pad.bRightTrigger > XINPUT_GAMEPAD_TRIGGER_THRESHOLD;
        }

        if (active) {
          mark_physical_input(GetTickCount64());
        }
      }
    }

    // -- Polling thread --------------------------------------------------------

    std::thread g_thread;
    std::mutex g_stop_mutex;
    std::condition_variable g_stop_cv;
    bool g_stop = false;

    void
      poll_thread_main() {
      std::unique_lock lock {g_stop_mutex};
      while (!g_stop) {
        lock.unlock();
        sample_keyboard_mouse();
        poll_controllers_once();
        lock.lock();
        g_stop_cv.wait_for(lock, POLL_INTERVAL, [] {
          return g_stop;
        });
      }
    }

    class deinit_t: public platf::deinit_t {
    public:
      ~deinit_t() override {
        if (!g_running.exchange(false, std::memory_order_acq_rel)) {
          return;
        }
        {
          std::lock_guard lock {g_stop_mutex};
          g_stop = true;
        }
        g_stop_cv.notify_all();
        if (g_thread.joinable()) {
          g_thread.join();
        }
        BOOST_LOG(info) << "local_input_monitor: stopped";
      }
    };

  }  // namespace

  std::unique_ptr<platf::deinit_t>
    start() {
    if (g_running.exchange(true, std::memory_order_acq_rel)) {
      BOOST_LOG(warning) << "local_input_monitor: start() called while already running";
      return std::make_unique<deinit_t>();
    }
    {
      std::lock_guard lock {g_stop_mutex};
      g_stop = false;
    }
    g_last_physical_tick.store(0, std::memory_order_relaxed);
    g_thread = std::thread(poll_thread_main);
    BOOST_LOG(info) << "local_input_monitor: started";
    return std::make_unique<deinit_t>();
  }

  std::chrono::milliseconds
    time_since_local_input() {
    // Sample now so the answer is exact rather than up to one poll interval old.
    sample_keyboard_mouse();

    const auto last = g_last_physical_tick.load(std::memory_order_relaxed);
    if (last == 0) {
      return std::chrono::milliseconds::max();
    }
    const std::uint64_t now = GetTickCount64();
    return std::chrono::milliseconds(now > last ? now - last : 0);
  }

  bool
    locally_active(std::chrono::milliseconds threshold) {
    return time_since_local_input() <= threshold;
  }

}  // namespace local_input_monitor

#endif  // _WIN32
