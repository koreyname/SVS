#pragma once

#include <csignal>

namespace ORB_SLAM3
{
namespace utils
{
namespace detail
{
static volatile std::sig_atomic_t g_shouldQuit = 0;

inline void HandleSignal(int)
{
    g_shouldQuit = 1;
}
}

inline void RegisterSignalHandlers()
{
    detail::g_shouldQuit = 0;
    std::signal(SIGINT, detail::HandleSignal);
    std::signal(SIGTERM, detail::HandleSignal);
}

inline bool ShouldQuit()
{
    return detail::g_shouldQuit != 0;
}
}
}
