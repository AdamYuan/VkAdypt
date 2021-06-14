#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cinttypes>

constexpr const char *kAppName = "Adypt";
constexpr uint32_t kDefaultWidth = 1280, kDefaultHeight = 720;
constexpr uint32_t kMinWidth = 256, kMinHeight = 256;
constexpr uint32_t kMaxWidth = 3840, kMaxHeight = 3840;
constexpr uint32_t kFrameCount = 3;

constexpr float kCamNear = 1.0f / 512.0f, kCamFar = 4.0f;

constexpr uint32_t kFilenameBufSize = 512;

constexpr uint32_t kLogLimit = 256;

constexpr uint32_t kMinBounce = 2;
constexpr uint32_t kDefaultBounce = 4;
constexpr uint32_t kMaxBounce = 16;

constexpr float kDefaultSunRadiance = 5.0f;
constexpr float kMaxSunRadiance = 20.0f;

constexpr uint32_t kPTResultUpdateInterval = 10;

#endif
