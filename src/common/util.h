#pragma once

#include <bit>
#include <filesystem>
#include <inttypes.h>
#include <string>

namespace util {
void init();
const std::filesystem::path& getSpaceCalibratorInstallDir();
const std::filesystem::path& getSpaceCalibratorLangsDir();
const std::filesystem::path& getSpaceCalibratorImagesDir();
const std::filesystem::path& getSpaceCalibratorConfigDir();

const std::filesystem::path& getSpaceCalibratorLogsDir();
const std::filesystem::path& getSpaceCalibratorDumpsDir();
const std::filesystem::path& getSpaceCalibratorConfigPath();
}

#ifndef _DEBUG
#define ASSERT(cond, msg)
#else
#include <assert.h>
#define ASSERT(cond, msg) assert(cond)
#endif

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

// for base station comms
inline uint16_t make_be16(uint16_t value)
{
    if constexpr (std::endian::native == std::endian::big) {
        return value;
    } else {
        return (uint16_t)((value >> 8) | (value << 8));
    }
}
