/* 
 * Copyright (C) 2020 fleroviux
 * 
 * Licensed under GPLv3 or any later version.
 * Refer to the included LICENSE file.
 */

#pragma once

#include <cstdlib>
#include <string>
#include <third_party/fmtlib/include/fmt/format.h>

namespace common::logger {

enum class Level {
  Trace,
  Debug,
  Info,
  Warn,
  Error,
  Fatal
};

void append(Level level,
            const char* file,
            int line,
            std::string const& message);

#define LOG_TRACE(message, ...) common::logger::append(common::logger::Level::Trace, __FILE__, __LINE__, \
                                                       fmt::format(message, ## __VA_ARGS__));

#define LOG_DEBUG(message, ...) common::logger::append(common::logger::Level::Debug, __FILE__, __LINE__, \
                                                       fmt::format(message, ## __VA_ARGS__));

#define LOG_INFO(message, ...) common::logger::append(common::logger::Level::Info, __FILE__, __LINE__, \
                                                      fmt::format(message, ## __VA_ARGS__));

#define LOG_WARN(message, ...) common::logger::append(common::logger::Level::Warn, __FILE__, __LINE__, \
                                                      fmt::format(message, ## __VA_ARGS__));

#define LOG_ERROR(message, ...) common::logger::append(common::logger::Level::Error, __FILE__, __LINE__, \
                                                       fmt::format(message, ## __VA_ARGS__));

#define LOG_FATAL(message, ...) common::logger::append(logger::detail::Level::Fatal, __FILE__, __LINE__, \
                                                       fmt::format(message, ## __VA_ARGS__));

#define ASSERT(condition, message, ...) if (!(condition)) { LOG_ERROR(message, ## __VA_ARGS__); std::exit(-1); }

#define ASSERT_UNREACHABLE ASSERT(false, "reached supposedly unreachable code.");

} // namespace common::logger
