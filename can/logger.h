#pragma once

#include <string>
#include <unordered_map>

class NoSpamLogger {

public:
  enum Type {Debug, Info, Warning, Error};
  struct Stat {
    double ts = 0;
    uint32_t count = 0;
  };
  inline static uint32_t threshold = 500;  // 500 ms

  static void log(Type type, const std::string &msg);
  static std::string format_string(const char *fmt, ...);

private:
  static std::unordered_map<std::string, Stat> logs;
};

#define NO_SPAM_LOGD(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Debug, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
#define NO_SPAM_LOG(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Info, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
#define NO_SPAM_LOGW(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Warning, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
#define NO_SPAM_LOGE(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Error, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
