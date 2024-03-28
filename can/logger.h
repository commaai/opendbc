#pragma once

#include <string>
#include <unordered_map>

#include <cassert>
class NoSpamLogger {

public:
  enum Type {Debug, Info, Warning, Error};
  ~NoSpamLogger();
  static void log(Type type, const std::string &msg);
  static std::string format_string(const char *fmt, ...);

private:
  struct Stat {
    Type type;
    double ts = 0;
    uint32_t count = 0;
  };
  const uint32_t threshold = 500;  // 500 ms
  std::unordered_map<std::string, Stat> messages;
  void log(std::unordered_map<std::string, Stat>::iterator it);
};

#define NO_SPAM_LOGD(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Debug, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
#define NO_SPAM_LOG(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Info, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
#define NO_SPAM_LOGW(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Warning, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
#define NO_SPAM_LOGE(fmt, ...) NoSpamLogger::log(NoSpamLogger::Type::Error, NoSpamLogger::format_string(fmt, ##__VA_ARGS__))
