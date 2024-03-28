#include "opendbc/can/logger.h"

#ifdef SWAGLOG
// cppcheck-suppress preprocessorErrorDirective
#include SWAGLOG
#define LOGD(fmt, ...) cloudlog(CLOUDLOG_DEBUG, fmt, ## __VA_ARGS__)
#define LOG(fmt, ...) cloudlog(CLOUDLOG_INFO, fmt, ## __VA_ARGS__)
#define LOGW(fmt, ...) cloudlog(CLOUDLOG_WARNING, fmt, ## __VA_ARGS__)
#define LOGE(fmt, ...) cloudlog(CLOUDLOG_ERROR, fmt, ## __VA_ARGS__)
#else
#define LOGD(fmt, ...) (void)0
#define LOG(fmt, ...) (void)0
#define LOGW(fmt, ...) (void)0
#define LOGE(fmt, ...) (void)0
#endif


NoSpamLogger::~NoSpamLogger() {
  // flush messages
  for (auto it = messages.begin(); it != messages.end(); ++it) {
    if (it->second.count > 0) {
      log(it);
    }
  }
}

std::string NoSpamLogger::format_string(const char *fmt, ...) {
  constexpr int max_size = 1024;
  char buffer[max_size];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, max_size, fmt, args);
  va_end(args);
  return std::string(buffer);
}

void NoSpamLogger::log(NoSpamLogger::Type type, const std::string &msg) {
  static NoSpamLogger log;

  struct timespec t;
  clock_gettime(CLOCK_BOOTTIME, &t);
  double ts = t.tv_sec * 1000.0 + t.tv_nsec * 1e-6;

  auto [it, inserted] = log.messages.insert({msg, {.type = type, .ts = ts, .count = 0}});
  if (!inserted && (ts - it->second.ts) < log.threshold) {
    ++it->second.count;
    return;
  }

  log.log(it);
  it->second.ts = ts;
  it->second.count = 0;
}

void NoSpamLogger::log(std::unordered_map<std::string, Stat>::iterator it) {
   switch (it->second.type) {
    case Type::Debug: LOGD("%s [skipped:%u]", it->first.c_str(), it->second.count); break;
    case Type::Info: LOG("%s [skipped:%u]", it->first.c_str(), it->second.count); break;
    case Type::Warning: LOGW("%s [skipped:%u]", it->first.c_str(), it->second.count); break;
    case Type::Error: LOGE("%s [skipped:%u]", it->first.c_str(), it->second.count); break;
    default: break;
  }
}
