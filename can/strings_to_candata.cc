#include <capnp/serialize.h>

#include <string>
#include <vector>

#include "cereal/gen/cpp/log.capnp.h"
#include "opendbc/can/common.h"

void strings_to_candata(const std::vector<std::string> &strings, std::vector<CanData> &can_data, bool sendcan) {
  kj::Array<capnp::word> aligned_buf;
  can_data.reserve(strings.size());
  for (const auto &s : strings) {
    const size_t buf_size = (s.length() / sizeof(capnp::word)) + 1;
    if (aligned_buf.size() < buf_size) {
      aligned_buf = kj::heapArray<capnp::word>(buf_size);
    }
    memcpy(aligned_buf.begin(), s.data(), s.length());

    // extract the messages
    capnp::FlatArrayMessageReader cmsg(aligned_buf.slice(0, buf_size));
    cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

    auto &can = can_data.emplace_back();
    can.nanos = event.getLogMonoTime();

    auto cans = sendcan ? event.getSendcan() : event.getCan();
    can.frames.reserve(cans.size());
    for (const auto cmsg : cans) {
      auto &frame = can.frames.emplace_back();
      frame.src = cmsg.getSrc();
      frame.address = cmsg.getAddress();
      auto dat = cmsg.getDat();
      frame.dat.resize(dat.size());
      memcpy(frame.dat.data(), dat.begin(), dat.size());
    }
  }
}
