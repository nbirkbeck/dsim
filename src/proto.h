#ifndef DSIM_PROTO_H_
#define DSIM_PROTO_H_ 1

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <glog/logging.h>
#include <fcntl.h>

class Proto {
public:
  template <class T>
  static bool ReadProto(const std::string& filename, T* proto) {
    using namespace google::protobuf;
    if (filename.size() == 0) {
      LOG(ERROR) << "Need at least one argument (the input file)";
      return false;
    }
    const int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
      LOG(ERROR) << "Unable to open input file: " << filename;
      return false;
    }
    io::FileInputStream fstream(fd);
    if (!TextFormat::Parse(&fstream, proto)) {
      LOG(ERROR) << "Unable to parse input:" << filename;
      return false;
    }
    return true;
  }
};

#endif  // DSIM_PROTO_H_
