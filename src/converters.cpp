#include "converters.hpp"

namespace autoware {
namespace msgs {

void convertFromRos2Message(const std_msgs::msg::Int8::SharedPtr &in, MutSocketBuffer &out) {
    out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Int16::SharedPtr &in, MutSocketBuffer &out) {
    out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Int32::SharedPtr &in, MutSocketBuffer &out) {
    out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Int64::SharedPtr &in, MutSocketBuffer &out) {
    out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int8 &out) {
    out.data = *boost::asio::buffer_cast<int8_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int16 &out) {
    out.data = *boost::asio::buffer_cast<int16_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int32 &out) {
    out.data = *boost::asio::buffer_cast<int32_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int64 &out) {
    out.data = *boost::asio::buffer_cast<int64_t *>(in);
}

}
}

