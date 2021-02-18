#include "udp_driver.hpp"

#include <iostream>

namespace autoware
{
namespace drivers
{

UdpDriver::UdpDriver(const IoContext &ctx) : m_ctx(ctx) {

}

UdpDriver::~UdpDriver() {
    std::cout << "[UdpDriver::~UdpDriver] INFO => Destructing..." << std::endl;
}

void UdpDriver::init_sender(const std::string &ip, uint16_t port) {
    m_sender.reset(new UdpSocket(m_ctx, ip, port));
}

void UdpDriver::init_receiver(const std::string &ip, uint16_t port) {
    m_receiver.reset(new UdpSocket(m_ctx, ip, port));
}

std::shared_ptr<UdpSocket> UdpDriver::sender() const {
    return m_sender;
}
std::shared_ptr<UdpSocket> UdpDriver::receiver() const {
    return m_receiver;
}

}  // namespace drivers
}  // namespace autoware
