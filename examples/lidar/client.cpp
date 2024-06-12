#include <vsomeip/enumeration_types.hpp>
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
#include <csignal>
#endif
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <vsomeip/handler.hpp>
#include <vsomeip/vsomeip.hpp>

#include "lidar_ids.h"

class client_sample {
public:
  client_sample()
      : app_(vsomeip::runtime::get()->create_application()) {}

  bool init() {
    if (!app_->init()) {
      std::cerr << "Couldn't initialize application" << std::endl;
      return false;
    }
    std::cout << "Client request_service[" << std::hex << kPointCloudServiceId
              << vsomeip::ANY_INSTANCE << "]" << std::endl;

    app_->register_message_handler(
        vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE, vsomeip::ANY_METHOD,
        std::bind(&client_sample::on_message, this, std::placeholders::_1));

    app_->register_availability_handler(
        kPointCloudServiceId, vsomeip::ANY_INSTANCE,
        std::bind(&client_sample::on_availability, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    app_->register_reboot_notification_handler(
        std::bind(&client_sample::reboot_cb, this, std::placeholders::_1));

    app_->request_service(kPointCloudServiceId, kLeftInstanceId);
    subscribe_event();
    return true;
  }

  void start() { app_->start(); }
  void subscribe_event();
  void reboot_cb(const vsomeip_v3::ip_address_t &);
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
  /*
   * Handle signal to shutdown
   */
  void stop() {
    app_->clear_all_handler();
    app_->unsubscribe(vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE,
                      kEventGroupId);
    app_->release_event(vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE, kEventId);
    app_->release_service(vsomeip::ANY_SERVICE, vsomeip::ANY_INSTANCE);
    app_->stop();
  }
#endif

  void on_availability(vsomeip::service_t _service,
                       vsomeip::instance_t _instance, bool _is_available) {
    std::cout << std::hex << "Service [" << _service << "." << _instance << "] _is_available:"
              << _is_available  << std::endl;
    if (!_is_available) {
      //subscribe_event();
    }
  }

  void on_message(const std::shared_ptr<vsomeip::message> &_response) {

    std::stringstream its_message;
    its_message << "on_msg type:" << (int)_response->get_message_type() << std::hex
                << ", [" <<  _response->get_service() << "." << _response->get_instance() << "." 
                <<  _response->get_method() << "] to Client/Session ["
                << _response->get_client() << "/" <<  _response->get_session() << "] = ";
    std::shared_ptr<vsomeip::payload> its_payload = _response->get_payload();
    its_message << "(" << std::dec << its_payload->get_length() << ") ";
    std::cout << its_message.str() << std::endl;

  }

private:
  std::shared_ptr<vsomeip::application> app_;
};

#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
client_sample *its_sample_ptr(nullptr);
void handle_signal(int _signal) {
  if (its_sample_ptr != nullptr && (_signal == SIGINT || _signal == SIGTERM))
    its_sample_ptr->stop();
}
#endif
void client_sample::subscribe_event() {
  std::set<vsomeip::eventgroup_t> its_groups;
  its_groups.insert(kEventGroupId);
  app_->request_event(kPointCloudServiceId, kLeftInstanceId, kEventId,
                      its_groups, vsomeip::event_type_e::ET_EVENT);
  // ET_FIELD ::If the packet content is the same, do not forward it
  // ET_EVENT ::If the packet content is the same, also forward it
  app_->subscribe(kPointCloudServiceId, kLeftInstanceId, kEventGroupId,
                  kCurrentMajor);
}
void client_sample::reboot_cb(const vsomeip_v3::ip_address_t &) {
  std::cerr << __LINE__ << " -------------- reboot_cb =================== " << std::endl;
}

int main(int argc, char **argv) {

  (void)argc;
  (void)argv;

  client_sample its_sample;
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
  its_sample_ptr = &its_sample;
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);
#endif
  if (its_sample.init()) {
    its_sample.start();
    return 0;
  } else {
    return 1;
  }
}
