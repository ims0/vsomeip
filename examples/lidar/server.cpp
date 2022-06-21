#include "vsomeip/constants.hpp"
#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
#include <csignal>
#endif
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

#include <vsomeip/vsomeip.hpp>

#include "lidar_ids.h"

class service_sample {
public:
  service_sample(bool _use_tcp, uint32_t _cycle)
      : app_(vsomeip::runtime::get()->create_application("lidar")),
        use_tcp_(_use_tcp),
        cycle_(_cycle),
        is_registered_(false), blocked_(false), running_(true),
        is_offered_(false),
        offer_thread_(std::bind(&service_sample::run, this)),
        notify_thread_(std::bind(&service_sample::notify, this)) {}

  bool init() {
    std::lock_guard<std::mutex> its_lock(mutex_);

    if (!app_->init()) {
      std::cerr << "Couldn't initialize application" << std::endl;
      return false;
    }
    app_->register_state_handler(
        std::bind(&service_sample::on_state, this, std::placeholders::_1));

    std::set<vsomeip::eventgroup_t> its_groups;
    its_groups.insert(kEventGroupId);
    app_->offer_event(kPointCloudServiceId, kLeftInstanceId, kEventId,
                      its_groups);
    {
      std::lock_guard<std::mutex> its_lock(payload_mutex_);
      payload_ = vsomeip::runtime::get()->create_payload();
    }

    blocked_ = true;
    condition_.notify_one();
    return true;
  }

  void start() { app_->start(); }

#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
  /*
   * Handle signal to shutdown
   */
  void stop() {
    running_ = false;
    blocked_ = true;
    condition_.notify_one();
    notify_condition_.notify_one();
    app_->clear_all_handler();
    offer_thread_.join();
    notify_thread_.join();
    app_->stop();
  }
#endif
    void offer() {
        std::lock_guard<std::mutex> its_lock(notify_mutex_);
        app_->offer_service(kPointCloudServiceId, kLeftInstanceId, kCurrentMajor);
        is_offered_ = true;
        notify_condition_.notify_one();
    }

    void stop_offer() {
        app_->stop_offer_service(kPointCloudServiceId, kLeftInstanceId, kCurrentMajor);
        is_offered_ = false;
    }


  void on_state(vsomeip::state_type_e _state) {
    std::cout << "Application " << app_->get_name() << " is "
              << (_state == vsomeip::state_type_e::ST_REGISTERED
                      ? "registered."
                      : "deregistered.")
              << std::endl;

    if (_state == vsomeip::state_type_e::ST_REGISTERED) {
      if (!is_registered_) {
        is_registered_ = true;
      }
    } else {
      is_registered_ = false;
    }
  }
    void run() {
        std::unique_lock<std::mutex> its_lock(mutex_);
        while (!blocked_)
            condition_.wait(its_lock);

        bool is_offer(true);
        while (running_) {
            if (is_offer)
                offer();
            else
                stop_offer();

            for (int i = 0; i < 10 && running_; i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            //is_offer = !is_offer; 
        }
    }
  void notify() {
    std::shared_ptr<vsomeip::message> its_message =
        vsomeip::runtime::get()->create_request(use_tcp_);

    its_message->set_service(kPointCloudServiceId);
    its_message->set_instance(kLeftInstanceId);
    its_message->set_method(kEventId);

    vsomeip::byte_t its_data[10];
    uint32_t its_size = 1;

    while (running_) {
      std::unique_lock<std::mutex> notify_mutex(notify_mutex_);
      while (!is_offered_ && running_)
        notify_condition_.wait(notify_mutex);
      while (is_offered_ && running_) {
        if (its_size == sizeof(its_data))
          its_size = 1;

        for (uint32_t i = 0; i < its_size; ++i)
          its_data[i] = static_cast<uint8_t>(i);

        {
          std::lock_guard<std::mutex> its_lock(payload_mutex_);
          payload_->set_data(its_data, its_size);

          std::cout << "Setting event (Length=" << std::dec << its_size << ")."
                    << std::endl;
          app_->notify(kPointCloudServiceId, kLeftInstanceId, kEventId,
                       payload_);
        }

        its_size++;

        std::this_thread::sleep_for(std::chrono::milliseconds(cycle_));
      }
    }
  }

private:
  std::shared_ptr<vsomeip::application> app_;
  bool is_registered_;
  bool use_tcp_;
  uint32_t cycle_;

  std::mutex mutex_;
  std::condition_variable condition_;
  bool blocked_;
  bool running_;

  std::mutex notify_mutex_;
  std::condition_variable notify_condition_;
  bool is_offered_;

  std::mutex payload_mutex_;
  std::shared_ptr<vsomeip::payload> payload_;

  // blocked_ / is_offered_ must be initialized before starting the threads!
  std::thread offer_thread_;
  std::thread notify_thread_;
};

#ifndef VSOMEIP_ENABLE_SIGNAL_HANDLING
service_sample *its_sample_ptr;
void handle_signal(int _signal) {
  if (its_sample_ptr != nullptr && (_signal == SIGINT || _signal == SIGTERM))
    its_sample_ptr->stop();
}
#endif

int main(int argc, char **argv) {
    bool use_tcp = false;
    uint32_t cycle = 1000; // default 1s

    std::string tcp_enable("--tcp");
    std::string udp_enable("--udp");
    std::string cycle_arg("--cycle");

    for (int i = 1; i < argc; i++) {
        if (tcp_enable == argv[i]) {
            use_tcp = true;
            break;
        }
        if (udp_enable == argv[i]) {
            use_tcp = false;
            break;
        }

        if (cycle_arg == argv[i] && i + 1 < argc) {
            i++;
            std::stringstream converter;
            converter << argv[i];
            converter >> cycle;
        }
    }
  service_sample its_sample(use_tcp, cycle);
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
