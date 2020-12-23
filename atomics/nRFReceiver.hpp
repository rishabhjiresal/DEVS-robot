#ifndef BOOST_SIMULATION_NRF_RECEIVER_HPP
#define BOOST_SIMULATION_NRF_RECEIVER_HPP

#include <stdio.h>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/message_bag.hpp>
#include <limits>
#include <math.h>
#include <assert.h>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <algorithm>
#include <limits>
#include <random>

using namespace cadmium;
using namespace std;




#ifdef RT_ARM_MBED
#include <cadmium/real_time/arm_mbed/embedded_error.hpp>
#include "../mbed-os/mbed.h"
#include "../drivers/NRF_DRIVER.hpp"

struct nRFReceiver_defs {
  struct out : public out_port<char> { };
};

template<typename TIME>
class nRFReceiver {
  using defs=nRFReceiver_defs;
public:
  TIME pollingRate;
  nRFReceiver() noexcept {
    MBED_ASSERT(false);
  }

  nRFReceiver(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName irq) noexcept {
    new (this) nRFReceiver(mosi, miso, sck, csn, ce, irq, TIME("00:00:00:050"));
  }

  nRFReceiver(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName irq, TIME rate) noexcept {
    pollingRate = rate;
    state.nrf = new drivers::NRF_DRIVER(mosi, miso, sck, csn, ce, irq);
    state.nrf->PowerUp();
  }

  struct state_type {
    char receivedData[1];
    char last;
    drivers::NRF_DRIVER* nrf;
  }; state_type state;

  using input_ports=std::tuple<>;
  using output_ports=std::tuple<typename defs::out>;

  void internal_transition() {
    state.last = state.receivedData[0];
    state.nrf->ReceiveData(state.receivedData, 1);
  }

  void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
    MBED_ASSERT(false);

  }

  void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
      internal_transition();
      external_transition(TIME(), std::move(mbs));
  }

  typename make_message_bags<output_ports>::type output() const {
    typename make_message_bags<output_ports>::type bags;
    if(state.last != state.receivedData[0]) {
      char OUT = state.receivedData[0];
      get_messages<typename defs::out>(bags).push_back(OUT);
    }
    }

    TIME time_advance() const {
      return pollingRate;
    }

    friend std::ostringstream& operator<<(std::ostringstream& os, const typename nRFReceiver<TIME>::state_type& i) {
   os << "nRF Received: " << i.receivedFromController[0];
   return os;
  }
};

#else
  #include <cadmium/io/iestream.hpp>
  using namespace cadmium;
  using namespace std;

  struct nRFReceiver_defs {
  struct out : public out_port<char> { };
};

  template<typename TIME>
  class nRFReceiver : public iestream_input<char,TIME, nRFReceiver_defs>{
    public:
      nRFReceiver() = default;
      nRFReceiver(const char* file_path) : iestream_input<char,TIME, nRFReceiver_defs>(file_path) {}
      nRFReceiver(const char* file_path, TIME t) : iestream_input<char,TIME, nRFReceiver_defs>(file_path) {}
  };
#endif
#endif
