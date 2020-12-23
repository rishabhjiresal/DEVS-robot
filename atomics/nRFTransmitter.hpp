#ifndef BOOST_SIMULATION_NRF_TRANSMITTER_HPP
#define BOOST_SIMULATION_NRF_TRANSMITTER_HPP

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


struct nRFTransmitter_defs {
  struct in : public in_port<char> { };
};

#ifdef RT_ARM_MBED
#include <cadmium/real_time/arm_mbed/embedded_error.hpp>
#include "../mbed-os/mbed.h"
#include "../drivers/NRF_DRIVER.hpp"


template<typename TIME>
class nRFTransmitter {
  using defs=nRFTransmitter_defs;
public:
  nRFTransmitter() noexcept {
    MBED_ASSERT(false);
  }

  nRFTransmitter(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName irq) noexcept {
    state.nrf = new drivers::NRF_DRIVER(mosi, miso, sck, csn, ce, irq);
    state.nrf->PowerUp();
  }

  struct state_type {
    char receivedFromController[1];
    drivers::NRF_DRIVER* nrf;
  }; state_type state;

  using input_ports=std::tuple<typename defs::in>;
  using output_ports=std::tuple<>;

  void internal_transition() {
    state.active = false;
  }

  void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
    for(const auto &x : get_messages<typename defs::in>(mbs)) {
      state.receivedFromController[0] = x;
    }
    state.active = true;
  }

  void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
      internal_transition();
      external_transition(TIME(), std::move(mbs));
  }

  typename make_message_bags<output_ports>::type output() const {
    typename make_message_bags<output_ports>::type bags;
    int temp;
    temp = state.nrf->TransmitData(receivedFromController, 1);
    }

	TIME time_advance() const {
    if(state.active) {
      return TIME("00:00:00");
    }
    return std::numeric_limits<TIME>::infinity();
  }

  friend std::ostringstream& operator<<(std::ostringstream& os, const typename nRFTransmitter<TIME>::state_type& i) {
 os << "nRF Transmitted: " << i.receivedFromController[0];
 return os;
}
};

#else

#include <cadmium/io/oestream.hpp>
using namespace cadmium;
using namespace std;

template<typename TIME>
class nRFTransmitter : public oestream_output<char, TIME, nRFTransmitter_defs> {
public:
  nRFTransmitter() = default;
  nRFTransmitter(const char* file_path) : oestream_output<char,TIME, nRFTransmitter_defs>(file_path){}
  nRFTransmitter(const char* file_path, TIME t) : oestream_output<char, TIME, nRFTransmitter_defs>(file_path){}
};


#endif
#endif