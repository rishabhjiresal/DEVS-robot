#ifndef BOOST_SIMULATION_NRF_CONTROLLER_HPP
#define BOOST_SIMULATION_NRF_CONTROLLER_HPP

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

struct nRFController_defs {
  struct LocationInput : public in_port<char> { };
  struct nRFReceiverInput : public in_port<char> { };

  struct TransmitterOut : public out_port<char> { };
  struct PositionControllerOut : public out_port<char> { };
};

template<typename TIME>
class nRFController {
  using defs=nRFController_defs;
public:
  nRFController() noexcept {

  }
  struct state_type {
    char currentLocation; //send to nRFTransmitter... received from RFIDController
    char ToGoPosition; //send to PositionController this is from receiver
    bool active;
  }; state_type state;

  using input_ports=std::tuple<typename defs::LocationInput, typename defs::nRFReceiverInput>;
  using output_ports=std::tuple<typename defs::TransmitterOut, typename defs::PositionControllerOut>;

  void internal_transition () {
    state.active = false;
  }

  void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs){

    for(const auto &x : get_messages<typename defs::nRFReceiverInput>(mbs)) {
			state.ToGoPosition = x;
		}
    for(const auto &x : get_messages<typename defs::LocationInput>(mbs)) {
			state.currentLocation = x;
		}
        printf("%c", state.ToGoPosition);


    state.active = true;
  }

  void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
    internal_transition();
    external_transition(TIME(), std::move(mbs));
  }

  typename make_message_bags<output_ports>::type output() const {
    typename make_message_bags<output_ports>::type bags;
    char OUT1 = state.currentLocation;
    char OUT2 = state.ToGoPosition;
    get_messages<typename defs::TransmitterOut>(bags).push_back(OUT1);
    get_messages<typename defs::PositionControllerOut>(bags).push_back(OUT2);
    return bags;
  }

  TIME time_advance() const {
    if(state.active) {
      return TIME("00:00:00");
    }
    return std::numeric_limits<TIME>::infinity();
  }

  friend std::ostringstream& operator<<(std::ostringstream& os, const typename nRFController<TIME>::state_type& i) {
             os << "Sent to nRF for transmitting: " << i.currentLocation << "Sent to Position Controller for comparison: " << i.ToGoPosition;
             return os;
           }
};
#endif
