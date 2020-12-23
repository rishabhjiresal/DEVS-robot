#ifndef BOOST_SIMULATION_POSITION_CONTROLLER_HPP
#define BOOST_SIMULATION_POSITION_CONTROLLER_HPP

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

struct PositionController_defs {
  struct CurrentPositionInputFromRFID : public in_port<char> { };
  struct ToGoPositionInputFromNRF : public in_port<char> { };

  struct RobotMovementOut : public out_port<char> { };
};

template<typename TIME>
class PositionController {
  using defs=PositionController_defs;
public:
  PositionController() noexcept {
    state.currentPosition = '\0';
    state.ToGoPosition = '\0';
    state.RobotFrontorBack = '\0';
    state.active = false;
  }

  struct state_type {
    char currentPosition;
    char lastcurrentPosition;
    char ToGoPosition;
    char RobotFrontorBack;
    bool active;
  };state_type state;

  using input_ports=std::tuple<typename defs::CurrentPositionInputFromRFID, typename defs::ToGoPositionInputFromNRF>;
  using output_ports=std::tuple<typename defs::RobotMovementOut>;

  void internal_transition () {
    state.lastcurrentPosition = state.currentPosition;
    state.active = false;
  }

  void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
    for(const auto &x : get_messages<typename defs::CurrentPositionInputFromRFID>(mbs)) {
      state.currentPosition = x;
    }
    for(const auto &x : get_messages<typename defs::ToGoPositionInputFromNRF>(mbs)) {
			state.ToGoPosition = x;
		}

    if((state.currentPosition == state.ToGoPosition) && state.ToGoPosition != '\0') {
      state.RobotFrontorBack = 'S';
    }
    else if((state.currentPosition < state.ToGoPosition) && state.ToGoPosition != '\0') {
      state.RobotFrontorBack = 'F';
    }
    else if((state.currentPosition > state.ToGoPosition) && state.ToGoPosition != '\0') {
      state.RobotFrontorBack = 'B';
    }
    else {
      state.RobotFrontorBack = 'U';
    }
    state.active = true;
  }

  void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

  typename make_message_bags<output_ports>::type output() const {
    typename make_message_bags<output_ports>::type bags;
		char OUT = state.RobotFrontorBack;
		get_messages<typename defs::RobotMovementOut>(bags).push_back(OUT);

		return bags;

  }

  TIME time_advance() const {
    if(state.active) {
      return TIME("00:00:01");
    }
    return std::numeric_limits<TIME>::infinity();
  }

  friend std::ostringstream& operator<<(std::ostringstream& os, const typename PositionController<TIME>::state_type& i) {
             os << "Sent Data to the Robot Movement: " << i.RobotFrontorBack;
             return os;
           }
};
#endif
