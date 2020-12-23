#ifndef BOOST_SIMULATION_RFID_CONTROLLER_HPP
#define BOOST_SIMULATION_RFID_CONTROLLER_HPP

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

struct RFID_Controller_defs
{
		struct RFIDInput : public in_port<int> { };
        struct Location : public out_port<char> { };
};

template<typename TIME>
class RFID_Controller {
	using defs=RFID_Controller_defs;
public:
	RFID_Controller() noexcept {
		state.active = false;
		state.dataSent = '\0';
		state.receivedData = 0;
	}
	struct state_type {
		char dataSent;
		int receivedData;
		bool active;
		/* data */
	}; state_type state;
	using input_ports=std::tuple<typename defs::RFIDInput>;
	using output_ports=std::tuple<typename defs::Location>;

	void internal_transition() {
		state.active = false;
	}

	void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs){
		for(const auto &x : get_messages<typename defs::RFIDInput>(mbs)) {
			state.receivedData = x;
		}
		if(state.receivedData == 123) {
			state.dataSent = 'A';
		}
		if(state.receivedData == 234) {
			state.dataSent = 'B';
		}
		if(state.receivedData == 345) {
			state.dataSent = 'C';
		}

		state.active = true;
	}

	void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
		internal_transition();
		external_transition(TIME(), std::move(mbs));
	}

	typename make_message_bags<output_ports>::type output() const {
		typename make_message_bags<output_ports>::type bags;
		char OUT = state.dataSent;
		get_messages<typename defs::Location>(bags).push_back(OUT);

		return bags;
	}

	TIME time_advance() const {
		if(state.active) {
			return TIME("00:00:00");
		}
		return std::numeric_limits<TIME>::infinity();
	}

	friend std::ostringstream& operator<<(std::ostringstream& os, const typename RFID_Controller<TIME>::state_type& i) {
						 os << "Sent Data: " << i.dataSent;
						 return os;
					 }
};
#endif
