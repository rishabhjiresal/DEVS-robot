#ifndef BOOST_SIMULATION_RFID_SENSOR_HPP
#define BOOST_SIMULATION_RFID_SENSOR_HPP

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
#include "../drivers/RFID_DRIVER.hpp"

struct RFID_Sensor_defs
{
        struct out : public out_port<int> { };
};
/******************************************************************************
* REAL-TIME IMPLEMENTATION
*****************************************************************************/

template<typename TIME>
class RFID_Sensor
{
using defs=RFID_Sensor_defs;
	public:
		TIME pollingRate;
    RFID_Sensor() noexcept {
      MBED_ASSERT(false);
    }

		RFID_Sensor(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName reset) noexcept {
      new (this) RFID_Sensor(mosi, miso, sclk, cs, reset, TIME("00:00:00:050"));
    		}

        RFID_Sensor(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName reset, TIME rate) noexcept {
          pollingRate = rate;
          state.rfid = new drivers::RFID_DRIVER(mosi, miso, sclk, cs, reset);
          state.rfid->InitializeRFID();
        }

    		struct state_type {
    			bool newTag;
    			int last;
    			int output;
          drivers::RFID_DRIVER* rfid;
    			}; state_type state;

    		using input_ports=std::tuple<>;
    		using output_ports=std::tuple<typename defs::out>;

    		void internal_transition() {
    			state.last = state.output;
    			state.rfid->ReturnID(state.output);
    			}

    		void external_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
      			MBED_ASSERT(false);
      //throw std::logic_error("External transition called in a model with no input ports");
    }

    		void confluence_transition(TIME e, typename make_message_bags<input_ports>::type mbs) {
      			internal_transition();
      			external_transition(TIME(), std::move(mbs));
    }

    		typename make_message_bags<output_ports>::type output() const {
    			typename make_message_bags<output_ports>::type bags;
    			if(state.last != state.output) {
    				int OUT = state.output;
    				get_messages<typename defs::out>(bags).push_back(OUT);
    				}
          }

    		TIME time_advance() const {
    			  return pollingRate;
   		 }

   		 friend std::ostringstream& operator<<(std::ostringstream& os, const typename RFID_Sensor<TIME>::state_type& i) {
      os << "RFID Output " << i.output;
      return os;
    }
  };

#else
  #include <cadmium/io/iestream.hpp>
  using namespace cadmium;
  using namespace std;

  //Port definition
  struct RFID_Sensor_defs{
    struct out : public out_port<int> {};
  };

  template<typename TIME>
  class RFID_Sensor : public iestream_input<int,TIME, RFID_Sensor_defs>{
    public:
      RFID_Sensor() = default;
      RFID_Sensor(const char* file_path) : iestream_input<int,TIME, RFID_Sensor_defs>(file_path) {}
      RFID_Sensor(const char* file_path, TIME t) : iestream_input<int,TIME, RFID_Sensor_defs>(file_path) {}
  };
#endif
#endif
