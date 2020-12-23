#include <iostream>
#include <chrono>
#include <algorithm>
#include <string>

#include <cadmium/modeling/coupling.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>
#include <cadmium/modeling/dynamic_coupled.hpp>
#include <cadmium/modeling/dynamic_atomic.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/tuple_to_ostream.hpp>
#include <cadmium/logger/common_loggers.hpp>
#include <cadmium/io/iestream.hpp>


#include <NDTime.hpp>

#include <cadmium/real_time/arm_mbed/io/digitalInput.hpp>
#include <cadmium/real_time/arm_mbed/io/analogInput.hpp>
#include <cadmium/real_time/arm_mbed/io/pwmOutput.hpp>
#include <cadmium/real_time/arm_mbed/io/digitalOutput.hpp>

#include "../atomics/RFIDSensor.hpp"
#include "../atomics/RFIDController.hpp"
#include "../atomics/nRFController.hpp"
#include "../atomics/PositionController.hpp"
#include "../atomics/nRFReceiver.hpp"
#include "../atomics/nRFTransmitter.hpp"

#ifdef RT_ARM_MBED
  #include "../mbed.h"
#else
  const char* RFIDSensorIN = "./inputs/SensorInput.txt";
  const char* TransmittedbyNRF = "./outputs/TransmittedbyNRF.txt";
  const char* ReceivedbyNRF = "./inputs/ReceivedbyNRF.txt";
  const char* A0  = "./inputs/A0_RightIR_In.txt";
  const char* A2  = "./inputs/A2_CenterIR_In.txt";
  const char* D4  = "./inputs/D4_LeftIR_In.txt";
  const char* A1  = "./inputs/A1_middleRightIR_In.txt";
  const char* A3  = "./inputs/A3_middleLefttIR_In.txt";
  const char* A5  = "./inputs/A5_LightSensor_In.txt";
  const char* D8  = "./outputs/D8_RightMotor1_Out.txt";
  const char* D11 = "./outputs/D11_RightMotor2_Out.txt";
  const char* D12 = "./outputs/D12_LeftMotor1_Out.txt";
  const char* D13 = "./outputs/D13_LeftMotor2_Out.txt";
  const char* PB_15;
  const char* PB_14;
  const char* PB_10;
  const char* PB_1;
  const char* PB_13;
  const char* D6;
  const char* D7;
  const char* D15;
  const char* PC_12;
  const char* PC_11;
  const char* PC_10;
  const char* PC_9;
  const char* D14;
  const char* D9;

#endif

using namespace std;

using hclock=chrono::high_resolution_clock;
using TIME = NDTime;

int main(int argc, char ** argv) {

  //This will end the main thread and create a new one with more stack.
  #ifdef RT_ARM_MBED
    //Logging is done over cout in RT_ARM_MBED
    struct oss_sink_provider{
      static std::ostream& sink(){
        return cout;
      }
    };
  #else
    // all simulation timing and I/O streams are ommited when running embedded

    auto start = hclock::now(); //to measure simulation execution time

    /*************** Loggers *******************/

    static std::ofstream out_data("ROBOT2_output.txt");
    struct oss_sink_provider{
      static std::ostream& sink(){
        return out_data;
      }
    };
  #endif

using info=cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using debug=cadmium::logger::logger<cadmium::logger::logger_debug, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using state=cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using log_messages=cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using routing=cadmium::logger::logger<cadmium::logger::logger_message_routing, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using global_time=cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using local_time=cadmium::logger::logger<cadmium::logger::logger_local_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
using log_all=cadmium::logger::multilogger<info, debug, state, log_messages, routing, global_time, local_time>;
using logger_top=cadmium::logger::multilogger<log_messages, global_time>;

using AtomicModelPtr=std::shared_ptr<cadmium::dynamic::modeling::model>;
using CoupledModelPtr=std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>>;

#ifdef RT_ARM_MBED
 AtomicModelPtr RFID_Sensor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<RFID_Sensor, TIME>("RFID_Sensor1", PB_15, PB_14, PB_10, PB_1, PB_13);
 AtomicModelPtr nRFTransmitter1 = cadmium::dynamic::translate::make_dynamic_atomic_model<nRFTransmitter, TIME>("nRFTransmitter1", PC_12, PC_11, PC_10, PA_15, PA_13, PC_2);
 AtomicModelPtr nRFReceiver1 = cadmium::dynamic::translate::make_dynamic_atomic_model<nRFReceiver, TIME>("nRFReceiver1", PC_12, PC_11, PC_10, PA_15, PA_13, PC_2);

 #else
 AtomicModelPtr RFID_Sensor1 = cadmium::dynamic::translate::make_dynamic_atomic_model<RFID_Sensor, TIME>("RFID_Sensor1", RFIDSensorIN);
 AtomicModelPtr nRFTransmitter1 = cadmium::dynamic::translate::make_dynamic_atomic_model<nRFTransmitter, TIME>("nRFTransmitter1", TransmittedbyNRF);
 AtomicModelPtr nRFReceiver1 = cadmium::dynamic::translate::make_dynamic_atomic_model<nRFReceiver, TIME>("nRFReceiver1", ReceivedbyNRF);
 #endif
 AtomicModelPtr RFID_Controller1 = cadmium::dynamic::translate::make_dynamic_atomic_model<RFID_Controller, TIME>("RFID_Controller1");
 AtomicModelPtr nRFController1 = cadmium::dynamic::translate::make_dynamic_atomic_model<nRFController, TIME>("nRFController1");
 AtomicModelPtr PositionController1 = cadmium::dynamic::translate::make_dynamic_atomic_model<PositionController, TIME>("PositionController1");

cadmium::dynamic::modeling::Ports iports_TOP = {};
cadmium::dynamic::modeling::Ports oports_TOP = {};
cadmium::dynamic::modeling::Models submodels_TOP = {RFID_Sensor1, RFID_Controller1, nRFTransmitter1, nRFController1, nRFReceiver1, PositionController1 };
cadmium::dynamic::modeling::EICs eics_TOP = {};
cadmium::dynamic::modeling::EOCs eocs_TOP = {};
cadmium::dynamic::modeling::ICs ics_TOP = {
   cadmium::dynamic::translate::make_IC<RFID_Sensor_defs::out, RFID_Controller_defs::RFIDInput>("RFID_Sensor1","RFID_Controller1"),
   cadmium::dynamic::translate::make_IC<RFID_Controller_defs::Location, nRFController_defs::LocationInput>("RFID_Controller1", "nRFController1"),
   cadmium::dynamic::translate::make_IC<RFID_Controller_defs::Location, PositionController_defs::CurrentPositionInputFromRFID>("RFID_Controller1", "PositionController1"),
   cadmium::dynamic::translate::make_IC<nRFReceiver_defs::out, nRFController_defs::nRFReceiverInput>("nRFReceiver1", "nRFController1"),
   cadmium::dynamic::translate::make_IC<nRFController_defs::TransmitterOut, nRFTransmitter_defs::in>("nRFController1", "nRFTransmitter1"),
   cadmium::dynamic::translate::make_IC<nRFController_defs::PositionControllerOut, PositionController_defs::ToGoPositionInputFromNRF>("nRFController1", "PositionController1")
 };
   CoupledModelPtr TOP = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
    "TOP",
    submodels_TOP,
    iports_TOP,
    oports_TOP,
    eics_TOP,
    eocs_TOP,
    ics_TOP
    );

  cadmium::dynamic::engine::runner<NDTime, logger_top> r(TOP, {0});

  r.run_until(NDTime("100:00:00:000"));
#ifndef RT_ARM_MBED
  return 0;
#endif

}
