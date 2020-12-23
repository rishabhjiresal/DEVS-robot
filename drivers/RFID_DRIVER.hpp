#ifndef RFID_DRIVER_HPP
#define RFID_DRIVER_hpp

#include "MFRC522.h"

namespace drivers {

class RFID_DRIVER {
	private:
		MFRC522 mfrc;

	public:

	RFID_DRIVER(PinName mosi, PinName miso, PinName sclk, PinName cs, PinName reset) : mfrc(mosi, miso, sclk, cs, reset) {

	}

	void InitializeRFID() {
		mfrc.PCD_Init();
		ThisThread::sleep_for(5);
	}

	bool IsNewCardPresent() {
		bool temp;
		temp = mfrc.PICC_IsNewCardPresent();
		return temp;
	}

	bool ReadCardSerial() {
		bool temp;
		temp = mfrc.PICC_ReadCardSerial();
		return temp;
	}

	void ReturnID(int uid){
		uint8_t id[1];
    		if(!IsNewCardPresent()) {
  			return;
  			}

  			if(!ReadCardSerial())
  			{
  			return;
  			}
		for(uint8_t i =0; i<mfrc.uid.size; i++){
		id[i] = mfrc.uid.uidByte[i];
		}
		uid = id[0];
	}
};
}
#endif
