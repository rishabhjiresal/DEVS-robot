#ifndef NRF_DRIVER_HPP
#define NRF_DRIVER_hpp


#include "nRF24L01P.h"


namespace drivers {

class NRF_DRIVER {
	private:
		nRF24L01P nrf;

	public:

	NRF_DRIVER(PinName mosi, PinName miso, PinName sck, PinName csn, PinName ce, PinName irq) : nrf(mosi, miso, sck, csn, ce, irq) {

	}

	void PowerUp() {
		nrf.powerUp();
	}

	void TransferSize(int size) {
		nrf.setTransferSize(size);
	}

	void EnableNRF() {
		nrf.enable();
	}

	bool CheckDatainPipe() {
		bool status;
		status = nrf.readable();
		return status;
	}

	int ReceiveData(char data[], int count) {
		nrf.setReceiveMode();
		int temp;
		if(CheckDatainPipe()){
		temp = nrf.read(NRF24L01P_PIPE_P0, data, count);
	}
		return temp;
	}

	void printDetails(){

 	   // Display the (default) setup of the nRF24L01+ chip
 	   printf( "nRF24L01+ Frequency    : %d MHz\r\n",  nrf.getRfFrequency() );
 	   printf( "nRF24L01+ Output power : %d dBm\r\n",  nrf.getRfOutputPower() );
 	   printf( "nRF24L01+ Data Rate    : %d kbps\r\n", nrf.getAirDataRate() );
 	   printf( "nRF24L01+ TX Address   : 0x%010llX\r\n", nrf.getTxAddress() );
 	   printf( "nRF24L01+ RX Address   : 0x%010llX\r\n", nrf.getRxAddress() );
	}

	void ENautoAck(){
		nrf.enableAutoAcknowledge(NRF24L01P_PIPE_P0);
	}


	int TransmitData(char data[], int count) {
		nrf.setTransmitMode();
		int temp;
		temp = nrf.write(NRF24L01P_PIPE_P0, data, count);
		return temp;
	}

	void disableNRF() {
		nrf.disable();
	}



};
}
#endif
