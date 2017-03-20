
#include "Board/boardGalgo.h"
#include "Board/exceptions.h"

namespace controller {

// TO-DO Zapytać o Type
BoardGalgo::BoardGalgo( const std::string &port, int baudRate ) :
    Board( "Board Galgo", TYPE_USB2DYNAMIXEL ), protocolVersion_( 2.0 ),
    portHandler_( dynamixel::PortHandler::getPortHandler( port.c_str() ) ) {
  if( !portHandler_->openPort() )
      throw FailedOpeningPortException("Failed to open the port \"" + port + '\"');
  if( !portHandler_->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException("Failed to change the baudrate to " + baudRate);
}
BoardGalgo::~BoardGalgo() {
  portHandler_->closePort();
}

void BoardGalgo::toggleTorque( uint8_t dynamixelId, bool onOrOff ) {
    uint8_t error;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler( protocolVersion_ );
    int communicationResult = packetHandler->write1ByteTxRx( portHandler_,
            dynamixelId, 64, onOrOff, &error );

    if( communicationResult != COMM_SUCCESS ) {
        packetHandler->printTxRxResult( communicationResult );
    }
    else if( error != 0 ) {
        packetHandler->printRxPacketError( error );
    }
}

void BoardGalgo::setLED(int legNo, int jointNo, bool powered){
    uint8_t id = legNo * 10 + jointNo;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler( protocolVersion_ );

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler_, id, ADDR_LED, powered, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0) {
      packetHandler->printRxPacketError(dxl_error);
    }
}

unsigned int BoardGalgo::setPosition(int legNo, int jointNo, double angle){}
unsigned int BoardGalgo::setPosition(int legNo, const std::vector<double>& angle){}
unsigned int BoardGalgo::setPosition(const std::vector<double>& angle){}

unsigned int BoardGalgo::setSpeed(int legNo, int jointNo, double speed){}
unsigned int BoardGalgo::setSpeed(int legNo, const std::vector<double>& speed){}
unsigned int BoardGalgo::setSpeed(const std::vector<double>& speed){}

unsigned int BoardGalgo::setComplianceMargin(int legNo, int jointNo, double margin){}
unsigned int BoardGalgo::setComplianceMargin(int legNo, const std::vector<double> margin){}
unsigned int BoardGalgo::setComplianceMargin(const std::vector<double> margin){}

unsigned int BoardGalgo::setComplianceSlope(int legNo, int jointNo, double slope){}
unsigned int BoardGalgo::setComplianceSlope(int legNo, const std::vector<double>& slope){}
unsigned int BoardGalgo::setComplianceSlope(const std::vector<double>& slope){}

unsigned int BoardGalgo::setTorqueLimit(int legNo, int jointNo, double torqueLimit){}
unsigned int BoardGalgo::setTorqueLimit(int legNo, const std::vector<double>& torqueLimit){}
unsigned int BoardGalgo::setTorqueLimit(const std::vector<double>& torqueLimit){}

unsigned int BoardGalgo::readPosition(int legNo, int jointNo, double& angle){}
unsigned int BoardGalgo::readPosition(int legNo, std::vector<double>& angle){}
unsigned int BoardGalgo::readPosition(std::vector<double>& angle){}

unsigned int BoardGalgo::readForce(int legNo, double& contactForce){}
unsigned int BoardGalgo::readForce(const std::vector<double>& contactForce){}

unsigned int BoardGalgo::readTorqueForce(int legNo, walkers::TorqueForce& valueTF){}
unsigned int BoardGalgo::readTorqueForce(const std::vector<double>& valueTF){}

bool BoardGalgo::readContact(int legNo){}
void BoardGalgo::readContacts(std::vector<bool>& contact){}

unsigned int BoardGalgo::readCurrent(int legNo, int jointNo, double& servoCurrent){}
unsigned int BoardGalgo::readCurrent(int legNo, std::vector<double>& servoCurrent){}
unsigned int BoardGalgo::readCurrent( std::vector<double>& servoCurrent){}

unsigned int BoardGalgo::readTorque(int legNo, int jointNo, double& servoTorque){}
unsigned int BoardGalgo::readTorque(int legNo,std::vector<double>& servoTorque){}
unsigned int BoardGalgo::readTorque(std::vector<double>& servoTorque){}

void BoardGalgo::setOffset(int legNo, int jointNo, double offset){}
void BoardGalgo::setOffset(int legNo, const std::vector<double> offset){}
void BoardGalgo::setOffset(const std::vector<double> offset){}

void BoardGalgo::setDefault(void){}

} // namespace controller
