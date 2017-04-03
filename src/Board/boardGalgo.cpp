#include "Board/boardGalgo.h"
#include "Board/exceptions.h"

namespace controller {

BoardGalgo::BoardGalgo( const std::string &port, int baudRate ) :
    Board( "Board Galgo", TYPE_GALGO ),
    portHandler_( dynamixel::PortHandler::getPortHandler( port.c_str() ) ) {
  if( !portHandler_->openPort() )
      throw FailedOpeningPortException("Failed to open the port \"" + port + '\"');
  if( !portHandler_->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException("Failed to change the baudrate to " + std::to_string( baudRate ) ) ;
}
BoardGalgo::~BoardGalgo() {
  portHandler_->closePort();
}

// TO-DO Customowe wyjątki;
// umożliwienie wypisania TxRxResult po złapaniu wyjątku
void BoardGalgo::handle( dynamixel::PacketHandler *packetHandler,
        int communicationResult ) {
    if( communicationResult != COMM_SUCCESS ) {
        packetHandler->printTxRxResult( communicationResult );
        throw std::runtime_error( "Dynamixel communication unsuccessful" );
    }
}
void BoardGalgo::handle( dynamixel::PacketHandler *packetHandler,
        uint8_t error ) {
    if( error != 0 ) {
        printf( "Dynamixel hardware error: " );
        packetHandler->printRxPacketError( error );
    }
}
void BoardGalgo::handle( dynamixel::PacketHandler *packetHandler,
        int communicationResult, uint8_t error ) {
    #ifndef NDEBUG
    handle( packetHandler, communicationResult );
    #endif // #ifndef NDEBUG
    handle( packetHandler, error );
}

BoardGalgo::tId BoardGalgo::convert( int legNo, int jointNo ) {
    return static_cast< tId >( legNo * 10 + jointNo );
}

void BoardGalgo::toggleTorque( int legNo, int joinNo, bool onOrOff ) {
    toggleTorque( convert( legNo, joinNo ), onOrOff );
}
void BoardGalgo::toggleTorque( int legNo,
        const std::vector< bool >& onOrOff ) {}
void BoardGalgo::toggleTorque( const std::vector< bool >& onOrOff ) {}

void BoardGalgo::setLED(int legNo, int jointNo, bool powered){
    tId id = convert(legNo, jointNo);
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler_, id, LED, powered, &dxl_error);
    handle(packetHandler, dxl_comm_result, dxl_error);
}

void BoardGalgo::setLED(int legNo, const std::vector<bool>& powered){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler, LED, 1);

    uint8_t v;

    for(int i = 0; i < 3; i++){
        v = powered[i];
        groupSyncWrite.addParam(convert(legNo, i), &v);
    }

    dxl_comm_result = groupSyncWrite.txPacket();
    handle(packetHandler, dxl_comm_result);
}

void BoardGalgo::setLED(const std::vector<bool> &powered){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler, LED, 1);

    uint8_t v;
    int ix = 0;

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            v = powered[ix];
            groupSyncWrite.addParam(convert(i, j), &v);
            ix++;
        }
    }

    dxl_comm_result = groupSyncWrite.txPacket();
    handle(packetHandler, dxl_comm_result);
}

void BoardGalgo::setOperatingMode(int legNo, int jointNo, uint8_t operatingMode){
    tId dynamixel = convert(legNo, jointNo);
    uint8_t error;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    toggleTorque(dynamixel, false);

    int communicationResult = packetHandler->write1ByteTxRx(portHandler_, dynamixel, OPERATING_MODE, operatingMode, &error);
    handle(packetHandler, communicationResult, error);
}

void BoardGalgo::toggleTorque( tId dynamixel, bool onOrOff ) {
    uint8_t error;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler( PROTOCOL_VERSION );
    int communicationResult = packetHandler->write1ByteTxRx( portHandler_,
            dynamixel, TORQUE_ENABLE, onOrOff, &error );
    handle( packetHandler, communicationResult, error );
}

uint16_t BoardGalgo::convertAngle( double angle ) {
    return static_cast< uint16_t >( angle * 11.375 );
}
unsigned int BoardGalgo::setPosition(int legNo, int jointNo, double angle){
    tId dynamixel = convert( legNo, jointNo );
    toggleTorque( dynamixel, true );
    uint8_t error;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler( PROTOCOL_VERSION );
    int communicationResult = packetHandler->write4ByteTxRx( portHandler_,
            dynamixel, GOAL_POSITION, convertAngle( angle ), &error );
    handle( packetHandler, communicationResult, error );

    return 0;
}
unsigned int BoardGalgo::setPosition(int legNo, const std::vector<double>& angle){}
unsigned int BoardGalgo::setPosition(const std::vector<double>& angle){}

uint32_t BoardGalgo::convertSpeed(double value){
    return static_cast<uint32_t>(value * MAX_SPEED);
}

unsigned int BoardGalgo::setSpeed(int legNo, int jointNo, double speed){
    tId dynamixel = convert(legNo, jointNo);
    uint8_t error;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    toggleTorque(dynamixel, true);
    int communicationResult = packetHandler->write4ByteTxRx(portHandler_, dynamixel, GOAL_VELOCITY, convertSpeed(speed), &error);
    handle(packetHandler, communicationResult, error);
}

unsigned int BoardGalgo::setSpeed(int legNo, const std::vector<double>& speed){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler, GOAL_VELOCITY, 4);

    toggleTorque(convert(legNo, 0), true);
    toggleTorque(convert(legNo, 1), true);
    //toggleTorque(convert(legNo, 2), true);

    uint32_t s;
    uint8_t v[4];

    for(int i = 0; i < 3; i++){
        s = convertSpeed(speed[i]);
        v[0] = DXL_LOBYTE(DXL_LOWORD(s));
        v[1] = DXL_HIBYTE(DXL_LOWORD(s));
        v[2] = DXL_LOBYTE(DXL_HIWORD(s));
        v[3] = DXL_HIBYTE(DXL_HIWORD(s));
        groupSyncWrite.addParam(convert(legNo, i), v);
    }

    dxl_comm_result = groupSyncWrite.txPacket();
    handle(packetHandler, dxl_comm_result);
}

unsigned int BoardGalgo::setSpeed(const std::vector<double>& speed){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler, GOAL_VELOCITY, 4);

    toggleTorque(convert(1, 0), true);
    toggleTorque(convert(1, 1), true);
    //toggleTorque(convert(legNo, 2), true);

    uint32_t s;
    uint8_t v[4];
    int ix = 0;

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            s = convertSpeed(speed[ix]);
            v[0] = DXL_LOBYTE(DXL_LOWORD(s));
            v[1] = DXL_HIBYTE(DXL_LOWORD(s));
            v[2] = DXL_LOBYTE(DXL_HIWORD(s));
            v[3] = DXL_HIBYTE(DXL_HIWORD(s));
            groupSyncWrite.addParam(convert(i, j), v);
            ix++;
        }
    }

    dxl_comm_result = groupSyncWrite.txPacket();
    handle(packetHandler, dxl_comm_result);
}

unsigned int BoardGalgo::setComplianceMargin(int legNo, int jointNo, double margin){}
unsigned int BoardGalgo::setComplianceMargin(int legNo, const std::vector<double> margin){}
unsigned int BoardGalgo::setComplianceMargin(const std::vector<double> margin){}

unsigned int BoardGalgo::setComplianceSlope(int legNo, int jointNo, double slope){}
unsigned int BoardGalgo::setComplianceSlope(int legNo, const std::vector<double>& slope){}
unsigned int BoardGalgo::setComplianceSlope(const std::vector<double>& slope){}

unsigned int BoardGalgo::setTorqueLimit(int legNo, int jointNo, double torqueLimit){}
unsigned int BoardGalgo::setTorqueLimit(int legNo, const std::vector<double>& torqueLimit){}
unsigned int BoardGalgo::setTorqueLimit(const std::vector<double>& torqueLimit){}

double BoardGalgo::convert( uint32_t position ) {
    return position / 11.375;
}
unsigned int BoardGalgo::readPosition(int legNo, int jointNo, double& angle){
    tId dynamixel = convert( legNo, jointNo );
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler( PROTOCOL_VERSION );
    uint32_t presentPosition;
    uint8_t error;
    int communicationResult = packetHandler->read4ByteTxRx(portHandler_, dynamixel,
            PRESENT_POSITION, &presentPosition, &error);

    handle( packetHandler, communicationResult, error );

    angle = convert( presentPosition );

    return 0;
}
unsigned int BoardGalgo::readPosition(int legNo, std::vector<double>& angle){}
unsigned int BoardGalgo::readPosition(std::vector<double>& angle){}

unsigned int BoardGalgo::readForce(int legNo, double& contactForce){}
unsigned int BoardGalgo::readForce(const std::vector<double>& contactForce){}

unsigned int BoardGalgo::readTorqueForce(int legNo, walkers::TorqueForce& valueTF){}
unsigned int BoardGalgo::readTorqueForce(const std::vector<double>& valueTF){}

bool BoardGalgo::readContact(int legNo){}
void BoardGalgo::readContacts(std::vector<bool>& contact){}

double BoardGalgo::convertCurrent(uint16_t value){
    return (int16_t)value/((double)MAX_CURRENT);
}

unsigned int BoardGalgo::readCurrent(int legNo, int jointNo, double& servoCurrent){
    tId id = convert(legNo, jointNo);
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    uint16_t v = 0;

    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler_, id, PRESENT_CURRENT, &v, &dxl_error);
    servoCurrent = convertCurrent(v);
    handle(packetHandler, dxl_comm_result, dxl_error);

    return 0;
}

unsigned int BoardGalgo::readCurrent(int legNo, std::vector<double>& servoCurrent){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler, PRESENT_CURRENT, 2);

    for(int i = 0; i < 3; i++){
        groupSyncRead.addParam(convert(legNo, i));
    }

    dxl_comm_result = groupSyncRead.txRxPacket();
    handle(packetHandler, dxl_comm_result);

    for(int i = 0; i < 3; i++){
        servoCurrent[i] = convertCurrent(groupSyncRead.getData(convert(legNo, i), PRESENT_CURRENT, 2));
    }
}

unsigned int BoardGalgo::readCurrent(std::vector<double>& servoCurrent){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler, PRESENT_CURRENT, 2);

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            groupSyncRead.addParam(convert(i, j));
        }
    }

    dxl_comm_result = groupSyncRead.txRxPacket();
    handle(packetHandler, dxl_comm_result);

    int ix = 0;

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            servoCurrent[ix] = convertCurrent(groupSyncRead.getData(convert(i, j), PRESENT_CURRENT, 2));
            ix++;
        }
    }
}

unsigned int BoardGalgo::readTorque(int legNo, int jointNo, double& servoTorque){}
unsigned int BoardGalgo::readTorque(int legNo,std::vector<double>& servoTorque){}
unsigned int BoardGalgo::readTorque(std::vector<double>& servoTorque){}

void BoardGalgo::setOffset(int legNo, int jointNo, double offset){}
void BoardGalgo::setOffset(int legNo, const std::vector<double> offset){}
void BoardGalgo::setOffset(const std::vector<double> offset){}

void BoardGalgo::setDefault(void){}

} // namespace controller
