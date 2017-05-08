#include <utility>
#include "Board/boardGalgo.h"
#include "Board/exceptions.h"

namespace controller {

const uint8_t BoardGalgo::OPERATINGMODE_VELOCITY = 1;
const uint8_t BoardGalgo::OPERATINGMODE_POSITION = 3;

BoardGalgo::BoardGalgo( const std::string &rightLegsDevPath,
                        const std::string &leftLegsDevPath,
                        int baudRate ) :
        Board( "Board Galgo", TYPE_GALGO ),
        rightLegs_( dynamixel::PortHandler::getPortHandler(
                           rightLegsDevPath.c_str() ) ),
        leftLegs_( dynamixel::PortHandler::getPortHandler(
                           leftLegsDevPath.c_str() ) ),
        packetHandler_( dynamixel::PacketHandler::getPacketHandler(
                                PROTOCOL_VERSION ) ) {
    preparePortHandler( rightLegs_, baudRate );
    preparePortHandler( leftLegs_, baudRate );
    preparePortHandlersByLegNumberMap();
}
void BoardGalgo::preparePortHandler( const tPortHandler& portHandler,
                                     int baudRate ) {
  if( !portHandler->openPort() )
      throw FailedOpeningPortException(
            std::string( "Failed to open the port \"" ) +
            portHandler->getPortName() + '\"');
  if( !portHandler->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException("Failed to change the baudrate to " + std::to_string( baudRate ) ) ;
}
void BoardGalgo::preparePortHandlersByLegNumberMap() {
    portHandlersByLegNumber_.insert( std::make_pair( 1, rightLegs_ ) );
    portHandlersByLegNumber_.insert( std::make_pair( 2, rightLegs_ ) );
    portHandlersByLegNumber_.insert( std::make_pair( 3, leftLegs_ ) );
    portHandlersByLegNumber_.insert( std::make_pair( 4, leftLegs_ ) );
}
BoardGalgo::~BoardGalgo() {
  rightLegs_->closePort();
  leftLegs_->closePort();
}

void BoardGalgo::reboot( int legNo, int jointNo ) {
    tId dynamixel = convert( legNo, jointNo );
    uint8_t error;
    int communicationResult = packetHandler_->reboot( portHandlersByLegNumber_.at( legNo ).get(), dynamixel,
            &error );
    handle( communicationResult, error );
}
// WIP
void BoardGalgo::reboot( int legNo ) {}
void BoardGalgo::reboot() {}

// TO-DO Customowe wyjątki;
// umożliwienie wypisania TxRxResult po złapaniu wyjątku
void BoardGalgo::handle( int communicationResult ) {
    if( communicationResult != COMM_SUCCESS ) {
        packetHandler_->printTxRxResult( communicationResult );
        throw std::runtime_error( "Dynamixel communication unsuccessful" );
    }
}
void BoardGalgo::handle( uint8_t error ) {
    if( error != 0 ) {
        printf( "Dynamixel error: " );
        packetHandler_->printRxPacketError( error );
    }
}
void BoardGalgo::handle( int communicationResult, uint8_t error ) {
    #ifndef NDEBUG
    handle( communicationResult );
    #endif // #ifndef NDEBUG
    handle( error );
}

BoardGalgo::tId BoardGalgo::convert( int legNo, int jointNo ) {
    return static_cast< tId >( legNo * 10 + jointNo );
}

void BoardGalgo::toggleTorque( int legNo, int joinNo, bool onOrOff ) {
    uint8_t error;
    int communicationResult = packetHandler_->write1ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(),
            convert( legNo, joinNo ), TORQUE_ENABLE, onOrOff, &error );
    handle( communicationResult, error );
}
void BoardGalgo::toggleTorque( int legNo,
        const std::vector< bool >& onOrOff ) {
    dynamixel::GroupSyncWrite groupSyncWrite( portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(),
            TORQUE_ENABLE, 1 );
    uint8_t convertedOnOrOff;
    for( int joinNo = 0; joinNo < 3; ++joinNo ) {
        convertedOnOrOff = onOrOff[ joinNo ];
        groupSyncWrite.addParam( convert( legNo, joinNo ), &convertedOnOrOff );
    }
    handle( groupSyncWrite.txPacket() );
    groupSyncWrite.clearParam();
}
void BoardGalgo::toggleTorque( const std::vector< bool >& onOrOff ) {
    // dynamixel::GroupSyncWrite groupSyncWrite( portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(),
    //         TORQUE_ENABLE, 1 );
    // uint8_t convertedOnOrOff;
    // for( int legNo = 0, i = 0; legNo < 4; ++legNo ) {
    //     for( int joinNo = 0; joinNo < 3; ++joinNo, ++i ) {
    //         convertedOnOrOff = onOrOff[ i ];
    //         groupSyncWrite.addParam( convert( legNo, joinNo ),
    //                 &convertedOnOrOff );
    //     }
    // }
    // handle( groupSyncWrite.txPacket() );
    // groupSyncWrite.clearParam();
}

void BoardGalgo::setLED(int legNo, int jointNo, bool powered){
    tId id = convert(legNo, jointNo);
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), id, LED, powered, &dxl_error);
    handle(dxl_comm_result, dxl_error);
}

void BoardGalgo::setLED(int legNo, const std::vector<bool>& powered){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::GroupSyncWrite groupSyncWrite(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), LED, 1);

    uint8_t v;

    for(int i = 0; i < 3; i++){
        v = powered[i];
        groupSyncWrite.addParam(convert(legNo, i), &v);
    }

    dxl_comm_result = groupSyncWrite.txPacket();
    handle(dxl_comm_result);
}

void BoardGalgo::setLED(const std::vector<bool> &powered){
    // int dxl_comm_result = COMM_TX_FAIL;
    // dynamixel::GroupSyncWrite groupSyncWrite(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), LED, 1);

    // uint8_t v;
    // int ix = 0;

    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 3; j++){
    //         v = powered[ix];
    //         groupSyncWrite.addParam(convert(i, j), &v);
    //         ix++;
    //     }
    // }

    // dxl_comm_result = groupSyncWrite.txPacket();
    // handle(dxl_comm_result);
}

void BoardGalgo::setOperatingMode(int legNo, int jointNo, uint8_t operatingMode){
    uint8_t error;

    toggleTorque(legNo, jointNo, false);

    int communicationResult = packetHandler_->write1ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), convert(legNo, jointNo), OPERATING_MODE, operatingMode, &error);
    handle(communicationResult, error);
}

void BoardGalgo::setOperatingMode(int legNo, const std::vector<uint8_t>& operatingMode){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::GroupSyncWrite groupSyncWrite(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), OPERATING_MODE, 1);

    std::vector<bool> torque(3, false);
    toggleTorque(legNo, torque);

    uint8_t v;

    for(int i = 0; i < 3; i++){
        v = operatingMode[i];
        groupSyncWrite.addParam(convert(legNo, i), &v);
    }

    dxl_comm_result = groupSyncWrite.txPacket();
    handle(dxl_comm_result);
}

void BoardGalgo::setOperatingMode(const std::vector<uint8_t>& operatingMode){
    // int dxl_comm_result = COMM_TX_FAIL;
    // dynamixel::GroupSyncWrite groupSyncWrite(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), OPERATING_MODE, 1);

    // std::vector<bool> torque(12, false);
    // toggleTorque(torque);

    // uint8_t v;
    // int ix = 0;

    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 3; j++){
    //         v = operatingMode[ix];
    //         groupSyncWrite.addParam(convert(i, j), &v);
    //         ix++;
    //     }
    // }

    // dxl_comm_result = groupSyncWrite.txPacket();
    // handle(dxl_comm_result);
}

uint16_t BoardGalgo::convertAngle( double angle ) {
    return static_cast< uint16_t >( angle * 11.375 );
}
unsigned int BoardGalgo::setPosition(int legNo, int jointNo, double angle){
    toggleTorque( legNo, jointNo, true );
    uint8_t error;
    int communicationResult = packetHandler_->write4ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(),
            convert( legNo, jointNo ), GOAL_POSITION, convertAngle( angle ), &error );
    handle( communicationResult, error );

    return 0;
}
unsigned int BoardGalgo::setPosition(int legNo, const std::vector<double>& angle){
    dynamixel::GroupSyncWrite groupSyncWrite( portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(),
            GOAL_POSITION, 4 );
    toggleTorque( legNo, std::vector< bool >( 3, true ) );
    uint8_t angleAsBytes[ 4 ];
    for( int joinNo = 0; joinNo < 3; ++joinNo ) {
        uint16_t convertedAngle = convertAngle( angle[ joinNo ] );
        angleAsBytes[ 0 ] = DXL_LOBYTE( DXL_LOWORD( convertedAngle ) );
        angleAsBytes[ 1 ] = DXL_HIBYTE( DXL_LOWORD( convertedAngle ) );
        angleAsBytes[ 2 ] = DXL_LOBYTE( DXL_HIWORD( convertedAngle ) );
        angleAsBytes[ 3 ] = DXL_HIBYTE( DXL_HIWORD( convertedAngle ) );
        groupSyncWrite.addParam( convert( legNo, joinNo ),
                angleAsBytes );
    }
    handle( groupSyncWrite.txPacket() );
    groupSyncWrite.clearParam();
}
unsigned int BoardGalgo::setPosition(const std::vector<double>& angle){
    // dynamixel::GroupSyncWrite groupSyncWrite( portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(),
    //         GOAL_POSITION, 4 );
    // toggleTorque( std::vector< bool >( 4 * 3, true ) );
    // uint8_t angleAsBytes[ 4 ];
    // for( int legNo = 0, i = 0; legNo < 4; ++legNo ) {
    //     for( int joinNo = 0; joinNo < 3; ++joinNo, ++i ) {
    //         uint16_t convertedAngle = convertAngle( angle[ i ] );
    //         angleAsBytes[ 0 ] = DXL_LOBYTE( DXL_LOWORD( convertedAngle ) );
    //         angleAsBytes[ 1 ] = DXL_HIBYTE( DXL_LOWORD( convertedAngle ) );
    //         angleAsBytes[ 2 ] = DXL_LOBYTE( DXL_HIWORD( convertedAngle ) );
    //         angleAsBytes[ 3 ] = DXL_HIBYTE( DXL_HIWORD( convertedAngle ) );
    //         groupSyncWrite.addParam( convert( legNo, joinNo ),
    //                 angleAsBytes );
    //     }
    // }
    // handle( groupSyncWrite.txPacket() );
    // groupSyncWrite.clearParam();
}

uint32_t BoardGalgo::convertSpeed(double value){
    return static_cast<uint32_t>(value * MAX_SPEED);
}

unsigned int BoardGalgo::setSpeed(int legNo, int jointNo, double speed){
    uint8_t error;

    toggleTorque(legNo, jointNo, true);
    int communicationResult = packetHandler_->write4ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), convert(legNo, jointNo), GOAL_VELOCITY, convertSpeed(speed), &error);
    handle(communicationResult, error);
}

unsigned int BoardGalgo::setSpeed(int legNo, const std::vector<double>& speed){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::GroupSyncWrite groupSyncWrite(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), GOAL_VELOCITY, 4);

    std::vector<bool> torque(3, true);
    toggleTorque(legNo, torque);

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
    handle(dxl_comm_result);
}

unsigned int BoardGalgo::setSpeed(const std::vector<double>& speed){
    // int dxl_comm_result = COMM_TX_FAIL;
    // dynamixel::GroupSyncWrite groupSyncWrite(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), GOAL_VELOCITY, 4);

    // std::vector<bool> torque(12, true);
    // toggleTorque(torque);

    // uint32_t s;
    // uint8_t v[4];
    // int ix = 0;

    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 3; j++){
    //         s = convertSpeed(speed[ix]);
    //         v[0] = DXL_LOBYTE(DXL_LOWORD(s));
    //         v[1] = DXL_HIBYTE(DXL_LOWORD(s));
    //         v[2] = DXL_LOBYTE(DXL_HIWORD(s));
    //         v[3] = DXL_HIBYTE(DXL_HIWORD(s));
    //         groupSyncWrite.addParam(convert(i, j), v);
    //         ix++;
    //     }
    // }

    // dxl_comm_result = groupSyncWrite.txPacket();
    // handle(dxl_comm_result);
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
    uint32_t presentPosition;
    uint8_t error;
    int communicationResult = packetHandler_->read4ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), dynamixel,
            PRESENT_POSITION, &presentPosition, &error);

    handle( communicationResult, error );

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
    uint16_t v = 0;

    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), id, PRESENT_CURRENT, &v, &dxl_error);
    servoCurrent = convertCurrent(v);
    handle(dxl_comm_result, dxl_error);

    return 0;
}

unsigned int BoardGalgo::readCurrent(int legNo, std::vector<double>& servoCurrent){
    int dxl_comm_result = COMM_TX_FAIL;
    dynamixel::GroupSyncRead groupSyncRead(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), PRESENT_CURRENT, 2);

    for(int i = 0; i < 3; i++){
        groupSyncRead.addParam(convert(legNo, i));
    }

    dxl_comm_result = groupSyncRead.txRxPacket();
    handle(dxl_comm_result);

    for(int i = 0; i < 3; i++){
        servoCurrent[i] = convertCurrent((uint16_t)groupSyncRead.getData(convert(legNo, i), PRESENT_CURRENT, 2));
    }
}

unsigned int BoardGalgo::readCurrent(std::vector<double>& servoCurrent){
    // int dxl_comm_result = COMM_TX_FAIL;
    // dynamixel::GroupSyncRead groupSyncRead(portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), PRESENT_CURRENT, 2);

    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 3; j++){
    //         groupSyncRead.addParam(convert(i, j));
    //     }
    // }

    // dxl_comm_result = groupSyncRead.txRxPacket();
    // handle(dxl_comm_result);

    // int ix = 0;

    // for(int i = 0; i < 4; i++){
    //     for(int j = 0; j < 3; j++){
    //         servoCurrent[ix] = convertCurrent((uint16_t)groupSyncRead.getData(convert(i, j), PRESENT_CURRENT, 2));
    //         ix++;
    //     }
    // }
}

unsigned int BoardGalgo::readTorque(int legNo, int jointNo, double& servoTorque){}
unsigned int BoardGalgo::readTorque(int legNo,std::vector<double>& servoTorque){}
unsigned int BoardGalgo::readTorque(std::vector<double>& servoTorque){}

int BoardGalgo::convertToIndex(int legNo, int jointNo){
    return legNo*3 + jointNo;
}

double BoardGalgo::convertRadToDeg(double angle){
    return angle * (180/M_PI);
}

void BoardGalgo::setOffset(int legNo, int jointNo, double offset){
    angleOffset[convertToIndex(legNo, jointNo)] += (int)convertRadToDeg(offset);
}

void BoardGalgo::setOffset(int legNo, const std::vector<double> offset){
    for(int i = 0; i < 3; i++){
         angleOffset[convertToIndex(legNo, i)] += (int)convertRadToDeg(offset[i]);
    }
}

void BoardGalgo::setOffset(const std::vector<double> offset){
    for(int i = 0; i < 12; i++){
         angleOffset[i] += (int)convertRadToDeg(offset[i]);
    }
}

void BoardGalgo::setDefault(void){}

} // namespace controller
