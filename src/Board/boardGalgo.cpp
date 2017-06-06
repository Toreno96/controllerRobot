#include <algorithm>
#include <cmath>
#include <utility>
#include "Board/boardGalgo.h"
#include "Board/exceptions.h"
#include "Helpers/algorithm.h"
#include "Wrappers/dynamixel3/communicationResult.h"
#include "Wrappers/dynamixel3/syncReader.h"
#include "Wrappers/dynamixel3/syncWriter.h"
#include "../3rdParty/tinyXML/tinyxml2.h"

namespace controller {

BoardGalgo::Ptr boardGalgo;

BoardGalgo::BoardGalgo( const std::string &rightLegsDevPath,
                        const std::string &leftLegsDevPath,
                        int baudRate, uint8_t torqueEnable ) :
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
    setTorque( std::vector< uint8_t >( 4 * JOINTS_COUNT_IN_SINGLE_LEG, torqueEnable ) );

    zeroAngle[0] = 90; zeroAngle[1]  = 0; zeroAngle[2]  = 0;
    zeroAngle[3] = 90; zeroAngle[4]  = 0; zeroAngle[5]  = 0;
    zeroAngle[6] = -90; zeroAngle[7]  = 0; zeroAngle[8]  = 0;
    zeroAngle[9] = -90; zeroAngle[10] = 0; zeroAngle[11] = 0;

    signOfAngle[0] = -1; signOfAngle[1]  = 1; signOfAngle[2]  = 1;
    signOfAngle[3] = -1; signOfAngle[4]  = 1; signOfAngle[5]  = 1;
    signOfAngle[6] = 1; signOfAngle[7]  = 1; signOfAngle[8]  = 1;
    signOfAngle[9] = 1; signOfAngle[10] = 1; signOfAngle[11] = 1;
}

BoardGalgo::BoardGalgo(std::string configFilename) :
        Board( "Board Galgo", TYPE_GALGO ),
        config(configFilename),
        rightLegs_( dynamixel::PortHandler::getPortHandler(
                           config.rightLegsDevPath.c_str() ) ),
        leftLegs_( dynamixel::PortHandler::getPortHandler(
                           config.leftLegsDevPath.c_str() ) ),
        packetHandler_( dynamixel::PacketHandler::getPacketHandler(
                                PROTOCOL_VERSION ) ) {
    preparePortHandler( rightLegs_, config.baudRate );
    preparePortHandler( leftLegs_, config.baudRate );
    preparePortHandlersByLegNumberMap();
    setTorque( std::vector< uint8_t >( 4 * JOINTS_COUNT_IN_SINGLE_LEG, 1 ) );

    zeroAngle[0] = 90; zeroAngle[1]  = 0; zeroAngle[2]  = 0;
    zeroAngle[3] = 90; zeroAngle[4]  = 0; zeroAngle[5]  = 0;
    zeroAngle[6] = -90; zeroAngle[7]  = 0; zeroAngle[8]  = 0;
    zeroAngle[9] = -90; zeroAngle[10] = 0; zeroAngle[11] = 0;

    signOfAngle[0] = -1; signOfAngle[1]  = 1; signOfAngle[2]  = 1;
    signOfAngle[3] = -1; signOfAngle[4]  = 1; signOfAngle[5]  = 1;
    signOfAngle[6] = 1; signOfAngle[7]  = 1; signOfAngle[8]  = 1;
    signOfAngle[9] = 1; signOfAngle[10] = 1; signOfAngle[11] = 1;
}

//------------------------------------------------------------------------------
void BoardGalgo::Config::load(std::string configFilename){
    tinyxml2::XMLDocument configSrv;
    configSrv.LoadFile(std::string("../../resources/" + configFilename).c_str());
    if (configSrv.ErrorID())
        std::cout << "unable to load board Galgo config file.\n";
    rightLegsDevPath = configSrv.FirstChildElement("boardGalgo")->FirstChildElement("parameters")->Attribute("rightLegsDevPath");
    leftLegsDevPath = configSrv.FirstChildElement("boardGalgo")->FirstChildElement("parameters")->Attribute("leftLegsDevPath");
    configSrv.FirstChildElement("boardGalgo")->FirstChildElement("parameters")->QueryIntAttribute("baudRate",&baudRate);
}

void BoardGalgo::preparePortHandler( const tPortHandler& portHandler,
                                     int baudRate ) {
  if( !portHandler->openPort() )
      throw FailedOpeningPortException( portHandler->getPortName() );
  if( !portHandler->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException(baudRate);
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
    ++legNo;
    ++jointNo;
    tId dynamixel = convert( legNo, jointNo );
    uint8_t error;
    int result = packetHandler_->reboot( portHandlersByLegNumber_.at( legNo ).get(), dynamixel,
            &error );
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
}

BoardGalgo::tId BoardGalgo::convert( int legNo, int jointNo ) {
    return static_cast< tId >( legNo * 10 + jointNo );
}
std::vector< BoardGalgo::tId > BoardGalgo::getSingleLegIds( int legNo ) {
    std::vector< tId > jointsIds( JOINTS_COUNT_IN_SINGLE_LEG );
    int jointNumber = FIRST_JOINT_NUMBER;
    std::generate( jointsIds.begin(), jointsIds.end(),
            [ this, legNo, &jointNumber ]() {
                return convert( legNo, jointNumber++ );
            } );
    return jointsIds;
}
std::vector< BoardGalgo::tId >
        BoardGalgo::getTwoLegsIds( int legNo1, int legNo2 ) {
    return merge( getSingleLegIds( legNo1 ), getSingleLegIds( legNo2 ) );
}
std::vector< BoardGalgo::tId > BoardGalgo::getRightLegsIds() {
    return getTwoLegsIds( 1, 2 );
}
std::vector< BoardGalgo::tId > BoardGalgo::getLeftLegsIds() {
    return getTwoLegsIds( 3, 4 );
}

void BoardGalgo::setTorque( int legNo, int jointNo, uint8_t boolean ) {
    uint8_t error;
    int result = packetHandler_->write1ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), TORQUE_ENABLE, boolean, &error );
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
}
void BoardGalgo::setTorque( int legNo,
        const std::vector< uint8_t >& boolean ) {
    dynamixel3wrapper::SyncWriter< uint8_t > writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            TORQUE_ENABLE );
    auto receivers = getSingleLegIds( legNo );
    writer.write( receivers, boolean );
}
void BoardGalgo::setTorque( const std::vector< uint8_t >& boolean ) {
    dynamixel3wrapper::SyncWriter< uint8_t > rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            TORQUE_ENABLE );
    dynamixel3wrapper::SyncWriter< uint8_t > leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            TORQUE_ENABLE );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto it = rightWriter.write( rightReceivers, boolean.begin() );
    leftWriter.write( leftReceivers, it );
}

void BoardGalgo::setLED(int legNo, int jointNo, uint8_t boolean){
    ++legNo;
    ++jointNo;
    tId id = convert(legNo, jointNo);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandlersByLegNumber_.at(legNo).get(), id, LED, boolean, &dxl_error);
    dynamixel3wrapper::CommunicationResult communicationResult(packetHandler_.get(),
            dxl_comm_result, dxl_error);
    communicationResult.handle();
}

void BoardGalgo::setLED(int legNo, const std::vector<uint8_t>& boolean){
    ++legNo;
    dynamixel3wrapper::SyncWriter< uint8_t > writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            LED );
    auto receivers = getSingleLegIds( legNo );
    writer.write( receivers, boolean );
}

void BoardGalgo::setLED(const std::vector<uint8_t> &boolean){
    dynamixel3wrapper::SyncWriter< uint8_t > rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            LED );
    dynamixel3wrapper::SyncWriter< uint8_t > leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            LED );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto it = rightWriter.write( rightReceivers, boolean.begin() );
    leftWriter.write( leftReceivers, it );
}

void BoardGalgo::setOperatingMode(int legNo, int jointNo, uint8_t operatingMode){
    ++legNo;
    ++jointNo;
    setTorque( legNo, jointNo, 0 );
    uint8_t error;
    int result = packetHandler_->write1ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), OPERATING_MODE, operatingMode, &error );
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
    setTorque( legNo, jointNo, 1 );
}

void BoardGalgo::setOperatingMode(int legNo, const std::vector<uint8_t>& operatingMode){
    ++legNo;
    setTorque( legNo, std::vector< uint8_t >( JOINTS_COUNT_IN_SINGLE_LEG, 0 ) );
    dynamixel3wrapper::SyncWriter< uint8_t > writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            OPERATING_MODE );
    auto receivers = getSingleLegIds( legNo );
    writer.write( receivers, operatingMode );
    setTorque( legNo, std::vector< uint8_t >( JOINTS_COUNT_IN_SINGLE_LEG, 1 ) );
}

void BoardGalgo::setOperatingMode(const std::vector<uint8_t>& operatingMode){
    setTorque( std::vector< uint8_t >( 4 * JOINTS_COUNT_IN_SINGLE_LEG, 0 ) );
    dynamixel3wrapper::SyncWriter< uint8_t > rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            OPERATING_MODE );
    dynamixel3wrapper::SyncWriter< uint8_t > leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            OPERATING_MODE );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto it = rightWriter.write( rightReceivers, operatingMode.begin() );
    leftWriter.write( leftReceivers, it );
    setTorque( std::vector< uint8_t >( 4 * JOINTS_COUNT_IN_SINGLE_LEG, 1 ) );
}

unsigned int BoardGalgo::setPosition(int legNo, int jointNo, double angle){
    ++legNo;
    ++jointNo;
    uint8_t error;
    int result = packetHandler_->write4ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), GOAL_POSITION, tAngleDynamixel( tAngleRadians( angle + M_PI ) ).val(), &error );
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
    return 0;
}
unsigned int BoardGalgo::setPosition(int legNo, const std::vector<double>& angle){
    ++legNo;
    dynamixel3wrapper::SyncWriter< uint32_t > writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            GOAL_POSITION );
    auto receivers = getSingleLegIds( legNo );
    writer.write( receivers, angle, []( double value ){
        return tAngleDynamixel( tAngleRadians( value + M_PI ) ).val();
    } );
    return 0;
}
unsigned int BoardGalgo::setPosition(const std::vector<double>& angle){
    dynamixel3wrapper::SyncWriter< uint32_t > rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            GOAL_POSITION );
    dynamixel3wrapper::SyncWriter< uint32_t > leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            GOAL_POSITION );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto converter = []( double value ){
        return tAngleDynamixel( tAngleRadians( value + M_PI ) ).val();
    };
    auto it = rightWriter.write( rightReceivers, angle.begin(), converter );
    leftWriter.write( leftReceivers, it, converter );
    return 0;
}

unsigned int BoardGalgo::setSpeed(int legNo, int jointNo, double speed){
    ++legNo;
    ++jointNo;
    uint8_t error;
    int result = packetHandler_->write4ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), PROFILE_VELOCITY, tSpeedDynamixel( tSpeedInterval( speed ) ).val(), &error );
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
    return 0;
}

unsigned int BoardGalgo::setSpeed(int legNo, const std::vector<double>& speed){
    ++legNo;
    dynamixel3wrapper::SyncWriter< uint32_t > writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            PROFILE_VELOCITY );
    auto receivers = getSingleLegIds( legNo );
    writer.write( receivers, speed, []( double value ){
        return tSpeedDynamixel( tSpeedInterval( value ) ).val();
    } );
    return 0;
}

unsigned int BoardGalgo::setSpeed(const std::vector<double>& speed){
    dynamixel3wrapper::SyncWriter< uint32_t > rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            PROFILE_VELOCITY );
    dynamixel3wrapper::SyncWriter< uint32_t > leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            PROFILE_VELOCITY );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto converter = []( double value ){
        return tSpeedDynamixel( tSpeedInterval( value ) ).val();
    };
    auto it = rightWriter.write( rightReceivers, speed.begin(), converter );
    leftWriter.write( leftReceivers, it, converter );
    return 0;
}

unsigned int BoardGalgo::setComplianceMargin(int legNo, int jointNo, double margin){
    throw NotSupportedException();
}
unsigned int BoardGalgo::setComplianceMargin(int legNo, const std::vector<double> margin){
    throw NotSupportedException();
}
unsigned int BoardGalgo::setComplianceMargin(const std::vector<double> margin){
    throw NotSupportedException();
}

unsigned int BoardGalgo::setComplianceSlope(int legNo, int jointNo, double slope){
    throw NotSupportedException();
}
unsigned int BoardGalgo::setComplianceSlope(int legNo, const std::vector<double>& slope){
    throw NotSupportedException();
}
unsigned int BoardGalgo::setComplianceSlope(const std::vector<double>& slope){
    throw NotSupportedException();
}

unsigned int BoardGalgo::setTorqueLimit(int legNo, int jointNo, double torqueLimit){
    ++legNo;
    ++jointNo;
    uint8_t error;
    int result = packetHandler_->write2ByteTxRx( portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), GOAL_CURRENT, tCurrentDynamixel( tCurrentInterval( torqueLimit ) ).val(), &error );
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
    return 0;
}
unsigned int BoardGalgo::setTorqueLimit(int legNo, const std::vector<double>& torqueLimit){
    ++legNo;
    dynamixel3wrapper::SyncWriter< uint16_t > writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            GOAL_CURRENT );
    auto receivers = getSingleLegIds( legNo );
    writer.write( receivers, torqueLimit, []( double value ){
        return tCurrentDynamixel( tCurrentInterval( value ) ).val();
    } );
    return 0;
}
unsigned int BoardGalgo::setTorqueLimit(const std::vector<double>& torqueLimit){
    dynamixel3wrapper::SyncWriter< uint16_t > rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            GOAL_CURRENT );
    dynamixel3wrapper::SyncWriter< uint16_t > leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            GOAL_CURRENT );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto converter = []( double value ){
        return tCurrentDynamixel( tCurrentInterval( value ) ).val();
    };
    auto it = rightWriter.write( rightReceivers, torqueLimit.begin(), converter );
    leftWriter.write( leftReceivers, it, converter );
    return 0;
}

unsigned int BoardGalgo::readPosition(int legNo, int jointNo, double& angle){
    ++legNo;
    ++jointNo;
    uint32_t presentPosition;
    uint8_t error;
    int result = packetHandler_->read4ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), PRESENT_POSITION, &presentPosition, &error);
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error );
    communicationResult.handle();
    angle = tAngleRadians( tAngleDynamixel( presentPosition ) ).val() - M_PI;
    return 0;
}

unsigned int BoardGalgo::readPosition(int legNo, std::vector<double>& angle){
    ++legNo;
    dynamixel3wrapper::SyncReader< uint32_t > reader( portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), PRESENT_POSITION );
    auto receivers = getSingleLegIds( legNo );
    angle = reader.read< double >( receivers, []( uint32_t value ){
        return tAngleRadians( tAngleDynamixel( value ) ).val() - M_PI;
    } );
    return 0;
}

unsigned int BoardGalgo::readPosition(std::vector<double>& angle){
    dynamixel3wrapper::SyncReader< uint32_t > rightReader( rightLegs_.get(), packetHandler_.get(), PRESENT_POSITION );
    dynamixel3wrapper::SyncReader< uint32_t > leftReader( leftLegs_.get(), packetHandler_.get(), PRESENT_POSITION );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto converter = []( uint32_t value ){
        return tAngleRadians( tAngleDynamixel( value ) ).val() - M_PI;
    };
    angle = unsortedMerge( rightReader.read< double >( rightReceivers, converter ),
            leftReader.read< double >( leftReceivers, converter ) );
    return 0;
}

unsigned int BoardGalgo::readForce(int legNo, double& contactForce){
    throw NotSupportedException();
}
unsigned int BoardGalgo::readForce(const std::vector<double>& contactForce){
    throw NotSupportedException();
}

unsigned int BoardGalgo::readTorqueForce(int legNo, walkers::TorqueForce& valueTF){
    throw NotSupportedException();
}
unsigned int BoardGalgo::readTorqueForce(const std::vector<double>& valueTF){
    throw NotSupportedException();
}

bool BoardGalgo::readContact(int legNo){
    throw NotSupportedException();
}
void BoardGalgo::readContacts(std::vector<bool>& contact){
    throw NotSupportedException();
}

unsigned int BoardGalgo::readCurrent(int legNo, int jointNo, double& servoCurrent){
    ++legNo;
    ++jointNo;
    uint16_t presentCurrent;
    uint8_t error;
    int result = packetHandler_->read2ByteTxRx(portHandlersByLegNumber_.at( legNo ).get(), convert( legNo, jointNo ), PRESENT_CURRENT, &presentCurrent, &error);
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error );
    communicationResult.handle();
    servoCurrent = tCurrentAmpers( tCurrentDynamixel( presentCurrent ) ).val();
    return 0;
}

unsigned int BoardGalgo::readCurrent(int legNo, std::vector<double>& servoCurrent){
    ++legNo;
    dynamixel3wrapper::SyncReader< uint16_t > reader( portHandlersByLegNumber_.at( legNo ).get(), packetHandler_.get(), PRESENT_CURRENT );
    auto receivers = getSingleLegIds( legNo );
    servoCurrent = reader.read< double >( receivers, []( uint16_t value ){
        return tCurrentAmpers( tCurrentDynamixel( value ) ).val();
    } );
    return 0;
}

unsigned int BoardGalgo::readCurrent(std::vector<double>& servoCurrent){
    dynamixel3wrapper::SyncReader< uint16_t > rightReader( rightLegs_.get(), packetHandler_.get(), PRESENT_CURRENT );
    dynamixel3wrapper::SyncReader< uint16_t > leftReader( leftLegs_.get(), packetHandler_.get(), PRESENT_CURRENT );
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto converter = []( uint16_t value ){
        return tCurrentAmpers( tCurrentDynamixel( value ) ).val();
    };
    servoCurrent = unsortedMerge( rightReader.read< double >( rightReceivers, converter ),
            leftReader.read< double >( leftReceivers, converter ) );
    return 0;
}

unsigned int BoardGalgo::readTorque(int legNo, int jointNo, double& servoTorque){
    throw NotSupportedException();
}
unsigned int BoardGalgo::readTorque(int legNo,std::vector<double>& servoTorque){
    throw NotSupportedException();
}
unsigned int BoardGalgo::readTorque(std::vector<double>& servoTorque){
    throw NotSupportedException();
}

int BoardGalgo::convertToIndex(int legNo, int jointNo){
    return (legNo - 1) * 3 + jointNo - 1;
}

void BoardGalgo::setOffset(int legNo, int jointNo, double offset){
    ++legNo;
    ++jointNo;
    tAngleDynamixel convertedOffset = tAngleRadians( offset );

    angleOffset[convertToIndex(legNo, jointNo)] = static_cast< int >( convertedOffset.val() );

    setTorque(legNo, jointNo, false);
    uint8_t error;
    int result = packetHandler_->write4ByteTxRx(portHandlersByLegNumber_.at(legNo).get(),
            convert(legNo, jointNo), HOMING_OFFSET, convertedOffset.val(), &error);
    dynamixel3wrapper::CommunicationResult communicationResult( packetHandler_.get(), result, error);
    communicationResult.handle();
}

void BoardGalgo::setOffset(int legNo, const std::vector<double> offset){
    ++legNo;
    dynamixel3wrapper::SyncWriter<uint32_t> writer(
            portHandlersByLegNumber_.at(legNo).get(), packetHandler_.get(),
            HOMING_OFFSET);
    auto receivers = getSingleLegIds(legNo);
    writer.write(receivers, offset, [](double value){
        return tAngleDynamixel(tAngleRadians(value)).val();
    });
}

void BoardGalgo::setOffset(const std::vector<double> offset){
    dynamixel3wrapper::SyncWriter<uint32_t> rightWriter(
            rightLegs_.get(), packetHandler_.get(),
            HOMING_OFFSET);
    dynamixel3wrapper::SyncWriter<uint32_t> leftWriter(
            leftLegs_.get(), packetHandler_.get(),
            HOMING_OFFSET);
    auto rightReceivers = getRightLegsIds();
    auto leftReceivers = getLeftLegsIds();
    auto converter = [](double value){
        return tAngleDynamixel(tAngleRadians(value)).val();
    };
    auto it = rightWriter.write(rightReceivers, offset.begin(), converter);
    leftWriter.write(leftReceivers, it, converter);
}

void BoardGalgo::setDefault(void){
    throw NotSupportedException();
}

Board* createBoardGalgo( const std::string &rightLegsDevPath,
                         const std::string &leftLegsDevPath,
                         int baudRate ) {
    boardGalgo.reset( new BoardGalgo( rightLegsDevPath, leftLegsDevPath, baudRate ) );
    return boardGalgo.get();
}

Board* createBoardGalgo(std::string configFilename) {
    boardGalgo.reset( new BoardGalgo(configFilename) );
    return boardGalgo.get();
}

} // namespace controller
