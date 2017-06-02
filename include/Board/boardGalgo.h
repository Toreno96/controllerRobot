#ifndef _BOARD_GALGO_H_
#define _BOARD_GALGO_H_

#include <array>
#include <map>
#include <memory>
#include "../../3rdParty/dynamixel3/include/dynamixel_sdk.h"
#include "board.h"
#include "Helpers/angle.h"
#include "Helpers/current.h"
#include "Helpers/speed.h"
#include "Wrappers/dynamixel3/globals.h"

namespace controller {

/// create a single board controller (with usb2dynamixel)
Board* createBoardGalgo( const std::string &rightLegsDevPath,
                         const std::string &leftLegsDevPath,
                         int baudRate );

Board* createBoardGalgo(std::string configFilename);

class BoardGalgo : public Board {
    public:
        using Ptr = std::unique_ptr< BoardGalgo >;
        BoardGalgo( const std::string &rightLegsDevPath,
                    const std::string &leftLegsDevPath,
                    int baudRate );

        BoardGalgo(std::string configFilename);

        ~BoardGalgo();

        class Config {
            public:

                Config(){
                }

                Config(std::string configFilename) {
                    load(configFilename);
                }

                void load(std::string configFilename);

                /// rightLegsDevPath
                std::string rightLegsDevPath;
                /// rightLegsDevPath
                std::string leftLegsDevPath;
                /// baudrate
                int baudRate;
        };

        // TO-DO Dokumentacja doxygen poniższych funkcji setLED
        void setLED(int legNo, int jointNo, uint8_t boolean);
        void setLED(int legNo, const std::vector<uint8_t>& boolean);
        void setLED(const std::vector<uint8_t>& boolean);

        void setOperatingMode(int legNo, int jointNo, uint8_t operatingMode);
        void setOperatingMode(int legNo, const std::vector<uint8_t>& operatingMode);
        void setOperatingMode(const std::vector<uint8_t>& operatingMode);

        void reboot( int legNo, int jointNo );
        
       /**
        * \brief Set reference position value for servomotor.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param angle Angle value.
        * \return Return error value.
        */
        unsigned int setPosition(int legNo, int jointNo, double angle);
       /**
        * \brief Set reference position value for all serwomotors in particular leg.
        * \param legNo Leg number.
        * \param &angle Vector of joint angles.
        * \return Return error value.
        */
        unsigned int setPosition(int legNo, const std::vector<double>& angle);
       /**
        * \brief Set reference position value for all serwomotors.
        * \param &angle Vector of joint angles.
        * \return Return error value.
        */
        unsigned int setPosition(const std::vector<double>& angle);

       /**
        * \brief Set reference speed value for servomotor.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param speed Speed value.
        * \return Return error value.
        */
        unsigned int setSpeed(int legNo, int jointNo, double speed);
       /**
        * \brief Set reference speed value for all serwomotors in particular leg.
        * \param legNo Leg number.
        * \param &speed Vector of joint speeds.
        * \return Return error value.
        */
        unsigned int setSpeed(int legNo, const std::vector<double>& speed);
       /**
        * \brief Set reference speed value for all serwomotors.
        * \param &speed Vector of joint speeds.
        * \return Return error value.
        */
        unsigned int setSpeed(const std::vector<double>& speed);

       /**
        * \brief Set compliance margin for servomotor.
        * \details [0,254]- dead zone -- for this area the torque is zero.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param margin Compliance margin.
        * \return Return error value.
        */
        unsigned int setComplianceMargin(int legNo, int jointNo, double margin);
       /**
        * \brief Set compliance margins for all serwomotors in particular leg.
        * \details [0,254]- dead zone -- for this area the torque is zero, returns error value
        * \param legNo Leg number
        * \param margin Vector of compliance margins.
        * \return Return error value.
        */
        unsigned int setComplianceMargin(int legNo, const std::vector<double> margin);
       /**
        * \brief Set compliance margins for all serwomotors.
        * \param margin Vector of compliance margins.
        * \return Return error value.
        */
        unsigned int setComplianceMargin(const std::vector<double> margin);


       /**
        * \brief Set compliance slope for serwomotor.
        * \details [1,254] - the area with the reduced torque
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param slope Compiance slope.
        * \return Return error value.
        */
        unsigned int setComplianceSlope(int legNo, int jointNo, double slope);
       /**
        * \brief Set compliance slope for all serwomotors in particular leg.
        * \details [1,254]- the area with the reduced torque.
        * \param legNo Leg number.
        * \param &slope Vector of compilace slopes.
        * \return Return error value.
        */
        unsigned int setComplianceSlope(int legNo, const std::vector<double>& slope);
       /**
        * \brief Set compliance slope for all serwomotors.
        * \details [1,254]- the area with the reduced torque.
        * \param &slope Vector of compilace slopes.
        * \return Return error value.
        */
        unsigned int setComplianceSlope(const std::vector<double>& slope);

       /**
        * \brief Set torque limit for serwomotor.
        * \details [0,1023] - the torque limit.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param torqueLimit Torque limit.
        * \return Return error value.
        */
        unsigned int setTorqueLimit(int legNo, int jointNo, double torqueLimit);
       /**
        * \brief Set torque limit for serwomotors.
        * \details [0,1023] - the torque limit for servos.
        * \param &torqueLimit Vector of Torque limits.
        * \return Return error value.
        */
        unsigned int setTorqueLimit(int legNo, const std::vector<double>& torqueLimit);
       /**
        * \brief Set torque limit for serwomotors.
        * \details [0,1023] - the torque limit for servos.
        * \param &torqueLimit Vector of Torque limits.
        * \return Return error value.
        */
        unsigned int setTorqueLimit(const std::vector<double>& torqueLimit);

       /**
        * \brief Returns current position of the servomotor.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param &angle Angle value.
        * \return Return error value.
        */
        unsigned int readPosition(int legNo, int jointNo, double& angle);
       /**
        * \brief Returns current position of the servomotors in particular leg.
        * \param legNo Leg number.
        * \param &angle Angle values.
        * \return Return error value.
        */
        unsigned int readPosition(int legNo, std::vector<double>& angle);
       /**
        * \brief Returns current position of the servomotors.
        * \param &angle Angle values.
        * \return Return error value.
        */
        unsigned int readPosition(std::vector<double>& angle);

       /**
        * \brief Returns contact force from 3-axis force sensor in particular leg.
        * \param legNo Leg number.
        * \param &contactForce Contact force value.
        * \return Return error value.
        */
        // Niepotrzebna funkcja - nie ruszać.
        unsigned int readForce(int legNo, double& contactForce);
       /**
        * \brief Returns contact forces from 3-axis force sensors.
        * \param &contactForce Contact force value.
        * \return Return error value.
        */
        // Niepotrzebna funkcja - nie ruszać.
        unsigned int readForce(const std::vector<double>& contactForce);

        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(int legNo, walkers::TorqueForce& valueTF);
        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(const std::vector<double>& valueTF);

        /// Returns value from microswitch mounted on the leg's tip
        bool readContact(int legNo);
        ///Returns value from microswitch mounted on the leg's tip
        void readContacts(std::vector<bool>& contact);

       /**
        * \brief Returns current value from servo.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param &servoCurrent Current value.
        * \return Return error value.
        */
        unsigned int readCurrent(int legNo, int jointNo, double& servoCurrent);
       /**
        * \brief Returns current value from servos.
        * \param &servoCurrent Current values.
        * \return Return error value.
        */
        unsigned int readCurrent(int legNo, std::vector<double>& servoCurrent);
       /**
        * \brief Returns current value from servos.
        * \param &servoCurrent Current values.
        * \return Return error value.
        */
        unsigned int readCurrent( std::vector<double>& servoCurrent);

       /**
        * \brief Returns torque(load) value from servo.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param &servoTorque Torque(load) value.
        * \return Return error value.
        */
        unsigned int readTorque(int legNo, int jointNo, double& servoTorque);
       /**
        * \brief Returns torque(load) value from servos in particular leg.
        * \param legNo Leg number.
        * \param &servoTorque Torque(load) values.
        * \return Return error value.
        */
        unsigned int readTorque(int legNo,std::vector<double>& servoTorque);
       /**
        * \brief Returns torque(load) value from servos.
        * \param &servoTorque Torque(load) values.
        * \return Return error value.
        */
        unsigned int readTorque(std::vector<double>& servoTorque);

       /**
        * \brief Returns servo's offset.
        * \param legNo Leg number.
        * \param jointNo Joint number.
        * \param offset Offset value in radians.
        * \return Return error value.
        */
        void setOffset(int legNo, int jointNo, double offset);
       /**
        * \brief Returns offset of servos in particular leg.
        * \param legNo Leg number.
        * \param offset Vector of offset values in radians.
        * \return Return error value.
        */
        void setOffset(int legNo, const std::vector<double> offset);
       /**
        * \brief Returns offset of servos.
        * \param offset Vector of offset values in radians.
        * \return Return error value.
        */
        void setOffset(const std::vector<double> offset);

       /**
        * \brief Set default value.
        */
        void setDefault(void);

        static constexpr uint8_t OPERATINGMODE_VELOCITY = 1;
        static constexpr uint8_t OPERATINGMODE_POSITION = 3;
        static constexpr uint8_t OPERATINGMODE_CURRENT_BASED_POSITION = 5;

    private:
        Config config;
        using tPortHandler = std::shared_ptr< dynamixel::PortHandler >;
        using tPacketHandler = std::shared_ptr< dynamixel::PacketHandler >;
        using tId = dynamixel3wrapper::tId;
        using tAddress = dynamixel3wrapper::tAddress;
        using tAngleDynamixel = Angle< tAngleUnitDynamixel >;
        using tAngleRadians = Angle< tAngleUnitRadians >;
        using tCurrentDynamixel = Current< tCurrentUnitDynamixel >;
        using tCurrentInterval = Current< tCurrentUnitInterval >;
        using tCurrentAmpers = Current< tCurrentUnitAmpers >;
        using tSpeedInterval = Speed< tSpeedUnitInterval >;
        using tSpeedDynamixel = Speed< tSpeedUnitDynamixel >;
        static constexpr float PROTOCOL_VERSION = 2.0;
        static constexpr tAddress OPERATING_MODE = 11;
        static constexpr tAddress HOMING_OFFSET = 20;
        static constexpr tAddress TORQUE_ENABLE = 64;
        static constexpr tAddress LED = 65;
        static constexpr tAddress GOAL_CURRENT = 102;
        //static constexpr tAddress GOAL_VELOCITY = 104;
        static constexpr tAddress PROFILE_VELOCITY = 112;
        static constexpr tAddress GOAL_POSITION = 116;
        static constexpr tAddress PRESENT_CURRENT = 126;
        static constexpr tAddress PRESENT_VELOCITY	= 128;
        static constexpr tAddress PRESENT_POSITION = 132;
        static constexpr int MAX_SPEED = 1023;
        static constexpr int MAX_CURRENT = 1193;
        static constexpr int FIRST_JOINT_NUMBER = 1;
        static constexpr std::size_t JOINTS_COUNT_IN_SINGLE_LEG = 3;

        void preparePortHandler( const tPortHandler& portHandler, int baudRate );
        void preparePortHandlersByLegNumberMap();

        tId convert( int legNo, int jointNo );
        std::vector< tId > getSingleLegIds( int legNo );
        std::vector< tId > getTwoLegsIds( int legNo1, int legNo2 );
        std::vector< tId > getRightLegsIds();
        std::vector< tId > getLeftLegsIds();
        int convertToIndex(int legNo, int jointNo);

        // TO-DO Dokumentacja doxygen poniższych funkcji setTorque
        void setTorque( int legNo, int jointNo, uint8_t boolean );
        void setTorque( int legNo, const std::vector< uint8_t >& boolean );
        void setTorque( const std::vector< uint8_t >& boolean );

        tPortHandler rightLegs_;
        tPortHandler leftLegs_;
        std::map< int, tPortHandler > portHandlersByLegNumber_;
        tPacketHandler packetHandler_;

        /// Default offset values of angles for serwomotors.
        int angleOffset[12];

        /// Default values of angles for serwomotors.
        int zeroAngle[12];

        int signOfAngle[12];
};

} // namespace controller

#endif // _BOARD_GALGO_H_
