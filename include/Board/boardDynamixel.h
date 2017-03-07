/** @file boardDynamixel.h
 *
 * implementation - Board with usb2dynamixel
 * Author1: Pawel Szczygiel
 */

#ifndef BOARDDYNAMIXEL_H_INCLUDED
#define BOARDDYNAMIXEL_H_INCLUDED

#include "board.h"
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>
#include "../3rdParty/dynamixel/dynamixel.h"
#include <array>
#include <cmath>

#define DEFAULT_BAUDNUM 1
#define MOVE_SERWOMOTOR 0x1E
#define P_PRESENT_POSITION_L 36
#define TORQUE 40

#define MOVING_SPEED 32
#define PRESENT_VOLTAGE 42      //used readCurrent

#define LEG_0 0
#define LEG_1 1
#define LEG_2 2
#define LEG_3 3
#define LEG_4 4
#define LEG_5 5

#define JOINT_0 0
#define JOINT_1 1
#define JOINT_2 2

#define P_MOVING_SPEED_L 32 /*!< wartosc predkosci serwomechanizmu */
#define P_CW_COMPLIANCE_MARGIN 26 /*!< blad pomiedzy zadana a aktualna pozycja, w ruchu zgodnie ze wskazowkami zegara (CW) */
#define P_CCW_COMPLIANCE_MARGIN 27 /*!< blad pomiedzy zadana a aktualna pozycja, w ruchu przeciwnym do wskazowek zegara (CCW) */
#define P_CW_COMPLIANCE_SLOPE 28 /*!< wartosc momentu serwomechanizmu gdy zbliza sie juz do zadanego polozenia, CW */
#define P_CCW_COMPLIANCE_SLOPE 29 /*!< wartosc momentu serwomechanizmu gdy zbliza sie juz do zadanego polozenia, CCW */
#define P_TORQUE_LIMIT_L 34 //maximum output torque
#define P_TEMERATURE_LIMIT_L 11 /* temperature limit*/

#define SET_TORQUE_LIMIT 34 //adres do ustawienia ograniczenia momentu
#define GET_MAX_TORQUE 14 //adres odczytu maxymalnego momentu

#define _DEG2RAD10              (1800/M_PI)
#define _RAD2DEG10                (M_PI/1800)
#define _DEG2DYNAMIXEL          (0.341333)
#define _OFFSETANGLE            512

#define READ_WORD               0
#define WRITE_WORD              1
#define READ_BYTE               2
#define WRITE_BYTE              3
#define INITIALIZE              4
#define TERMINATE               5

namespace controller {

    /// create a single board controller (with usb2dynamixel)
    Board* createBoardDynamixel(void);
}

using namespace controller;


/**
 * \brief Board implementation.
 */
class BoardDynamixel : public Board{
    public:
        static const double DEG2DYNAMIXEL;
        /// Pointer
        typedef std::unique_ptr<BoardDynamixel> Ptr;

        /// Construction
        BoardDynamixel(void);

        /// Destructor
        ~BoardDynamixel(void);
        /**
         * \brief Set reference position value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param angle Angle value.
         * \return Return error value.
         */

        unsigned int setPosition(int legNo, int jointNo, double angles);
        //CDynamixel obiect;
        /**
         * \brief Set reference position value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
        unsigned int setPosition(int legNo, const std::vector<double>& angles);
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
         * \brief Set torque limit for serwomotors in particular leg.
         * \details [0,1023] - the torque limit for servos.
         * \param legNo Leg number.
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
         * \param reversePos1 if 1st jointNo should be reversed
         * \return Return error value.
         */
        unsigned int readPosition(int legNo, std::vector<double>& angles);

        /**
         * \brief Returns current position of the servomotors.
         * \param &angle Angle values.
         * \return Return error value.
         */
        unsigned int readPosition(std::vector<double>& angles);

        /**
         * \brief Returns contact force from 3-axis force sensor in particular leg.
         * \param legNo Leg number.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
        unsigned int readForce(int legNo, double& contactForce);

        // Niepotrzebna funkcja - nie ruszać.
        /**
         * \brief Returns contact forces from 3-axis force sensors.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
        unsigned int readForce(const std::vector<double>& contactForce);

        // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(int legNo, walkers::TorqueForce& valueTF);

        // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact force from 3-axis torque/force sensor
        unsigned int readTorqueForce(const std::vector<double>& valueTF);

        /// Returns value from microswitch mounted on the leg's tip
        bool readContact(int legNo);

        /// Returns values from microswitches mounted on the leg's tip
        void readContacts(std::vector<bool>& contacts);

        /**
         * \brief Returns current value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoCurrent Current value.
         * \return Return error value.
         */
        unsigned int readCurrent(int legNo, int jointNo, double& servoCurrent);

        /**
         * \brief Returns current values from servos in particular leg.
         * \param legNo Leg number.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
        unsigned int readCurrent(int legNo, std::vector<double>& servoCurrent);

        /**
         * \brief Returns current value from servos.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
        unsigned int readCurrent(std::vector<double>& servoCurrent);

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
         * \param offset Offset value.
         * \return Return error value.
         */
        void setOffset(int legNo, int jointNo, double offset);
        /**
         * \brief Returns offset of servos in particular leg.
         * \param legNo Leg number.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
        void setOffset(int legNo, const std::vector<double> offset);
        /**
         * \brief Returns offset of servos.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
        void setOffset(const std::vector<double> offset);
        /**
         * \brief Set default value.
         */
        void setDefault(void);

    private:
        /// Two objects for executing the operations
        CDynamixel dynamixelMotors[2];
        /// GPIO files
        int fileDescriptorsGPIO[6];
        /**
         * \brief Function replacing bibliotecal dxl_write_word
         * \param usd2dynNo Usb number
         * \param dynamixelCmd e.g. write_word, read_word, initialize
         * \param servoNo servomotor number
         * \param command e.g. movingSpeed, complianceMargin
         * \param value value of command
        */
        double  sendCommand(int dynamixelCmd, int usb2dynNo, int servoNo, int command, double value);
        /// angle to dynamixel format
        double angle2dynamixel(double angle, int legNo, int jointNo) const;
        /// get results
        unsigned int getResult(int usb2dynNo);
        /// get error usb2dynamixel
        unsigned int getPacketError(int usb2dynNo);
        /// print dynamixel error
        void printDynamixelError(unsigned int errorNo);
        /// mutexes to prevent collision for usb2dynamixel devices
        std::array<std::mutex,2> mtxs;
        /// Default values of angles for serwomotors.
        int zero_angle[18];
        //std::vector <double> zero_angle;  //rad
        /// Default offset values of angles for serwomotors.
        int angle_offset[18];
        //std::vector<double> angle_offset; //rad
};

#endif // BOARDDYNAMIXEL_H_INCLUDED
