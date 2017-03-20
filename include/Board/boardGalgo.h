#ifndef _BOARD_GALGO_H_
#define _BOARD_GALGO_H_

#include "../../3rdParty/dynamixel3/include/dynamixel_sdk.h"
#include "board.h"

namespace controller {

    class BoardGalgo : Board {
        private:
            typedef uint16_t tAddress;
            const tAddress torqueEnable;
            // TO-DO Zamień nazwę na Led
            const tAddress ADDR_LED;
        public:
	  BoardGalgo( const std::string &port, int baudRate );
	  ~BoardGalgo();

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

            void setLED(int legNo, int jointNo, bool powered);

        /**
         * \brief Set default value.
         */
            void setDefault(void);

	private:
      void toggleTorque( uint8_t dynamixelId, bool onOrOff );
      const float protocolVersion_;
      dynamixel::PortHandler *portHandler_;
    };
}

#endif // _BOARD_GALGO_H_
