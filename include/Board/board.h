/** @file board.h
 *
 * Robot board interface
 *
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <iostream>
#include <string>
#include <vector>
#include <array>

namespace walkers {

    /// 3 element vector class
    typedef std::array<double,3> Vec3;

    class TorqueForce {
    public:
        ///force values
        Vec3 force;

        ///torque values
        Vec3 torque;

        ///susceptibility values
        std::vector<double> susceptibility;

    };

}

namespace controller {
    /// Board interface
    class Board{
        public:

            /// Board type
            enum Type {
                    /// Board with SPI interface (Messor)
                    TYPE_SPIBASED,
                    /// Board with usb2dynamixel (Messor2)
                    TYPE_USB2DYNAMIXEL,
                    /// Board Galgo
                    TYPE_GALGO
            };

            /// Board controller state
            class ControllerState{
                public:
                    /// servo neutral value (middle pose)
                    std::vector<double> neutralAngle;

                    /// servo neutral value (middle pose)
                    std::vector<double> offsetAngle;

                    /// reference position
                    std::vector<double> refAngle;

                    /// reference speed
                    std::vector<double> refSpeed;

                    /// reference compliance slope
                    std::vector<double> refComplianceSlope;

                    /// reference compliance slope
                    std::vector<double> refComplianceMargin;

                    /// reference torque limit
                    std::vector<double> refTorqueLimit;

                    /// Default constructor
                    inline ControllerState() {
                    }
            };

            /// overloaded constructor
            Board(const std::string _name, Type _type) : name(_name), type(_type) {}

            /// Name of the board
            virtual const std::string& getName() const { return name; }

 /**
         * \brief Set reference position value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param angle Angle value.
         * \return Return error value.
         */
            virtual unsigned int setPosition(int legNo, int jointNo, double angle) = 0;
            /**
         * \brief Set reference position value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
            virtual unsigned int setPosition(int legNo, const std::vector<double>& angle) = 0;
            /**
         * \brief Set reference position value for all serwomotors.
         * \param &angle Vector of joint angles.
         * \return Return error value.
         */
            virtual unsigned int setPosition(const std::vector<double>& angle) = 0;

            /**
         * \brief Set reference speed value for servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param speed Speed value.
         * \return Return error value.
         */
            virtual unsigned int setSpeed(int legNo, int jointNo, double speed) = 0;
            /**
         * \brief Set reference speed value for all serwomotors in particular leg.
         * \param legNo Leg number.
         * \param &speed Vector of joint speeds.
         * \return Return error value.
         */
            virtual unsigned int setSpeed(int legNo, const std::vector<double>& speed) = 0;
            /**
         * \brief Set reference speed value for all serwomotors.
         * \param &speed Vector of joint speeds.
         * \return Return error value.
         */
            virtual unsigned int setSpeed(const std::vector<double>& speed) = 0;

            /**
         * \brief Set compliance margin for servomotor.
         * \details [0,254]- dead zone -- for this area the torque is zero.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param margin Compliance margin.
         * \return Return error value.
         */
            virtual unsigned int setComplianceMargin(int legNo, int jointNo, double margin) = 0;
            /**
         * \brief Set compliance margins for all serwomotors in particular leg.
         * \details [0,254]- dead zone -- for this area the torque is zero, returns error value
         * \param legNo Leg number
         * \param margin Vector of compliance margins.
         * \return Return error value.
         */
            virtual unsigned int setComplianceMargin(int legNo, const std::vector<double> margin) = 0;
            /**
         * \brief Set compliance margins for all serwomotors.
         * \param margin Vector of compliance margins.
         * \return Return error value.
         */
            virtual unsigned int setComplianceMargin(const std::vector<double> margin) = 0;

            /**
         * \brief Set compliance slope for serwomotor.
         * \details [1,254] - the area with the reduced torque
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param slope Compiance slope.
         * \return Return error value.
         */
            virtual unsigned int setComplianceSlope(int legNo, int jointNo, double slope) = 0;
            /**
         * \brief Set compliance slope for all serwomotors in particular leg.
         * \details [1,254]- the area with the reduced torque.
         * \param legNo Leg number.
         * \param &slope Vector of compilace slopes.
         * \return Return error value.
         */
            virtual unsigned int setComplianceSlope(int legNo, const std::vector<double>& slope) = 0;
            /**
         * \brief Set compliance slope for all serwomotors.
         * \details [1,254]- the area with the reduced torque.
         * \param &slope Vector of compilace slopes.
         * \return Return error value.
         */
            virtual unsigned int setComplianceSlope(const std::vector<double>& slope) = 0;

           /**
         * \brief Set torque limit for serwomotor.
         * \details [0,1023] - the torque limit.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param torqueLimit Torque limit.
         * \return Return error value.
         */
            virtual unsigned int setTorqueLimit(int legNo, int jointNo, double torqueLimit) = 0;
            /**
         * \brief Set torque limit for serwomotors.
         * \details [0,1023] - the torque limit for servos.
         * \param &torqueLimit Vector of Torque limits.
         * \return Return error value.
         */
            virtual unsigned int setTorqueLimit(int legNo, const std::vector<double>& torqueLimit) = 0;
            /**
         * \brief Set torque limit for serwomotors.
         * \details [0,1023] - the torque limit for servos.
         * \param &torqueLimit Vector of Torque limits.
         * \return Return error value.
         */
            virtual unsigned int setTorqueLimit(const std::vector<double>& torqueLimit) = 0;

              /**
         * \brief Returns current position of the servomotor.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &angle Angle value.
         * \return Return error value.
         */
            virtual unsigned int readPosition(int legNo, int jointNo, double& angle) = 0;
            /**
         * \brief Returns current position of the servomotors in particular leg.
         * \param legNo Leg number.
         * \param &angle Angle values.
         * \return Return error value.
         */
            virtual unsigned int readPosition(int legNo, std::vector<double>& angle) = 0;
              /**
         * \brief Returns current position of the servomotors.
         * \param &angle Angle values.
         * \return Return error value.
         */
            virtual unsigned int readPosition(std::vector<double>& angle) = 0;

            /**
         * \brief Returns contact force from 3-axis force sensor in particular leg.
         * \param legNo Leg number.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
            virtual unsigned int readForce(int legNo, double& contactForce) = 0;
            /// Niepotrzebna funkcja - nie ruszać.
        /**
         * \brief Returns contact forces from 3-axis force sensors.
         * \param &contactForce Contact force value.
         * \return Return error value.
         */
            virtual unsigned int readForce(const std::vector<double>& contactForce) = 0;

           // Niepotrzebna funkcja - nie ruszać.
        /// Returns contact force from 3-axis torque/force sensor
            virtual unsigned int readTorqueForce(int legNo, walkers::TorqueForce& valueTF)= 0;
            /// Returns contact force from 3-axis torque/force sensor
            virtual unsigned int readTorqueForce(const std::vector<double>& valueTF)= 0;
            /// Returns value from microswitch mounted on the leg's tip
            virtual bool readContact(int legNo) = 0;
            ///Returns value from microswitch mounted on the leg's tip
            virtual void readContacts(std::vector<bool>& contact) = 0;

            /**
         * \brief Returns current value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoCurrent Current value.
         * \return Return error value.
         */
            virtual unsigned int readCurrent(int legNo, int jointNo, double& servoCurrent) = 0;
            /**
         * \brief Returns current value from servos.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
            virtual unsigned int readCurrent(int legNo, std::vector<double>& servoCurrent) = 0;
            /**
         * \brief Returns current value from servos.
         * \param &servoCurrent Current values.
         * \return Return error value.
         */
            virtual unsigned int readCurrent( std::vector<double>& servoCurrent) = 0;

            /**
         * \brief Returns torque(load) value from servo.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param &servoTorque Torque(load) value.
         * \return Return error value.
         */
            virtual unsigned int readTorque(int legNo, int jointNo, double& servoTorque) = 0;
             /**
         * \brief Returns torque(load) value from servos in particular leg.
         * \param legNo Leg number.
         * \param &servoTorque Torque(load) values.
         * \return Return error value.
         */
            virtual unsigned int readTorque(int legNo,std::vector<double>& servoTorque) = 0;
             /**
         * \brief Returns torque(load) value from servos.
         * \param &servoTorque Torque(load) values.
         * \return Return error value.
         */
            virtual unsigned int readTorque(std::vector<double>& servoTorque) = 0;

             /**
         * \brief Returns servo's offset.
         * \param legNo Leg number.
         * \param jointNo Joint number.
         * \param offset Offset value.
         * \return Return error value.
         */
            virtual void setOffset(int legNo, int jointNo, double offset) = 0;
            /**
         * \brief Returns offset of servos in particular leg.
         * \param legNo Leg number.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
            virtual void setOffset(int legNo, const std::vector<double> offset) = 0;
            /**
         * \brief Returns offset of servos.
         * \param offset Vector of offset values.
         * \return Return error value.
         */
            virtual void setOffset(const std::vector<double> offset) = 0;

            /**
         * \brief Set default value.
         */
            virtual void setDefault(void) = 0;

            /// Virtual descrutor
            virtual ~Board() {}

        protected:
            /// Board name
            const std::string name;

            /// Board type
            Type type;

            /// The state of the controller
            ControllerState state;
    };
}

#endif // _BOARD_H_
