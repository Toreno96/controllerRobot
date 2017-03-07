
#include "Board/boardDynamixel.h"
#include <iostream>
#include "../3rdParty/dynamixel/dynamixel.h"
#include "../3rdParty/dynamixel/dxl_hal.h"
#include "Board/board.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <thread>
#include <mutex>
#include <cstring>

using namespace controller;
using namespace std;
/// A single instance of BoardDynamixel
BoardDynamixel::Ptr boardDynamixel;

BoardDynamixel::BoardDynamixel(void) : Board("Board Dynamixel", TYPE_USB2DYNAMIXEL) {
    //Every operation is executed on two objects in the same time (One object on one side of port)

    for(int i=0 ; i < 2 ; i++){
       int result =  dynamixelMotors[i].dxl_initialize(i+1, DEFAULT_BAUDNUM);

       for (int j=0;j<6;j++){   //deafault values for servos
           if(result == 1) {
               for (int k=0;k<3;k++) {
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_MOVING_SPEED_L,512);
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_CW_COMPLIANCE_MARGIN, 1);
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_CCW_COMPLIANCE_MARGIN, 1);
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_CW_COMPLIANCE_SLOPE, 1);
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_CCW_COMPLIANCE_SLOPE, 1);
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_TORQUE_LIMIT_L, 1012);
                   sendCommand(WRITE_WORD,i,(int)(j*10+k), P_TEMERATURE_LIMIT_L, 99);
               }
           }
       }
    }

    zero_angle[0]=450; zero_angle[1]=240; zero_angle[2]=1140;
    zero_angle[3]=0; zero_angle[4]=240; zero_angle[5]=1140;
    zero_angle[6]=-480; zero_angle[7]=240; zero_angle[8]=1140;
    zero_angle[9]=420; zero_angle[10]=-240; zero_angle[11]=-1140;
    zero_angle[12]=0; zero_angle[13]=-240; zero_angle[14]=-1140;
    zero_angle[15]=-450; zero_angle[16]=-240; zero_angle[17]=-1140;

    angle_offset[0]=-30; angle_offset[1]=30; angle_offset[2]=80;
    angle_offset[3]=265; angle_offset[4]=20; angle_offset[5]=-30;
    angle_offset[6]=-30; angle_offset[7]=125; angle_offset[8]=0;
    angle_offset[9]=60; angle_offset[10]=40; angle_offset[11]=0;
    angle_offset[12]=-190; angle_offset[13]=45; angle_offset[14]=80;
    angle_offset[15]=10; angle_offset[16]=75; angle_offset[17]=100;

    std::cout << "Setting up GPIOs\n";
    /// Setting up ground contacts
    // Enabling GPIOs
    int fd;
    if ((fd = open("/sys/class/gpio/export", O_WRONLY | O_NDELAY, 0)) == 0) {
        std::cout << "Error: Can't open /sys/class/gpio/export.\n";
    }

    char buffer[6][32];
    strcpy(buffer[0],"32");
    strcpy(buffer[1],"33");
    strcpy(buffer[2],"34");
    strcpy(buffer[3],"51");
    strcpy(buffer[4],"36");
    strcpy(buffer[5],"121");

    for(int i=0;i<6;i++){
        ssize_t size =write(fd, buffer[i], strlen(buffer[i]));
        size+=1;
    }
    close(fd);

    // Setting the direction of GPIOs
    for(int i=0;i<6;i++){
        char buff[60];
        sprintf(buff, "/sys/class/gpio/gpio%s/direction",buffer[i]);
        if ((fd = open(buff, O_WRONLY | O_NDELAY,
                0)) == 0) {
            std::cout << "Error: Can't open /sys/class/gpio/gpioX/direction.\n";
            exit(1);
        }
        strcpy(buff, "in");
        ssize_t size = write(fd, buff, strlen(buff));
        size+=1;
        close(fd);
    }

    // Opening the files for switches
    for(int i=0;i<6;i++){
        char buff[60];
        sprintf(buff, "/sys/class/gpio/gpio%s/value",buffer[i]);
        if ((fileDescriptorsGPIO[i] = open(buff, O_RDONLY | O_NDELAY, 0))
                == 0) {
            std::cout << "Error: Can't open /sys/class/gpio/gpioX/value.\n";
            exit(1);
        }
    }
    std::cout << "Added GPIOs (contact sensors)\n";

}

BoardDynamixel::~BoardDynamixel(void) {
    for(int i = 0; i < 2;  i++){
        //CDynamixel *pointMotor = &dynamixelMotors[i];
        //pointMotor->dxl_terminate();     //end of transmision
        sendCommand(TERMINATE,i, 0,0,0);
    }
}

/// get order result
unsigned int BoardDynamixel::getResult(int usb2dynNo){
    mtxs[usb2dynNo].lock();
    unsigned int ret = dynamixelMotors[usb2dynNo].dxl_get_result();
    mtxs[usb2dynNo].unlock();
    return ret;
}

/// print dynamixel error
void BoardDynamixel::printDynamixelError(unsigned int errorNo){
    std::cout << "error no: " << (unsigned int)errorNo <<std::endl;
    switch (errorNo) {
        case 1:
            std::cout << "Input voltage error occurred\n";
            break;
        case 2:
            std::cout << "Angle limit error occurred\n";
            break;
        case 3:
            std::cout << "Overheating error occurred\n";
            break;
        case 4:
            std::cout << "Range error occurred\n";
            break;
        case 5:
            std::cout << "Checksum error occurred\n";
            break;
        case 6:
            std::cout << "Overload error occurred\n";
            break;
        case 7:
            std::cout << "Instruction error occurred\n";
            break;
        default:
            std::cout << "Unknown error\n";
            break;
    }
}

/// get packet error
unsigned int BoardDynamixel::getPacketError(int usb2dynNo){
    mtxs[usb2dynNo].lock();
    if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 1; // Input voltage error occurred
    }
    else if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_ANGLE ) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 2;// Angle limit error occurred
    }
    else if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 3;// Overheating error occurred
    }
    else if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_RANGE) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 4;// Range error occurred
    }
    else if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 5;// Checksum error occurred
    }
    else if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 6;// Overload error occurred
    }
    else if(dynamixelMotors[usb2dynNo].dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1 ){
        mtxs[usb2dynNo].unlock();
        return 7;// Instruction error occurred
    }
    else {
        mtxs[usb2dynNo].unlock();
        return 0;
    }
}

double BoardDynamixel::sendCommand(int dynamixelCmd, int usb2dynNo, int servoNo, int command, double value){
    double temp = 0;
    //READ_WORD
    if(dynamixelCmd == 0){
        mtxs[usb2dynNo].lock();
        temp = dynamixelMotors[usb2dynNo].dxl_read_word(servoNo, command);
        mtxs[usb2dynNo].unlock();
        return temp;
    }
    //WRITE_WORD
    else if(dynamixelCmd ==1){
        mtxs[usb2dynNo].lock();
        dynamixelMotors[usb2dynNo].dxl_write_word(servoNo, command, (int)value);
        mtxs[usb2dynNo].unlock();
        return 0;
    }
    //READ_BYTE
    else if(dynamixelCmd ==2){
        mtxs[usb2dynNo].lock();
        temp = dynamixelMotors[usb2dynNo].dxl_read_byte(servoNo, command);
        mtxs[usb2dynNo].unlock();
        return temp;
    }
    //WRITE_BYTE
    else if(dynamixelCmd ==3){
        mtxs[usb2dynNo].lock();
        dynamixelMotors[usb2dynNo].dxl_write_byte(servoNo, command, (int)value);
        mtxs[usb2dynNo].unlock();
        return 0;
    }
    //INITIALIZE
    else if(dynamixelCmd ==4){
        mtxs[usb2dynNo].lock();
        dynamixelMotors[usb2dynNo].dxl_initialize(servoNo, command);
        mtxs[usb2dynNo].unlock();
        return 0;
    }
    //TERMINATE
    else if(dynamixelCmd ==5){
        mtxs[usb2dynNo].lock();
        dynamixelMotors[usb2dynNo].dxl_terminate();
        mtxs[usb2dynNo].unlock();
        return 0;
    }
    return 1;
 }


/// Set reference position value for servomotor, returns error value
unsigned int BoardDynamixel::setPosition(int legNo, int jointNo, double angle){
    if(legNo<3 && jointNo==2)
         angle=-angle;
    if(legNo>2 && jointNo==1)
         angle=-angle;

    angle = angle2dynamixel(angle, legNo, jointNo);

    sendCommand(WRITE_WORD,legNo < 3 ?0:1,(int)(legNo*10+jointNo), MOVE_SERWOMOTOR, angle);
    unsigned int error = getResult(legNo < 3 ?0:1);
    if (error == COMM_RXSUCCESS) {
        unsigned int pErr = getPacketError(legNo < 3 ?0:1);
        if (pErr>0){
            printDynamixelError(pErr);
            setTorqueLimit(legNo, jointNo, 1.0);
        }
        return pErr;
    }
    return 0;
}

/// Set reference position value for servomotors, returns error value
unsigned int BoardDynamixel::setPosition(int legNo, const std::vector<double>& angles){
    for (size_t jointNo=0; jointNo<angles.size();jointNo++){
        setPosition(legNo, (int)jointNo, angles[jointNo]);
    }
    return 0;
}

/// angle to dynamixel format
double BoardDynamixel::angle2dynamixel(double angle, int legNo, int jointNo) const{
    angle = angle*_DEG2RAD10;
    angle=-(angle+angle_offset[legNo*3+jointNo]-zero_angle[legNo*3+jointNo])* _DEG2DYNAMIXEL + _OFFSETANGLE;
    return angle;
}

/// Set reference position value for servomotors, returns error value
unsigned int BoardDynamixel::setPosition(const std::vector<double>& angles){
    for (int offset=0;offset<3;offset++){
        for (int i=0;i<6;i+=3){
            int legNo = i+offset;
            std::vector<double> refValues = {angles[legNo*3+0],angles[legNo*3+1],angles[legNo*3+2]};
            setPosition(legNo,refValues);
        }
    }
    return 0;
}

/// Set reference speed value for servomotor, returns error value
unsigned int BoardDynamixel::setSpeed(int legNo, int jointNo, double speed){
    sendCommand(WRITE_WORD, legNo <3 ?0:1, (int)(legNo*10 + jointNo), MOVING_SPEED, speed*1023);
    return 0;
}

/// Set reference speed value for servomotors, returns error value
unsigned int BoardDynamixel::setSpeed(int legNo, const std::vector<double>& speed){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++){
        //object->dxl_write_word(legNo*10+i,MOVING_SPEED,speed[i]*9);
        sendCommand(WRITE_WORD, legNo < 3 ?0:1, (int)(legNo*10+i),MOVING_SPEED, speed[i]*1023);
    }

    return 0;
}

/// Set reference speed value for servomotors, returns error value
unsigned int BoardDynamixel::setSpeed(const std::vector<double>& speed){
    //CDynamixel *object1=&dynamixelMotors[0];
    //CDynamixel *object2=&dynamixelMotors[1];
    int cnt = 0;
    int tmp = 0;
    for(int i=0;i<18;i++){
        if(i<9){
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
            }
            //object1->dxl_write_word(cnt*10+tmp,MOVING_SPEED,speed[i]*9);
            sendCommand(WRITE_WORD, 0, (int)(cnt*10+tmp),MOVING_SPEED, speed[i]*1023);
        }
            else{
            tmp=i%3;
            if(!(i%3) && i!=0){
                cnt++;
                }
            //object2->dxl_write_word(cnt*10+tmp,MOVING_SPEED,speed[i]*9);
            sendCommand(WRITE_WORD, 1, (int)(cnt*10+tmp),MOVING_SPEED, speed[i]*9);
        }
        }
    return 0;
}

/// Set compliance margin [0,254]- dead zone -- for this area the torque is zero, returns error value
unsigned int BoardDynamixel::setComplianceMargin(int legNo, int jointNo, double margin){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    //object->dxl_write_byte(legNo*10 + jointNo, P_CCW_COMPLIANCE_MARGIN, margin);
    //object->dxl_write_byte(legNo*10 + jointNo, P_CW_COMPLIANCE_MARGIN, margin);
    sendCommand(WRITE_BYTE, legNo < 3 ?0:1, (int)(legNo*10 + jointNo), P_CCW_COMPLIANCE_MARGIN, margin);
    sendCommand(WRITE_BYTE, legNo < 3 ?0:1, (int)(legNo*10 + jointNo), P_CW_COMPLIANCE_MARGIN, margin);
    return 0;
}

/// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
unsigned int BoardDynamixel::setComplianceMargin(int legNo, const std::vector<double> margin){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++)
    {
        //object->dxl_write_byte(legNo*10 + i, P_CCW_COMPLIANCE_MARGIN, margin[i]);
        //object->dxl_write_byte(legNo*10 + i, P_CW_COMPLIANCE_MARGIN, margin[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ?0:1, (int)(legNo*10 + i), P_CCW_COMPLIANCE_MARGIN, margin[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ?0:1, (int)(legNo*10 + i), P_CW_COMPLIANCE_MARGIN, margin[i]);
    }
    return 0;
}

/// Set compliance margins [0,254]- dead zone -- for this area the torque is zero, returns error value
unsigned int BoardDynamixel::setComplianceMargin(const std::vector<double> margin){
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //object1->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_MARGIN, margin[i]);
            //object2->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 0, (int)(cnt*10+tmp), P_CCW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 0, (int)(cnt*10+tmp), P_CW_COMPLIANCE_MARGIN, margin[i]);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //object1->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_MARGIN, margin[i]);
            //object2->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 1, (int)(cnt*10+tmp), P_CCW_COMPLIANCE_MARGIN, margin[i]);
            sendCommand(WRITE_BYTE, 1, (int)(cnt*10+tmp), P_CW_COMPLIANCE_MARGIN, margin[i]);
        }

    }
    return 0;
}

/// Set compiance slope [1,254] - the area with the reduced torque, returns error value
unsigned int BoardDynamixel::setComplianceSlope(int legNo, int jointNo, double slope){
    if(slope < 1){
        slope = 1;
    }else if(slope > 254){
        slope = 254;
    }
    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    //pMotor->dxl_write_byte(legNo*10+jointNo, P_CW_COMPLIANCE_SLOPE, slope);
    //pMotor->dxl_write_byte(legNo*10+jointNo, P_CCW_COMPLIANCE_SLOPE, slope);
    sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, (int)(legNo*10+jointNo), P_CW_COMPLIANCE_SLOPE, slope);
    sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, (int)(legNo*10+jointNo), P_CW_COMPLIANCE_SLOPE, slope);
    return 0;
}

/// Set compiance slope [1,254] - the area with the reduced torque, returns error value
unsigned int BoardDynamixel::setComplianceSlope(int legNo, const std::vector<double>& slope){

    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    for(int i = 0; i < 3; i++){     // i -> jointNo
        //pMotor->dxl_write_byte(legNo*10+i, P_CW_COMPLIANCE_SLOPE, slope[i]);
        //pMotor->dxl_write_byte(legNo*10+i, P_CCW_COMPLIANCE_SLOPE, slope[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, (int)(legNo*10+i), P_CW_COMPLIANCE_SLOPE, slope[i]);
        sendCommand(WRITE_BYTE, legNo < 3 ? 0:1, (int)(legNo*10+i), P_CCW_COMPLIANCE_SLOPE, slope[i]);
    }
    return 0;
}

/// Set compiance slope [1,254] - the area with the reduced torque, returns error value
unsigned int BoardDynamixel::setComplianceSlope(const std::vector<double>& slope){
    //CDynamixel *pMotorR = &dynamixelMotors[0];
    //CDynamixel *pMotorL = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //pMotorR->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_SLOPE, slope[i]);
            //pMotorR->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 0, (int)(cnt*10+tmp), P_CW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 0, (int)(cnt*10+tmp), P_CCW_COMPLIANCE_SLOPE, slope[i]);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //pMotorL->dxl_write_byte(cnt*10+tmp, P_CW_COMPLIANCE_SLOPE, slope[i]);
            //pMotorL->dxl_write_byte(cnt*10+tmp, P_CCW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 1, (int)(cnt*10+tmp), P_CW_COMPLIANCE_SLOPE, slope[i]);
            sendCommand(WRITE_BYTE, 1, (int)(cnt*10+tmp), P_CCW_COMPLIANCE_SLOPE, slope[i]);
        }

    }
    return 0;
}

/// Set torque limit torque_limit [0,1] - the torque limit, returns error value
unsigned int BoardDynamixel::setTorqueLimit(int legNo, int jointNo, double torqueLimit){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
            //object->dxl_write_word(legNo*10 + jointNo, SET_TORQUE_LIMIT, torqueLimit*1023);
    sendCommand(WRITE_WORD, legNo < 3 ? 0:1, (int)(legNo*10 + jointNo), SET_TORQUE_LIMIT, torqueLimit*1023);
    return 0;
}

/// Set torque limit torque_limit [0,1] - the torque limit, returns error value
unsigned int BoardDynamixel::setTorqueLimit(int legNo, const std::vector<double>& torqueLimit){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++)
            //object->dxl_write_word(legNo*10 + i, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
            sendCommand(WRITE_WORD, legNo < 3 ? 0:1, (int)(legNo*10 + i), SET_TORQUE_LIMIT, torqueLimit[i]*1023);
    return 0;
}

/// Set torque limit torque_limit [0,1] - the torque limit, returns error value
unsigned int BoardDynamixel::setTorqueLimit(const std::vector<double>& torqueLimit){
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //object1->dxl_write_word(cnt*10+tmp, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
            sendCommand(WRITE_WORD, 0, (int)(cnt*10 + tmp), SET_TORQUE_LIMIT, torqueLimit[i]*1023);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //object2->dxl_write_word(cnt*10+tmp, SET_TORQUE_LIMIT, torqueLimit[i]*1023);
            sendCommand(WRITE_WORD, 1, (int)(cnt*10+tmp), SET_TORQUE_LIMIT, torqueLimit[i]*1023);
        }

    }
    return 0;
}

/// Returns current position of the servomotor, returns error value
unsigned int BoardDynamixel::readPosition(int legNo, int jointNo, double& angle){
    double ang_odt;
    double ang;
    ang_odt = sendCommand(READ_WORD, legNo < 3 ?0:1, (int)(legNo*10 + jointNo), P_PRESENT_POSITION_L, 0);
    ang = ((ang_odt-512)/(-_DEG2DYNAMIXEL))-angle_offset[legNo*3+jointNo]+zero_angle[legNo*3+jointNo];
    angle=ang*_RAD2DEG10;
    if(legNo < 3 && jointNo == 2){
            angle = -angle;
    }
    if(legNo > 2 && jointNo == 1){
            angle = -angle;
    }
    return 0;
}

/// Returns current position of the servomotors, returns error value
unsigned int BoardDynamixel::readPosition(int legNo, std::vector<double>& angles){
    angles.resize(3);
    for(int i = 0; i<3; i++){
        double angleTmp;
        readPosition(legNo, i, angleTmp);
        angles[i]=angleTmp;
    }
    return 0;
}

/// Returns current position of the servomotor, returns error value
unsigned int BoardDynamixel::readPosition(std::vector<double>& angles){
    angles.resize(18);
    for (int legNo=0;legNo<3;legNo++){
        std::vector<double> angLeft, angRight;
        std::thread leftSide(static_cast<unsigned int (BoardDynamixel::*)(int, std::vector<double>&)>(&BoardDynamixel::readPosition),this, legNo, std::ref(angLeft));
        std::thread rightSide(static_cast<unsigned int (BoardDynamixel::*)(int, std::vector<double>&)>(&BoardDynamixel::readPosition),this, legNo+3, std::ref(angRight));
        leftSide.join();
        rightSide.join();
        for (int i=0;i<3;i++){
            angles[legNo*3+i]=angLeft[i];
            angles[(legNo+3)*3+i] = angRight[i];
        }
    }
    return 0;
}

/// Returns contact force from 1-axis force sensor
unsigned int BoardDynamixel::readForce(int legNo, double& contactForce){
    std::cout << legNo << " " << contactForce << "\n";
    throw std::runtime_error("Board Error: readForce method is not implemented\n");
    return 0;
}

/// Returns contact force from 1-axis force sensor
unsigned int BoardDynamixel::readForce(const std::vector<double>& contactForce){
    std::cout << contactForce.size() << "\n";
    throw std::runtime_error("Board Error: readForce method is not implemented\n");
    return 0;
}

/// Returns contact force from 3-axis torque/force sensor
unsigned int BoardDynamixel::readTorqueForce(int legNo, walkers::TorqueForce& valueTF){
    //std::cout << legNo << " " << valueTF.force.vector() << "\n";
    throw std::runtime_error("Board Error: readTorqueForce method is not implemented\n");
    return 0;
}

/// Returns contact force from 3-axis torque/force sensor
unsigned int BoardDynamixel::readTorqueForce(const std::vector<double>& valueTF){
    std::cout << valueTF.size() << "\n";
    throw std::runtime_error("Board Error: readTorqueForce method is not implemented\n");
    return 0;
}

/// Returns contact or from microswitch
bool BoardDynamixel::readContact(int legNo){
    char buffer;
    lseek(fileDescriptorsGPIO[legNo], 0, SEEK_SET);
    ssize_t size = read( fileDescriptorsGPIO[legNo], &buffer, 1 );
    size+=1;

    if ( buffer == '0')
        return true;
    else
        return false;
}

/// Returns contact or from microswitches
void BoardDynamixel::readContacts(std::vector<bool>& contacts){
    contacts.resize(6);
    for (int i=0;i<6;i++){
        contacts[i]=readContact(i);
    }
}

/// Returns current from servo
unsigned int BoardDynamixel::readCurrent(int legNo, int jointNo, double& servoCurrent){
    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    //servoCurrent = pMotor->dxl_read_word(legNo*10 + jointNo, PRESENT_VOLTAGE);
    servoCurrent = sendCommand(READ_WORD, legNo < 3 ? 0:1, (int)(legNo*10 + jointNo), PRESENT_VOLTAGE, 0);
    servoCurrent = servoCurrent/10;

    return 0;
}

/// Returns current from servo
unsigned int BoardDynamixel::readCurrent(int legNo, std::vector<double>& servoCurrent){
    //CDynamixel *pMotor = &dynamixelMotors[ legNo < 3 ?0:1 ];
    for(int i=0; i<3; i++){
        //servoCurrent.push_back( pMotor->dxl_read_word(legNo*10 + i, PRESENT_VOLTAGE) );
        servoCurrent.push_back(sendCommand(READ_WORD, legNo < 3 ? 0:1, (int)(legNo*10 + i), PRESENT_VOLTAGE, 0));
        servoCurrent[i] = servoCurrent[i] / 10;
    }
    return 0;
}

/// Returns current from servo
unsigned int BoardDynamixel::readCurrent( std::vector<double>& servoCurrent){
    //CDynamixel *pMotorR = &dynamixelMotors[0];
    //CDynamixel *pMotorL = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            //servoCurrent.push_back( pMotorR->dxl_read_word( cnt*10+tmp, PRESENT_VOLTAGE ) );
            servoCurrent.push_back(sendCommand(READ_WORD, 0, (int)(cnt*10+tmp), PRESENT_VOLTAGE, 0));
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            //servoCurrent.push_back( pMotorL->dxl_read_word( cnt*10+tmp, PRESENT_VOLTAGE ) );
            servoCurrent.push_back(sendCommand(READ_WORD, 1, (int)(cnt*10+tmp), PRESENT_VOLTAGE, 0));
        }
    servoCurrent[i] = servoCurrent[i]/10;
    }
    return 0;
}

/// Returns torque/load from servo
unsigned int BoardDynamixel::readTorque(int legNo, int jointNo, double& servoTorque){
    //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    servoTorque = sendCommand(READ_WORD, legNo < 3 ?0:1, (int)(legNo*10 + jointNo), TORQUE, 0) * sendCommand(READ_WORD, legNo < 3 ? 0:1, (int)(legNo*10+jointNo), GET_MAX_TORQUE, 0)/1024*28.3;
    //servoTorque = object->dxl_read_word(legNo*10 + jointNo, TORQUE)*object->dxl_read_word(legNo*10+jointNo, GET_MAX_TORQUE)/1024*28.3;
     return 0;
}

/// Returns torque/load from servo
unsigned int BoardDynamixel::readTorque(int legNo,std::vector<double>& servoTorque){
   //CDynamixel *object = &dynamixelMotors[legNo < 3 ?0:1];
    for(int i=0;i<3;i++)
    {
       servoTorque.push_back(sendCommand(READ_WORD, legNo < 3 ?0:1, (int)(legNo*10 + i), TORQUE, 0)*sendCommand(READ_WORD, legNo < 3 ? 0:1, (int)(legNo*10+i), GET_MAX_TORQUE, 0)/1024*28.3);
       //servoTorque.push_back(object->dxl_read_word(legNo*10 + i, TORQUE)*object->dxl_read_word(legNo*10+i, GET_MAX_TORQUE)/1024*28.3);
    }
    return 0;
}

/// Returns torque/load from servo
unsigned int BoardDynamixel::readTorque(std::vector<double>& servoTorque){
    //CDynamixel *object1 = &dynamixelMotors[0];
    //CDynamixel *object2 = &dynamixelMotors[1];
    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
           //servoTorque.push_back(object1->dxl_read_word(cnt*10+tmp, TORQUE)*object1->dxl_read_word(cnt*10+tmp, GET_MAX_TORQUE)/1024*28.3);
           servoTorque.push_back(sendCommand(READ_WORD, 0, (int)(cnt*10+tmp), TORQUE, 0)*sendCommand(READ_WORD, 0, (int)(cnt*10+tmp), GET_MAX_TORQUE, 0)/1024*28.3);
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            servoTorque.push_back(sendCommand(READ_WORD, 1, (int)(cnt*10+tmp), TORQUE, 0)*sendCommand(READ_WORD, 1, (int)(cnt*10+tmp), GET_MAX_TORQUE, 0)/1024*28.3);
            //servoTorque.push_back(object2->dxl_read_word(cnt*10+tmp, TORQUE)*object2->dxl_read_word(cnt*10+tmp, GET_MAX_TORQUE)/1024*28.3);
        }
    }
    return 0;
}

/// Set servo Offset
void BoardDynamixel::setOffset(int legNo, int jointNo, double offset){
    double localOffset;

    localOffset = offset;
    localOffset = localOffset * (180/M_PI);

    angle_offset[legNo*10+jointNo] += (int)localOffset ;

}

/// Set servo Offset
void BoardDynamixel::setOffset(int legNo, const std::vector<double> offset){
    vector <double> localOffset;
    for(int i=0; i<3; i++){
       localOffset.push_back( offset[i] );
       localOffset[i] = localOffset[i] * (180/M_PI);
    }

    for(int i=0; i<3; i++){
         angle_offset[legNo*10+i] += (int)localOffset[i];
    }
}

/// Set servo Offset
void BoardDynamixel::setOffset(const std::vector<double> offset){
    vector <double> localOffset;
    for(int i=0; i<18; i++){
       localOffset.push_back( offset[i] );
       localOffset[i] = localOffset[i] * (180/M_PI);
    }

    int tmp = 0;
    int cnt = 0;
    for(int i = 0; i < 18; i++){
        if(i < 9){  //Right side of Mesor
            tmp = i%3;
            if(!(i%3) && i != 0){
                cnt++;
            }
            angle_offset[cnt*10+tmp] += (int)localOffset[i];
        }else{      //Left side of Mesor
            tmp = i%3;
            if(!i%3){
                cnt++;
            }
            angle_offset[cnt*10+tmp] += (int)localOffset[i];
        }
    }
}

/// Board configuration -- set default value
void BoardDynamixel::setDefault(void){ }

controller::Board* controller::createBoardDynamixel(void) {
    boardDynamixel.reset(new BoardDynamixel());
    return boardDynamixel.get();
}
