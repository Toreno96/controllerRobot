/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 3 (Baudrate : 1000000)
//

#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <chrono>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

using namespace std;


// Control table address
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_CURRENT           102
#define ADDR_PRO_PRESENT_CURRENT	126
#define ADDR_PRO_GOAL_VELOCITY		104
#define ADDR_PRO_PRESENT_VELOCITY	128
#define ADDR_PRO_GOAL_EXT_POS   	116
#define ADDR_PRO_GOAL_CURR_BASE_POS 	116
#define ADDR_PRO_GOAL_PWM		100
#define ADDR_PRO_PRESENT_PWM		124 
#define ADDR_PRO_BAUDRATE  		8	
#define ADDR_OPERATING_MODE		11	// 0: Current control, 1: Velocity Control, 3: Position Control,
						// 4:  Ext. Pos. Control, 5: Current base pos control, 16: PWM Control.

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

  //newt = oldt;
// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        3000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_GOAL_POSITION_VALUE      	0            // Dynamixel will rotate between this value
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 1;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result000
  uint8_t dxl_error = 0;                          // Dynamixel error



int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void torque_off(void)
{
  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
}

void torque_on(void)
{
  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->printTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->printRxPacketError(dxl_error);
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }
}

int main()
{


  int dxl_goal_position = {DXL_GOAL_POSITION_VALUE};         // Goal position
  int dxl_operating_mode = 3; 			//Operating mode
  int dxl_goal_current = 0;
  int dxl_goal_velocity = 0;
  int dxl_goal_ext_pos = 0;
  int dxl_goal_curr_base_pos = 0;
  int dxl_goal_pwm = 0;
  int baudrate = 1;
  int set_baudrate = 1;

  int32_t dxl_present_position = 0;               // Present position
  int16_t dxl_present_current = 0;		  // Present current
  int32_t dxl_present_velocity = 0;		  // Present velocity
  int32_t dxl_present_ext_pos = 0;		  // Present extended position
  int32_t dxl_present_pwm = 0;			  // Present PWM  
  uint16_t length = 1;

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
/*torque_off();
  //USTAWIANIE BAUDRATE
  
  cout<<"WYBIERZ BUADRATE:"<<endl<<"0: 9600[bps]"<<endl<<"1: 57600[bps]"<<endl;
  cout<<"2: 115200[bps]"<<endl<<"3: 1M[bps]"<<endl<<"4: 2M[bps]"<<endl;
  cout<<"5: 3M[bps]"<<endl<<"6: 4M[bps]"<<endl<<endl;
  cin>>baudrate;

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUDRATE, baudrate, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }
  
  if (baudrate == 0) set_baudrate=9600;
  if (baudrate == 1) set_baudrate=57600;
  if (baudrate == 2) set_baudrate=115200;
  if (baudrate == 3) set_baudrate=1000000;
  if (baudrate == 4) set_baudrate=2000000;
  if (baudrate == 5) set_baudrate=3000000;
  if (baudrate == 6) set_baudrate=4000000;

  // Set port baudrate
  if (portHandler->setBaudRate(set_baudrate))
  {
    printf("Succeeded to change the baudrate to %03d!\n", set_baudrate);
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }*/

  while(1)
  {
    torque_off();
    printf("\n\n Press any key to continue! (or press ESC to quit!)\n\n");
    if (getch() == ESC_ASCII_VALUE)
      break;
    cout<<"WYBIERZ TRYB PRACY:"<<endl<<"0: Current control"<<endl<<"1: Velocity Control"<<endl;
    cout<<"3: Position Control"<<endl<<"4: Ext. Pos. Control"<<endl<<"5: Current base pos control"<<endl;
    cout<<"16: PWM Control"<<endl<<endl;
    
    cin>>dxl_operating_mode;
    
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, dxl_operating_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }

    torque_on();

    if (dxl_operating_mode == 0)
    {
	    cout<<"WPROWADZ ZADANY PRAD (ZAKRES -1193 do 1193): ";
	    cin>>dxl_goal_current;
	    cout<<endl;
            // Write goal current
	    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, dxl_goal_current, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }
	    do
	    {
	      // Read present current
	      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_CURRENT, (uint16_t*)&dxl_present_current, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
		packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
		packetHandler->printRxPacketError(dxl_error);
	      }

	      printf("[ID:%03d] GoalCurr:%03d  PresCurr:%03d\n Press ESC to STOP\n\n", DXL_ID, dxl_goal_current, dxl_present_current);

	    }while(getch() != ESC_ASCII_VALUE);
            
	    torque_off();
    }


    if (dxl_operating_mode == 1)
    {
	    cout<<"WPROWADZ ZADANA PREDKOSC (ZAKRES -480 do 480): ";
	    cin>>dxl_goal_velocity;
	    cout<<endl;
            // Write goal velocity
	    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_VELOCITY, dxl_goal_velocity, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }
	    do
	    {
	      // Read present current
	      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
		packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
		packetHandler->printRxPacketError(dxl_error);
	      }

	      printf("[ID:%03d] GoalVelo:%03d  PresVelo:%03d\n Press ESC to STOP", DXL_ID, dxl_goal_velocity, dxl_present_velocity);

	    }while(getch() != ESC_ASCII_VALUE);
            
	    torque_off();
    }

    if (dxl_operating_mode == 3)
    {
	    cout<<"WPROWADZ POZYCJE DOCELOWA (ZAKRES 0 - 4095): ";
	    cin>>dxl_goal_position;
	    cout<<endl;

	    // Write goal position
	    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }

	    do
	    {
	      // Read present position
	      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
		packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
		packetHandler->printRxPacketError(dxl_error);
	      }

	      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position, dxl_present_position);

	    }while((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

	    torque_off();
    }

    if (dxl_operating_mode == 4)
    {
	    cout<<"WPROWADZ ZADANA ILOSC OBROTOW (EXTENDED) (ZAKRES -256 do 256): ";
	    cin>>dxl_goal_ext_pos;
	    dxl_goal_ext_pos*=4095;
	    cout<<endl;
            // Write goal ext posistion
	    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_EXT_POS, dxl_goal_ext_pos, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }

	    do
	    {
	      // Read present position/home/robot/Dokumenty/Projekt_serwo/src/read_write.cpp:241:8: error: ‘buadrate’ was not declared in this scope

	      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
		packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
		packetHandler->printRxPacketError(dxl_error);
	      }

	      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_ext_pos, dxl_present_position);

	    }while((abs(dxl_goal_ext_pos- dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

	    torque_off();
    }

    if (dxl_operating_mode == 5)
    {
	    cout<<"WPROWADZ ZADANY PRAD (CURRENT BASE) (ZAKRES 0 do 1193): ";
	    cin>>dxl_goal_current;
	    cout<<endl;
            // Write goal current
	    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURRENT, dxl_goal_current, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }	    

	    cout<<"WPROWADZ ZADANA POZYCJE (CURRENT BASE) (ZAKRES 0 - 4095): ";
	    cin>>dxl_goal_curr_base_pos;
	    cout<<endl;
            // Write goal ext posistion
	    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_CURR_BASE_POS, dxl_goal_curr_base_pos, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }

	    do
	    {
	      // Read present position
	      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
		packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
		packetHandler->printRxPacketError(dxl_error);
	      }

	      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_curr_base_pos, dxl_present_position);

	    }while((abs(dxl_goal_curr_base_pos- dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

	    torque_off();
    }

    if (dxl_operating_mode == 16)
    {
	    cout<<"WPROWADZ ZADANA WARTOSC WYPELNIENIA PWM (ZAKRES -885 - 885): ";
	    cin>>dxl_goal_pwm;
	    cout<<endl;
            // Write goal PWM
	    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_PWM, dxl_goal_pwm, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }

	    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	    for(int i=0; i<100000;i++)
	    //do
	    {
	      // Read present PWM
	      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_PWM, (uint32_t*)&dxl_present_pwm, &dxl_error);
	      if (dxl_comm_result != COMM_SUCCESS)
	      {
		packetHandler->printTxRxResult(dxl_comm_result);
	      }
	      else if (dxl_error != 0)
	      {
		packetHandler->printRxPacketError(dxl_error);
	      }

	      //printf("[ID:%03d] GoalPWM:%03d  PresPWM:%03d\n", DXL_ID, dxl_goal_pwm, dxl_present_pwm);
	      
              //if (getch() == ESC_ASCII_VALUE) break;
	    }//while(1);

	    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now(); 
	    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;

	    torque_off();
    }
  }

  torque_off();

  //USTAWIANIE BUADRATE NA DEAFAULT
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_BAUDRATE, 1, &dxl_error);
	    if (dxl_comm_result != COMM_SUCCESS)
	    {
	      packetHandler->printTxRxResult(dxl_comm_result);
	    }
	    else if (dxl_error != 0)
	    {
	      packetHandler->printRxPacketError(dxl_error);
	    }

  // Close port
  portHandler->closePort();

  return 0;
}

