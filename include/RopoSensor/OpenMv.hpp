#ifndef ROPO_OPEN_MV_H
#define ROPO_OPEN_MV_H

#include "SerialCore.hpp"
#include <string.h>

namespace RopoSensor{
	class OpenMv : public RopoSensor::SerialCore{
		protected:
			double Distance,Theta;
			bool   IfExists;
			
            union UnionBuffer{
				char Message[9];
				uint8_t RawMessage[9];
			}ReceiveBuffer;
			virtual void Update(){
				int8_t message = Receive.ReadByte();
		        if(message == 0x66){
		            // static int cnt = 0;
		            // cnt ++;
		            Receive.Read(ReceiveBuffer.RawMessage,9);

		            Distance =  ((ReceiveBuffer.Message[0]-'0')*1000 + (ReceiveBuffer.Message[1]-'0') * 100 + (ReceiveBuffer.Message[2]-'0') * 10 + (ReceiveBuffer.Message[3]-'0')) / 200.0;
		            Theta    =  (((ReceiveBuffer.Message[4]-'0')*1000 + (ReceiveBuffer.Message[5]-'0') * 100 + (ReceiveBuffer.Message[6]-'0') * 10 + (ReceiveBuffer.Message[7]-'0')) / 10.0 - 150);
		        	IfExists =  ((int)ReceiveBuffer.Message[8]-'0');
				}
			}
			
		public:
			
			OpenMv(int _Port,int _Baudrate):SerialCore(_Port,_Baudrate){
				Distance = 1.0;
				Theta    = 0.0;
				IfExists = false;
		        memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));	
		    }
			OpenMv(int _Port,int _Baudrate,int _SamplingDelay):SerialCore(_Port,_Baudrate,_SamplingDelay){
				Distance = 1.0;
				Theta    = 0.0;
				IfExists = false;
		        memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));
		    }
			~OpenMv(){};                      
			double GetDistance(){
				return Distance;
			}
			double GetTheta(){
				return Theta;
			}
			bool GetExists(){
				return IfExists;
			}

    };

}

#endif