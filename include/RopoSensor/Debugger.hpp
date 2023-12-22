// Code : UTF - 8
#ifndef ROPO_SENSOR_DEBUGGER_HPP
#define ROPO_SENSOR_DEBUGGER_HPP

#include "SerialCore.hpp"
#include <string.h>

namespace RopoSensor{
	class Debugger : public SerialCore{
		public:
			union UnionBuffer{
				char Message[100];
				uint8_t RawMessage[100];
			}SendBuffer,ReceiveBuffer;

			virtual void Update(){
				if(SendBuffer.Message[0]=='\0')return;
				if(Reading){
					static int cnt = 0;
					// int ReceiveLen = Receive->flush();
					memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));
					uint8_t Res; 
					while(Receive.PeekByte() != -1){
						Res = Receive.ReadByte();				
						if((char)Res == '\n'||(char)Res == '\0'||(char)Res == ' ')cnt = 0;
						else ReceiveBuffer.RawMessage[cnt++] = Res;
					}
					return;
				}
				int Len = strlen(SendBuffer.Message);
				Receive.Write(SendBuffer.RawMessage,Len);
				memset(SendBuffer.Message,0,sizeof(SendBuffer));
			}

			Debugger(int _ID,int _Baudrate):SerialCore(_ID,_Baudrate){
				memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));
				memset(SendBuffer.Message,0,sizeof(SendBuffer));
			}
			Debugger(int _ID,int _Baudrate,int _SamplingDelay):SerialCore(_ID,_Baudrate,_SamplingDelay){
				memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));
				memset(SendBuffer.Message,0,sizeof(SendBuffer));
			}
			~Debugger(){}
	};
}
#endif //ROPO_SENSOR_DEBUGGER_HPP
