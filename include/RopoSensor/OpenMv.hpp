#ifndef ROPO_OPEN_MV_H
#define ROPO_OPEN_MV_H

#include "RopoMath/Vector.hpp"
#include "SerialCore.hpp"
#include <cstdio>
#include <string.h>
#include <iostream>

namespace RopoSensor{
	class OpenMv : public RopoSensor::SerialCore{
		public:
			double Ball_Data[6][2] = {0};
			int ball_count;
			int x_Dis,x_Roa,x_get;
			int flag_see;
			
            union UnionBuffer{
				char Message[50];
				uint8_t RawMessage[50];
			}ReceiveBuffer;
			virtual void Update(){
				int8_t message = Receive.ReadByte();
		        if(message == 102 ){
					Receive.Read(ReceiveBuffer.RawMessage,50);
					flag_see = ReceiveBuffer.Message[0]-48;
					ball_count = ReceiveBuffer.Message[1]-48;
					std::printf("Flag:%d , Number:%d \n",flag_see,ball_count);
					for (int i=0; i<ball_count; i++) {
						x_Dis = (int(ReceiveBuffer.Message[2+i*8]-48)*1000+int(ReceiveBuffer.Message[3+i*8]-48)*100+int(ReceiveBuffer.Message[4+i*8]-48)*10+int(ReceiveBuffer.Message[5+i*8]-48))/2;
						x_Roa = (int(ReceiveBuffer.Message[6+i*8]-48)*1000+int(ReceiveBuffer.Message[7+i*8]-48)*100+int(ReceiveBuffer.Message[8+i*8]-48)*10+int(ReceiveBuffer.Message[9+i*8]-48))/10 - 150;
						Ball_Data[i][0] = double(x_Dis) / 100.0;
						Ball_Data[i][1] = double(x_Roa) / 1.0;
						if (Ball_Data[i][0]>2) {
							Ball_Data[i][0] = 0;
							flag_see = 0;
						}
						if (Ball_Data[i][1] > 80 || Ball_Data[i][1] < -80) {
							Ball_Data[i][1] = 0;
							flag_see = 0;
						}
					}
					std::printf("OpenMv_output:%d , %lf , %lf \n",ball_count,Ball_Data[0][0],Ball_Data[0][1]);
		        }
			}
			
		public:
			OpenMv(int _Port,int _Baudrate):SerialCore(_Port,_Baudrate){
				
		        memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));	
		    }
			OpenMv(int _Port,int _Baudrate,int _SamplingDelay):SerialCore(_Port,_Baudrate,_SamplingDelay){
				
		        memset(ReceiveBuffer.Message,0,sizeof(ReceiveBuffer));
		    }
			~OpenMv(){};

			int GetBall_Count(){
				return ball_count;
			}

			int If_See(){
				return flag_see;
			}
			double Get_Ball_Dis(){
				if (ball_count == 1) {
					return Ball_Data[0][0];
				}
				else{
					double Center_Dis;
					for (int i=1; i<ball_count; i++) {
						Center_Dis = fabs(Ball_Data[i-1][1]) < fabs(Ball_Data[i][1]) ? Ball_Data[i-1][0] : Ball_Data[i][0];
					}
					return Center_Dis;
				}
			}
			double Get_Ball_Deg(){
				if (ball_count == 1) {
					return Ball_Data[0][1];
				}
				else{
					double Center_Deg;
					for (int i=1; i<ball_count; i++) {
						Center_Deg = fabs(Ball_Data[i-1][1]) < fabs(Ball_Data[i][1]) ? Ball_Data[i-1][1] : Ball_Data[i][1];
					}
					return Center_Deg;
				}
			}//偏左为负，偏右为正                   
			
    };

}

#endif //ROPO_OPEN_MV_H