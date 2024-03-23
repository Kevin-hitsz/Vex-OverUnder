// Code : UTF - 8
#ifndef ROPO_SENSOR_ENCODING_DISK_HPP
#define ROPO_SENSOR_ENCODING_DISK_HPP

#include "RopoMath/Misc.hpp"
#include "SerialCore.hpp"

namespace RopoSensor{
	class EncodingDisk : public SerialCore{
		protected:
			float PosX;		
			float PosY;
			float Angle[3];
			float W_Z;
			float Encoding_in_Car_X,Encoding_in_Car_Y,Encoding_in_Car_Angle;
			SystemSerial Send;
			virtual void Update(){
				static uint8_t ReceiveChar;
				static union{
					uint8_t Data[24];
					float ActVal[6];
				}Posture;
				static uint8_t Count = 0;
				ReceiveChar = Receive.ReadByte();
				switch(Count){
					case 0:
						if(ReceiveChar==0x0d)
							Count++;
						else
							Count = 0;
						break;
					case 1:
						if(ReceiveChar==0x0a){
							// i = 0;
							Receive.Read(Posture.Data,24);
							Count++;
						}
						else if(ReceiveChar==0x0d);
						else
							Count = 0;
						break;
					// case 2:					
						// Posture.data[i] = ReceiveChar;
						// i++;
						// if(i>=24){
						// 	i = 0;
						// 	Count++;
						// }
						// break;
					case 2:
						if(ReceiveChar==0x0a){
							Angle[2] = Posture.ActVal[0];
							Angle[0] = Posture.ActVal[1];
							Angle[1] = Posture.ActVal[2];
							PosX = Posture.ActVal[3];
							PosY = Posture.ActVal[4];
							W_Z  = Posture.ActVal[5];
						}
						Count = 0;
						break;
					// case 3:
						// 	if(ReceiveChar==0x0d){
								
						// 	}
						// 	Count = 0;
						// 	break;
					default:
						Count = 0;
						break;
				}
			}
		public:
			//x为机体正前方，y为机体正左
			//获得码盘相对于机体零点机体x方向的坐标pos_x
			float GetEncodingPosX0(){
				return  PosY * RopoMath::Cos(Encoding_in_Car_Angle) + PosX * RopoMath::Sin(Encoding_in_Car_Angle) + Encoding_in_Car_X;
			}		
			//获得码盘相对于机体零点机体y方向的坐标pos_y
			float GetEncodingPosY0(){
				return -PosX * RopoMath::Cos(Encoding_in_Car_Angle) + PosY * RopoMath::Sin(Encoding_in_Car_Angle) + Encoding_in_Car_Y;
			}			
			//获得机体相对于零点x方向的坐标pos_x
			float GetPosX(){
				return  GetEncodingPosX0() - Encoding_in_Car_X * RopoMath::Cos(Angle[2]) + Encoding_in_Car_Y * RopoMath::Cos(Angle[2]);
			}		
			//获得机体相对于零点y方向的坐标pos_y
			float GetPosY(){
				return  GetEncodingPosY0() - Encoding_in_Car_Y * RopoMath::Cos(Angle[2]) - Encoding_in_Car_X * RopoMath::Cos(Angle[2]);
			}		
			//获得机体偏离Tag所示轴向的角度Tag angle 
				//Tag: 0_x 1_y 2_z
			float GetAngle(int Tag){return Angle[Tag];}
			//设置零点位置
			void SetZero(){
				uint8_t Message[10] = "ACT0\n";
				Send.Write(Message,5);
			}			
			EncodingDisk(SerialID Receive_ID, std::int32_t Receive_Baudrate,SerialID Send_ID,std::int32_t Send_Baudrate)
				:Send(Send_ID,Send_Baudrate),SerialCore(Receive_ID,Receive_Baudrate),Encoding_in_Car_X(0),Encoding_in_Car_Y(0),Encoding_in_Car_Angle(0){}
			EncodingDisk(SerialID Receive_ID, std::int32_t Receive_Baudrate,SerialID Send_ID,std::int32_t Send_Baudrate,int _SamplingDelay)
				:Send(Send_ID,Send_Baudrate),SerialCore(Receive_ID,Receive_Baudrate,_SamplingDelay),Encoding_in_Car_X(0),Encoding_in_Car_Y(0),Encoding_in_Car_Angle(0){}
			EncodingDisk(SerialID Receive_ID, std::int32_t Receive_Baudrate,SerialID Send_ID,std::int32_t Send_Baudrate,int _SamplingDelay,float _Encoding_in_Car_X,float _Encoding_in_Car_Y)
				:Send(Send_ID,Send_Baudrate),SerialCore(Receive_ID,Receive_Baudrate,_SamplingDelay),Encoding_in_Car_X(_Encoding_in_Car_X),Encoding_in_Car_Y(_Encoding_in_Car_Y),Encoding_in_Car_Angle(0){}	
			EncodingDisk(SerialID Receive_ID, std::int32_t Receive_Baudrate,SerialID Send_ID,std::int32_t Send_Baudrate,int _SamplingDelay,float _Encoding_in_Car_X,float _Encoding_in_Car_Y,float _Encoding_in_Car_Angle)
				:Send(Send_ID,Send_Baudrate),SerialCore(Receive_ID,Receive_Baudrate,_SamplingDelay),Encoding_in_Car_X(_Encoding_in_Car_X),Encoding_in_Car_Y(_Encoding_in_Car_Y),Encoding_in_Car_Angle(_Encoding_in_Car_Angle){}
			~EncodingDisk(){}
	};
}

#endif //ROPO_SENSOR_ENCODING_DISK_HPP