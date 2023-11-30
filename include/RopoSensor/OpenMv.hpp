#ifndef ROPO_OPEN_MV_H
#define ROPO_OPEN_MV_H

#include "SerialCore.hpp"
#include <string.h>

namespace RopoSensor{
	class OpenMv : public RopoSensor::SerialCore{
		protected:
			int flag_blue,x_blue,flag_red,x_red,x_get;
			
            union UnionBuffer{
				char Message[8];
				uint8_t RawMessage[8];
			}ReceiveBuffer;
			virtual void Update(){
				int8_t message = Receive.ReadByte();
		        if(message == 0x66){
		            // static int cnt = 0;
		            // cnt ++;
		            Receive.Read(ReceiveBuffer.RawMessage,8);
		            flag_blue = (int)ReceiveBuffer.Message[0]-'0';
		            x_blue = ((int)ReceiveBuffer.Message[1]-'0')*100+((int)ReceiveBuffer.Message[2]-'0')*10+((int)ReceiveBuffer.Message[3]-'0')-120;
		            
		            flag_red = (int)ReceiveBuffer.Message[4]-'0';
		            x_red = ((int)ReceiveBuffer.Message[5]-'0')*100+((int)ReceiveBuffer.Message[6]-'0')*10+((int)ReceiveBuffer.Message[7]-'0')-120;
		            
		            // if(flag_red == 1){
		            //     x_get = x_red;
		            // }
		            // if(flag_blue == 1){
		            //     x_get = x_blue;
		            // }
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

			bool IfBlueExists(){
				return flag_blue;
			}
			bool IfRedExists(){
				return flag_red;
			}
			bool IfExists(){
				if(flag_blue == 1 || flag_red == 1){
					return 1;
				}
				else return 0;
			}

            double GetOpenMvBlueX(){
		        return (double)x_blue;
		    };						//x的范围为-120到120，当x为120度的时候，目标与车相对偏角大概在20度左右，
                                    //如果目标在左侧，则x为负
                                    //如果目标在右侧，则x为正；
                                    //并且x的数值不连续，间隔为10跳变，x可以为0;如-20，-10,0,10,20 
            double GetOpenMvRedX(){
		        return (double)x_red;
		    }						//同上

			double GetOpenMvX(){
				if(flag_red == 1){
					x_get = x_red;
				}
				if(flag_blue == 1){
					x_get = x_blue;
				}
		        return (double)x_get;
		    }						//得到需要对准的的x，
						  			//x的范围为-120到120，当x为120度的时候，目标与车相对偏角大概在20度左右，
                          			//如果目标在左侧，则x为负
                          			//如果目标在右侧，则x为正；                      
			

    };

}

#endif