// Code : UTF - 8
#ifndef ROPO_SENSOR_SERIAL_CORE_HPP
#define ROPO_SENSOR_SERIAL_CORE_HPP

#include "Api.hpp"

namespace RopoSensor{

	class SerialCore{
		protected:
			int SamplingDelay;
			bool Reading = false;

			SystemSerial Receive;
			SystemTaskPtr BackgroundTask;

			virtual void Update() = 0;

		public:
			bool IsReading(){return Reading;}				//判断是否正在接收信号？

			static void BackgroundFunction(void *param){
				if(param == nullptr) return;
				SerialCore *P = static_cast<SerialCore *>(param);
				while(1){
					if(P->Receive.PeekByte() != -1) P->Reading = true;
					else P->Reading = false;
					P->Update();
					SystemDelay(P->SamplingDelay);
				}
			}

			SerialCore(SerialID _ID, std::int32_t _Baudrate, int _SamplingDelay = 5):Receive(_ID,_Baudrate),SamplingDelay(_SamplingDelay){
				BackgroundTask = NewSystemTask(BackgroundFunction,this);
			}

			~SerialCore(){delete BackgroundTask;}
	};

}

#endif //ROPO_SENSOR_SERIAL_CORE_HPP
