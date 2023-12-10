// Code : UTF - 8
#ifndef ROPO_SENSOR_API_HPP
#define ROPO_SENSOR_API_HPP

#include <cstdint>
#include "../pros/serial.hpp"
#include "../pros/rtos.hpp"

namespace RopoSensor{
	
#ifdef _PROS_SERIAL_HPP_ // Check pros::Serial exists

	typedef std::uint8_t SerialID;

	class SystemSerial {
		private:
			pros::Serial Serial;
		public:
			SystemSerial(SerialID _ID, std::int32_t _Baudrate):Serial(_ID,_Baudrate){}
			~SystemSerial(){}
			virtual void SetBaudrate(std::int32_t Baudrate) const {
				Serial.set_baudrate(Baudrate);
			}
			virtual void Flush() const {
				Serial.flush();
			}
			virtual std::int32_t PeekByte() const {
				return Serial.peek_byte();
			}
			virtual std::int32_t ReadByte() const {
				return Serial.read_byte();
			}
			virtual void WriteByte(std::uint8_t Buffer) const {
				Serial.write_byte(Buffer);
			}
			virtual void Read(std::uint8_t *Buffer,std::int32_t Length) const {
				Serial.read(Buffer,Length);
			}
			virtual void Write(std::uint8_t *Buffer,std::int32_t Length) const {
				Serial.write(Buffer,Length);
			}	
	};

#else 

	typedef const char* SerialID;

	class SystemSerial{
		public:
			SystemSerial(SerialID _ID, std::int32_t _Baudrate){}
			~SystemSerial(){}
			virtual void SetBaudrate(std::int32_t Baudrate) const = 0;
			virtual void Flush() const = 0;
			virtual std::int32_t PeekByte() const = 0;
			virtual std::int32_t ReadByte() const = 0;
			virtual void WriteByte(std::uint8_t Buffer) const = 0;
			virtual void Read(std::uint8_t *Buffer,std::int32_t Length) const = 0;
			virtual void Write(std::uint8_t *Buffer,std::int32_t Length) const = 0;		
	};

#endif 

#ifdef _PROS_RTOS_HPP_ // Check pros::Task & pros::delay exists

	typedef pros::Task* SystemTaskPtr;

	inline SystemTaskPtr NewSystemTask(void (*CallbackFunction)(void *),void *Parameter)
	{
		return new pros::Task(CallbackFunction,Parameter);
	}

	inline void SystemDelay(const std::uint32_t Milliseconds){
		pros::delay(Milliseconds);
	}

#else 

	typedef void* SystemTaskPtr;
	inline SystemTaskPtr NewSystemTask(void (*CallbackFunction)(void *),void *Parameter){return nullptr;}
	inline void SystemDelay(const std::uint32_t Milliseconds){}

#endif 

}

#endif //ROPO_SENSOR_API_HPP
