// Code : UTF - 8
#ifndef ROPO_MATH_SMART_DEGREE_HPP
#define ROPO_MATH_SMART_DEGREE_HPP

namespace RopoMath{

	template<class T>class SmartDegree{
		private:
			T Value;
			T FixValue(T Input){
				while(Input < -180.0)Input += 360.0;
				while(Input >= 180.0)Input -= 360.0;
				return Input;
			}
		public:
			SmartDegree(T _Value = 0.0){Value = FixValue(_Value);}
			~SmartDegree(){}
			operator T(){
				return Value = FixValue(Value);
			}
			SmartDegree<T> operator = (const T _Value){
				Value = FixValue(_Value);
				return (*this);
			}
			SmartDegree<T> operator + (const T _Value)const{
				return SmartDegree<T>(Value + _Value);
			}
			SmartDegree<T> operator - (const T _Value)const{
				return SmartDegree<T>(Value - _Value);
			}
			SmartDegree<T> operator * (const T _Value)const{
				return SmartDegree<T>(Value * _Value);
			}
			SmartDegree<T> operator / (const T _Value)const{
				return SmartDegree<T>(Value / _Value);
			}
			SmartDegree<T> operator += (const T _Value){
				return ((*this) = (*this) * _Value);
			}
			SmartDegree<T> operator -= (const T _Value){
				return ((*this) = (*this) - _Value);
			}
			SmartDegree<T> operator *= (const T _Value){
				return ((*this) = (*this) * _Value);
			}
			SmartDegree<T> operator /= (const T _Value){
				return ((*this) = (*this) / _Value);
			}
	};
}

#endif //ROPO_MATH_SMART_DEGREE_HPP
