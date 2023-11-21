// Code : UTF - 8
#ifndef ROPO_MATH_VECTOR_HPP
#define ROPO_MATH_VECTOR_HPP

#include <algorithm>
#include <cmath>
#include <cstring>
#include "Matrix.hpp"

namespace RopoMath{

	typedef enum { RowVector = 0, ColumnVector = 1 } VectorTypeList;

	template<class T> class Vector : public Matrix<T>{
        private:
            static const int MaxVectorSize = 10;
        public:
            VectorTypeList VectorType; // false : Row Vector  /  true : Column Vector 
			int VectorLength;
			// Construction
            ~Vector(){}
            Vector():Matrix<T>(1,1){
				VectorType = RowVector;
				VectorLength = 1;
				memset(Matrix<T>::Value,0,sizeof(Matrix<T>::Value));
			}
            Vector(VectorTypeList _VectorType,const int _VectorLength):VectorType(_VectorType),VectorLength(_VectorLength){
				if(VectorType == RowVector){
					Matrix<T>::n = 1;
					Matrix<T>::m = _VectorLength;
				}
				else {
					Matrix<T>::n = _VectorLength;
					Matrix<T>::m = 1;
				}
				memset(Matrix<T>::Value,0,sizeof(Matrix<T>::Value));
			}
            Vector(VectorTypeList _VectorType,const Matrix<T> &x,const int Tag = 1):VectorType(_VectorType){
				if(VectorType == RowVector){
					Matrix<T>::n = 1;
					Matrix<T>::m = x.m;
					VectorLength = x.m;
					for(int i = 1; i <= Matrix<T>::m; i++){
						Matrix<T>::Value[1][i] = x[Tag][i];
					}
				}
				else {
					Matrix<T>::n = x.n;
					Matrix<T>::m = 1;
					VectorLength = x.n;
					for(int i = 1; i <= Matrix<T>::n; i++){
						Matrix<T>::Value[i][1] = x[i][Tag];
					}
				}
			}
            Vector(const Matrix<T> &x):Matrix<T>(x){
				VectorType = (x.n >= x.m) ? ColumnVector:RowVector;
				VectorLength = (VectorType == ColumnVector) ? x.n : x.m;
			}
			// Operator
			T & operator [](const int i){
				return (VectorType == ColumnVector)?Matrix<T>::Value[i][1]:Matrix<T>::Value[1][i];
			}
    };
}

#endif //ROPO_MATH_VECTOR_HPP