// Code : UTF - 8
#ifndef ROPO_MATH_MATRIX_HPP
#define ROPO_MATH_MATRIX_HPP

#include <algorithm>
#include <cmath>

namespace RopoMath{
	
	template <class T> class Matrix{
		private: 
			static const int MaxMatrixSize = 10;
		public:
			T Value[MaxMatrixSize + 1][MaxMatrixSize + 1];
			int n,m;
			// Construction
			~Matrix(){}
			Matrix<T>():n(MaxMatrixSize), m(MaxMatrixSize){
				//Default construction
				for (int i = 1 ; i <= n ; i++){
					for (int j = 1 ; j <= m ; j++){
						Value[i][j] = (i == j);
					}
				}
			}
			Matrix(const int _n, const int _m):n(_n), m(_m){
				// zero matrix construction
				for (int i = 1 ; i <= n ; i++){
					for (int j = 1 ; j <= m ; j++){
						Value[i][j] = 0;
					}
				}
			}
			Matrix(const Matrix &x):n(x.n), m(x.m){
				//Copy construction
				for (int i = 1 ; i <= n ; i++){
					for (int j = 1 ; j <= m ; j++){
						Value[i][j] = x.Value[i][j];
					}
				}
			}
			// Operator
			T* operator [](const int i){return Value[i];}
			friend Matrix<T> operator + (const Matrix<T> &x , const Matrix<T> &y){
				Matrix<T> Tmp(x.n, x.m);
				if(x.n != y.n || x.m != y.m)return Tmp;
				else {
					for(int i = 1 ; i <= Tmp.n ; i++){
						for(int j = 1 ; j <= Tmp.m ; j++){
							Tmp.Value[i][j] = x.Value[i][j] + y.Value[i][j];
						}
					}
				}
				return Tmp;
			}
			friend Matrix<T> operator - (const Matrix<T> &x , const Matrix<T> &y){
				Matrix<T> Tmp(x.n, x.m);
				if(x.n != y.n || x.m != y.m)return Tmp;
				else {
					for(int i = 1 ; i <= Tmp.n ; i++){
						for(int j = 1 ; j <= Tmp.m ; j++){
							Tmp.Value[i][j] = x.Value[i][j] - y.Value[i][j];
						}
					}
				}
				return Tmp;
			}
			friend Matrix<T> operator * (const Matrix<T> &x , const Matrix<T> &y){
				Matrix<T> Tmp(x.n , y.m);
				if (x.m != y.n) return Tmp;
				else {
					for (int i = 1 ; i <= Tmp.n ; i++){
						for (int j = 1 ; j <= Tmp.m ; j++){
							for (int k = 1 ; k <= x.m ; k++){
								Tmp.Value[i][j] += x.Value[i][k] * y.Value[k][j];
							}
						}
					}
				}
				return Tmp;
			}
			friend Matrix<T> operator * (const Matrix<T> &x , const T &y){
				Matrix<T> Tmp(x);
				for(int i=1;i<=x.n;i++){
					for(int j=1;j<=x.m;j++){
						Tmp[i][j] *= y;
					}
				}
				return Tmp;
			}
			friend Matrix<T> operator * (const T &y ,const Matrix<T> &x){
				return x*y;
			}
			friend Matrix<T> operator ^ (const Matrix<T> &x , const int y){
				Matrix<T> Tmp; // Tmp == E
				Tmp.n = Tmp.m = x.n;
				if(x.n!=x.m)return x;
				Matrix<T> A = x;
				int b = y;
				if (b < 0){//when b < 0, A needs to inverse
					A = A.Inverse();
					b = -b;
				}
				while (b){//Quick Power
					if(b & 1)Tmp = Tmp * A;

					A = A * A;
					b >>= 1;
				}
				return Tmp;
			}
			// Methods
			Matrix<T> Transpose(){
				Matrix<T> Tmp(m, n);
				for (int i = 1 ; i <= n ; i++) {
					for (int j = 1 ; j <= m ; j++) {
						Tmp.Value[j][i] = Value[i][j];
					}
				}
				return Tmp;
			}
			T Determinant(){
				if (n != m) return 0;
				T Res = 1;
				Matrix<T> Tmp = (*this); 
				//化为上三角矩阵
				for (int j = 1 ; j <= n ; j++){
					for (int i = j ; i <= n ; i++){
						if (Tmp[i][j] == 0)continue;
						else{
							for (int k = 1 ; k <= n ; k++){
								std::swap(Tmp[i][k],Tmp[j][k]);
							}
							Res *= (-1);
							break;
						}
					}
					if(Tmp[j][j] == 0)return 0;
					for (int i = j+1 ; i <= n ; i++){
						T P = Tmp[i][j] / Tmp[j][j];
						for (int k = j ; k <= n ; k++){
							Tmp[i][k] -= P * Tmp[j][k];
						}
					}
				}
				//求行列式的值
				for (int i = 1 ; i <= n ; i++){
					Res *= Tmp[i][i];
				}
				return Res;
			}
			Matrix<T> Inverse(){
				Matrix<T> Res;
				Res.n = Res.m= n;
				Matrix<T> Tmp = (*this);
				if (this->Determinant() == 0) return Res;
				for (int j = 1; j <= n ; j++){
					//交换行，保证为阶梯矩阵
					for (int i = j ; i <= n ; i++){
						if (Tmp[i][j] == 0)continue;
						else{
							for (int k = 1 ; k <= n ; k++){
								std::swap(Tmp[i][k],Tmp[j][k]);
								std::swap(Res[i][k],Res[j][k]);
							}
							break;
						}
					}
					//化为上三角矩阵
					for (int i = j+1 ; i <= n ; i++){
						T p = Tmp[i][j] / Tmp[j][j];
						for(int k = 1 ; k <= n ; k++){
							Res[i][k] -= p * Res[j][k];
						}
						for (int k = j ; k <= n ; k++){
							Tmp[i][k] -= p * Tmp[j][k];
						}
					}
				}
				//化为对角阵
				for (int j = n ; j >= 1 ; j--){
					for (int i = j-1 ; i >= 1 ; i--){
						T P = Tmp[i][j] / Tmp[j][j];
						for (int k = 1 ; k <= n ; k++){
							Res[i][k] -= P * Res[j][k];
						}
						Tmp[i][j] -= P * Tmp[j][j];
					}
					for (int k = 1 ; k <= n ; k++){
						Res[j][k] /= Tmp[j][j];
					}
					Tmp[j][j] = 1;
				}
				return Res;
			}
	};
}

#endif //ROPO_MATH_MATRIX_HPP