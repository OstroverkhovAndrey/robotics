
#ifndef _MATRIX_HPP_
#define _MATRIX_HPP_

#include <vector>
#include <ostream>
#include <any>

class Matrix{
    friend std::ostream& operator<<(std::ostream& os, const Matrix& m);
public:
    Matrix();
    Matrix(int N, int M);
    Matrix(const Matrix& other);
    Matrix(const std::vector<std::vector<float>>);
    ~Matrix();

    void SetElement(int i, int j, float val);
    float GetElement(int i, int j) const;
    
    Matrix T();

    Matrix operator+(Matrix& other);
    Matrix operator+(const Matrix& other);
    Matrix operator-(Matrix& other);
    Matrix operator-(const Matrix& other);
    Matrix operator*(Matrix& other);
    Matrix operator*(const Matrix& other);

    Matrix operator=(const Matrix& other);

    float det();
    Matrix inverse();
    
    int N() const {return N_;}
    int M() const {return M_;}

    //operator float() const;
private:
    int N_;
    int M_;
    float **matrix_ = nullptr;
};

#endif // _MATRIX_HPP_
