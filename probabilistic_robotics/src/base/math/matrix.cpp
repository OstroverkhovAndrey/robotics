
#include <iostream>

#include "base/math/matrix.hpp"

Matrix::Matrix() : N_(0), M_(0) {}

Matrix::Matrix(int N, int M) : N_(N), M_(M) {
    matrix_ = new float*[N_];
    for (int i = 0; i < N_; ++i) {
        matrix_[i] = new float[M_];
        for (int j = 0; j < M_; ++j) {
            matrix_[i][j] = 0;
        }
    }
}

Matrix::Matrix(const Matrix& other) : N_(other.N_), M_(other.M_){
    matrix_ = new float*[N_];
    for (int i = 0; i < N_; ++i) {
        matrix_[i] = new float[M_];
        for (int j = 0; j < M_; ++j) {
            matrix_[i][j] = other.matrix_[i][j];
        }
    }
}

Matrix::Matrix(const std::vector<std::vector<float>> other) :
    N_(other.size()), M_(other[0].size()) {
    matrix_ = new float*[N_];
    for (int i = 0; i < N_; ++i) {
        matrix_[i] = new float[M_];
        for (int j = 0; j < M_; ++j) {
            matrix_[i][j] = other[i][j];
        }
    }
}

Matrix::~Matrix() {
    if (matrix_ == nullptr) return;
    for (int i = 0; i < N_; ++i) {
        if (matrix_[i] == nullptr) continue;
        delete[] matrix_[i];
    }
    delete[] matrix_;
}

void Matrix::SetElement(int i, int j, float val) {
    matrix_[i][j] = val;
}

float Matrix::GetElement(int i, int j) const{
    return matrix_[i][j];
}

Matrix Matrix::T() {
    Matrix matrix_T(M_, N_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < M_; ++j) {
            matrix_T.matrix_[j][i] = matrix_[i][j];
        }
    }
    return matrix_T;
}

Matrix Matrix::operator+(Matrix& other) {
    Matrix matrix_ans(N_, M_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < M_; ++j) {
            matrix_ans.matrix_[i][j] = matrix_[i][j] + other.matrix_[i][j];
        }
    }
    return matrix_ans;
}

Matrix Matrix::operator+(const Matrix& other) {
    Matrix matrix_ans(N_, M_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < M_; ++j) {
            matrix_ans.matrix_[i][j] = matrix_[i][j] + other.matrix_[i][j];
        }
    }
    return matrix_ans;
}

Matrix Matrix::operator-(Matrix& other) {
    Matrix matrix_ans(N_, M_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < M_; ++j) {
            matrix_ans.matrix_[i][j] = matrix_[i][j] - other.matrix_[i][j];
        }
    }
    return matrix_ans;
}

Matrix Matrix::operator-(const Matrix& other) {
    Matrix matrix_ans(N_, M_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < M_; ++j) {
            matrix_ans.matrix_[i][j] = matrix_[i][j] - other.matrix_[i][j];
        }
    }
    return matrix_ans;
}

Matrix Matrix::operator*(Matrix& other) {
    if (M_ != other.N_) {
        std::cout << "ошибка при умножении матриц!" << std::endl;
        throw "ошибка при умножении матриц!";
    }
    Matrix matrix_ans(N_, other.M_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < other.M_; ++j) {
            matrix_ans.matrix_[i][j] = 0;
            for (int k = 0; k < M_; ++k) {
                matrix_ans.matrix_[i][j] += matrix_[i][k]*other.matrix_[k][j];
            }
        }
    }
    return matrix_ans;
}

Matrix Matrix::operator*(const Matrix& other) {
    if (M_ != other.N_) {
        std::cout << "ошибка при умножении матриц!" << std::endl;
        throw "ошибка при умножении матриц!";
    }
    Matrix matrix_ans(N_, other.M_);
    for (int i = 0; i < N_; ++i) {
        for (int j = 0; j < other.M_; ++j) {
            matrix_ans.matrix_[i][j] = 0;
            for (int k = 0; k < M_; ++k) {
                matrix_ans.matrix_[i][j] += matrix_[i][k]*other.matrix_[k][j];
            }
        }
    }
    return matrix_ans;
}

Matrix Matrix::operator=(const Matrix& other) {
    if (this == &other) {
        return *this;
    }
    if (matrix_ != nullptr) {
        for (int i = 0; i < N_; ++i) {
            if (matrix_[i] != nullptr) {
                delete[] matrix_[i];
            }
        }
        delete[] matrix_;
    }
    N_ = other.N_;
    M_ = other.M_;
    matrix_ = new float*[N_];
    for (int i = 0; i < N_; ++i) {
        matrix_[i] = new float[M_];
        for (int j = 0; j < M_; ++j) {
            matrix_[i][j] = other.matrix_[i][j];
        }
    }
    return *this;
}

float Matrix::det() {
    if (N_ != M_) return 0;
    else if (N_ == 2) {
        return matrix_[0][0]*matrix_[1][1] - matrix_[0][1]*matrix_[1][0];
    }
    else {
        // not implemented yet
        return 0;
    }
}

Matrix Matrix::inverse() {
    if (N_ != M_) return Matrix();
    else if (N_ == 2) {
        if (det() != 0) {
            Matrix matrix_ans(2, 2);
            matrix_ans.SetElement(0, 0, matrix_[1][1]/det());
            matrix_ans.SetElement(0, 1, -matrix_[0][1]/det());
            matrix_ans.SetElement(1, 0, -matrix_[1][0]/det());
            matrix_ans.SetElement(1, 1, matrix_[0][0]/det());
            return matrix_ans;
        }
    }
    else {
        // not implemented yet
        return Matrix();
    }
    return Matrix();
}

//Matrix::operator float() const {
//    std::cout << "operator float(), matrix_size: " << N_ << " " << M_ << std::endl;
//    return matrix_[0][0];
//    //if (N_ == 1 && M_ == 1) {
//    //    return matrix_[0][0];
//    //}
//    //std::cout << "Я сам себе выстрелил в ногу!!! "  << matrix_[0][0] << std::endl;
//    //return 0;
//}


std::ostream& operator<<(std::ostream& os, const Matrix& m) {
    os << "Matrix: size (" << m.N_ << ", " << m.M_ << ")\n";
    for (int i = 0; i < m.N_; ++i) {
        for (int j = 0; j < m.M_; ++j) {
            os << m.matrix_[i][j] << " ";
        }
        os << "\n";
    }
    return os;
}

