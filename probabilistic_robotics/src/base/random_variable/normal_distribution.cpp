
#include "normal_distribution.hpp"

#define PY_SSIZE_T_CLEAN

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <ostream>

#include <Python.h>

bool NormalDistribution::python_flag = false;

NormalDistribution::NormalDistribution(int dimension) : 
    dimension_(dimension),
    mean_(dimension_, 1),
    covariance_(dimension_, dimension_) {

    if (!python_flag) {
        setenv("PYTHONPATH", "./src/base/random_variable", 1);
        Py_Initialize();
        python_flag = true;
    }
}

bool NormalDistribution::SetMean(std::vector<float> mean) {
    for (int i = 0; i < dimension_; ++i) {
        mean_.SetElement(i, 0, mean[i]);
    }
    return true;
}

bool NormalDistribution::SetCovariance(std::vector<std::vector<float>> covariance) {
    for (int i = 0; i < dimension_; ++i) {
        for (int j = 0; j < dimension_; ++j) {
            covariance_.SetElement(i, j, covariance[i][j]);
        }
    }
    return true;
}

bool NormalDistribution::SetMean(Matrix mean) {
    mean_ = mean;
    return true;
}

bool NormalDistribution::SetCovariance(Matrix covariance) {
    covariance_ = covariance;
    return true;
}

float NormalDistribution::GetPoint(Matrix x) {
    //std::cout << "x: " << x.N() << " " << x.M() << std::endl;
    //std::cout << "mean_: " << mean_.N() << " " << mean_.M() << std::endl;
    Matrix t1 = x-mean_;
    //std::cout << "t1: " << t1.N() << " " << t1.M() << std::endl;

    //std::cout << "covariance_: " << covariance_.N() << " " << covariance_.M() << std::endl;
    Matrix t2 = covariance_.inverse();
    //std::cout << "t2: " << t2.N() << " " << t2.M() << std::endl;

    float ans = -0.5*((t1.T()*t2*t1).GetElement(0, 0));
    
    ans = pow(M_E, ans);
    ans /= pow(2*M_PI, dimension_/2);
    ans /= sqrt(covariance_.det());

    return ans;
}

Matrix NormalDistribution::GetRandomPoint() {
    std::cout << "!!!_1\n";
    Matrix ans = Matrix(dimension_, 1);

    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    int argc = 8;
    const char* argv[argc];
    argv[1] = "get_point";
    argv[2] = "get_point_dim2";

    argv[3] = std::to_string(mean_.GetElement(0, 0)).c_str();
    argv[4] = std::to_string(mean_.GetElement(1, 0)).c_str();

    argv[5] = std::to_string(covariance_.GetElement(0, 0)).c_str();
    argv[6] = std::to_string(covariance_.GetElement(1, 1)).c_str();
    argv[7] = std::to_string(covariance_.GetElement(0, 1)).c_str();
    if (argc < 3) {
        fprintf(stderr,"Usage: call pythonfile funcname [args]\n");
        return Matrix();
    }
    std::cout << "!!!_2\n";

    //setenv("PYTHONPATH", "./src/random_variable", 1);
    //Py_Initialize();
    std::cout << argv[1] << std::endl;
    pName = PyUnicode_DecodeFSDefault(argv[1]);
    /* Error checking of pName left out */

    std::cout << "pName: " << pName << std::endl;

    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    std::cout << "pModule: " << pModule << std::endl;

    if (pModule != NULL) {
        std::cout << "!!!_3\n";
        pFunc = PyObject_GetAttrString(pModule, argv[2]);
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(argc - 3);
            for (i = 0; i < argc - 3; ++i) {
                pValue = PyLong_FromLong(atoi(argv[i + 3]));
                if (!pValue) {
                    Py_DECREF(pArgs);
                    Py_DECREF(pModule);
                    fprintf(stderr, "Cannot convert argument\n");
                    return Matrix();
                }
                /* pValue reference stolen here: */
                PyTuple_SetItem(pArgs, i, pValue);
            }
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                //printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                //Py_DECREF(pValue);
                printf("Result of call: %d\n", PyList_Check(pValue));
                int count = (int) PyList_Size(pValue);
                printf("count : %d\n",count);
                float temp[count];
                PyObject *ptemp, *objectsRepresentation ;
                const char* a11;

                for (i = 0 ; i < count ; i++ )
                {
                    ptemp = PyList_GetItem(pValue,i);
                    objectsRepresentation = PyObject_Repr(ptemp);
                    //a11 = PyBytes_AS_STRING(objectsRepresentation);
                    a11 = PyUnicode_AsUTF8(objectsRepresentation);
                    temp[i] = (float)strtod(a11,NULL);
                    printf("res_%i = %f\n", i, temp[i]);
                }
                ans.SetElement(0, 0, temp[0]);
                ans.SetElement(1, 0, temp[1]);
                std::cout << "GET_POINT_SUCCESS!" << std::endl;
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return Matrix();
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", argv[2]);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", argv[1]);
        return Matrix();
    }
    //if (Py_FinalizeEx() < 0) {
    //    return Matrix();
    //}
    
    return ans;
}

//bool NormalDistribution::python_close() {
//    //Py_Finalize();
//    return true;
//}

NormalDistribution::~NormalDistribution() {
    //python_close();
}


NormalDistribution NormalDistribution::operator*(NormalDistribution & other) {

    Matrix K = covariance_ * (covariance_+other.covariance_).inverse();

    Matrix new_mean = mean_ + K * (other.mean_ - mean_);

    Matrix new_covariance = covariance_ - K * covariance_;

    NormalDistribution ans(2);

    ans.SetMean(std::vector<float>({new_mean.GetElement(0, 0), new_mean.GetElement(1, 0)}));
    
    ans.SetCovariance({{new_covariance.GetElement(0, 0),
                        new_covariance.GetElement(0, 1)},
                       {new_covariance.GetElement(1, 0),
                        new_covariance.GetElement(1, 1)}});
    return ans;
}

std::ostream& operator << (std::ostream &os, const NormalDistribution & distribution)
{
    return os << "mean: (" << distribution.mean_.GetElement(0, 0) << ", " <<
                            distribution.mean_.GetElement(1, 0) << "); covariance: ((" <<
                            distribution.covariance_.GetElement(0, 0) << ", " <<
                            distribution.covariance_.GetElement(0, 1) << "), (" <<
                            distribution.covariance_.GetElement(1, 0) << ", " <<
                            distribution.covariance_.GetElement(1, 1) << "));";
    //return os << person.getName() << " " << person.getAge();
}

