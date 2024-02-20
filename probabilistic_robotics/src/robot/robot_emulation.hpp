
#ifndef _ROBOT_EMULATION_HPP_
#define _ROBOT_EMULATION_HPP_

#include "base/math/matrix.hpp"
#include "robot/robot.hpp"

class RobotEmulation : public Robot {
public:

    RobotEmulation();

    bool move(Matrix u) override;
    Matrix get_sensor_measurement() override;

    Matrix get_real_state() override {return robot_state_;};


private:
    // это состояние эмулирует случайных характер движения робота
    // и не должно передаваться из этого класса в явном виде
    // кроме случаев визуализации
    Matrix robot_state_;

    // матрицы описывающие изменение состояни объекта
    // могут быть матрицами функциями, но в простом случае константы
    // или почти константы
    Matrix F;
    Matrix B;

    // описывает случайных характер движения
    Matrix Q;

    // описывает точность работы датчика
    Matrix R;

    // описывает преобразования между измерениями датчика и вектором состояния
    // системы, в самом простом случае единичная матрица
    Matrix H;
};

#endif // _ROBOT_EMULATION_HPP_
