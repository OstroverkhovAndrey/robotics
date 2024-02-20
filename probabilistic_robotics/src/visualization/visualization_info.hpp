
#ifndef _VISUALIZATION_INFO_HPP_
#define _VISUALIZATION_INFO_HPP_

#include <vector>

enum class DimChart {
    dim1_2D,
    dim2_2D,
    dim2_3D,
    dim3_3D,
};

enum class VisualizationMode {
    simple_one_ellipse,
    simple_five_ellipses,
    full,
};

class VisualizationInfo {
public:
    VisualizationInfo(int x, int y, DimChart dim, VisualizationMode mode, std::vector<int> coordinate);
    int size_x;
    int size_y;
    DimChart dimension;
    VisualizationMode vis_mode;
    std::vector<int> coordinate_for_visualization;
};

#endif // _VISUALIZATION_INFO_HPP_

