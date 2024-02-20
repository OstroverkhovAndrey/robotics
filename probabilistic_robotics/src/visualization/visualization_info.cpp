
#include "visualization/visualization_info.hpp"

VisualizationInfo::VisualizationInfo(
        int x, int y, DimChart dim, VisualizationMode mode,
        std::vector<int> coordinate) :
    size_x(x), size_y(y), dimension(dim), vis_mode(mode),
    coordinate_for_visualization(coordinate) {
}

