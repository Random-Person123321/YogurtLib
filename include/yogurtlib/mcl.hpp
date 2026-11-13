#pragma once

namespace yogurt {


struct Segment {
    double x1, y1;
    double x2, y2;
};

struct ParticleData {
    double x, y;
    double weight;
};

class FieldMap {
public:
    /**
     * @brief adds the walls to the MCL feild map
     *
     * @param half the distance from the exact center of your field to any inner wall (should be approx ~70.2 in)
     *
     * ### Example Code
     *
     *```cpp
     * void initalize() {
     *   yogurt::FieldMap fieldmap;
     *   drivetrain.FieldMap.addWalls(70.2);
     * }
     * ```

    */
    void addWalls(double half);

    /**
     * @brief adds a box to the MCL feild map
     *
     * @param cx the x position of the center of the box
     * @param cy the y position of the center of the box
     * @param width the width (x) of the box
     * @param depth the depth (y) of the box
     *
     * ### Example Code
     *
     *
     *```cpp
     * void initalize() {
     *   yogurt::FieldMap fieldmap;
     *   fieldMap.addBox(-48.0, - 24.0, 5.61, 5.61);
     * }
     * ```
    */
    void addBox(double cx, double cy, double width, double depth);

    const std::vector<Segment>& segment();

private:
    std::vector <Segment> fieldMap;

    void rayCast(double sx, double sy, double dx, double dy) const;
};

class MCL {
public:
    FieldMap fieldMap;


private:


};

} // namespace yogurt