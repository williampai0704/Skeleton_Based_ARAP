#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "ARAPInitEnum.h"

class InterfaceManager
{
private:
    bool mouseIsPressed = false;
    bool shiftIsPressed = false;
    bool moveSelectionMode = false;
    std::vector<int> selection;
    Eigen::Vector3d lastProjectedPoint = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d firstMoveDirection = Eigen::Vector3d(1, 0, 0);
    bool moveOnLine = true;

    std::vector<ControlPoint *> getSelectedControlPoints(Mesh &mesh) const;
    std::vector<int> getSelectedControlPointsIndex(const Mesh &mesh, bool invert = false) const;
    std::vector<int> getNonSelectedControlPointsIndex(const Mesh &mesh) const;
    void projectOnMoveDirection(igl::opengl::glfw::Viewer &viewer, Eigen::Vector3d &projectionReceiver) const;
    void displayMoveAxis(igl::opengl::glfw::Viewer &viewer, const Eigen::Vector3d &axisVector, const Eigen::MatrixXd &cppSelected) const;
    void setMoveDirection(const Eigen::Vector3d& direction, const bool& isShiftPressed, igl::opengl::glfw::Viewer& viewer, const Mesh& mesh, const Mesh& bone);

public:
    bool isBone = false;
    void onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, bool isShiftPressed, Eigen::MatrixXd bone_index);
    void onMouseReleased();
    bool onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, bool &needArap, const EInitialisationType &initialisationType);
    void onKeyPressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, Mesh &surface, unsigned char key, bool isShiftPressed, bool &needArap, EInitialisationType &initType);
    void onKeyReleased();
    void displaySelectedPoints(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh, const Mesh &bone) const;
    void displayBones(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh, const Mesh &bone, const Mesh &surface);
};
