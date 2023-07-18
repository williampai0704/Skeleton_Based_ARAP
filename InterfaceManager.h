#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "ARAPInitEnum.h"

class InterfaceManager
{
private:
    bool mouseIsPressed = false;
    bool shiftIsPressed = false;
    std::vector<int> selection;

    std::vector<int> getSelectedControlPointsIndex(const Mesh &mesh, bool invert = false) const; 
    std::vector<int> getNonSelectedControlPointsIndex(const Mesh &mesh) const;

public:
    void onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool isShiftPressed);
    void onMouseReleased();
    bool onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool &needArap, const EInitialisationType &initialisationType);
    void onKeyPressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, unsigned char key, bool isShiftPressed, bool &needArap, EInitialisationType &initType);
    void onKeyReleased();
    void displaySelectedPoints(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh) const;
};
