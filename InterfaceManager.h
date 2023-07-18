#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "ARAPInitEnum.h"

class InterfaceManager
{
private:
    bool mouseIsPressed = false;
    bool shiftIsPressed = false;
    std::vector<int> selection;

public:
    void onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool isShiftPressed);
    void onMouseReleased();
    bool onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool &needArap, const EInitialisationType &initialisationType);
    void onKeyPressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, unsigned char key, bool isShiftPressed, bool &needArap, EInitialisationType &initType);
    void onKeyReleased();
};
