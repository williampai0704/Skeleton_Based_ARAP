#include "InterfaceManager.h"

void InterfaceManager::onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool isShiftPressed)
{
    std::cout << "Mouse pressed" << std::endl;
    mouseIsPressed = true;

    // TODO: select control point
}

void InterfaceManager::onMouseReleased()
{
    std::cout << "Mouse released" << std::endl;
    mouseIsPressed = false;
}

bool InterfaceManager::onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool &needArap, const EInitialisationType &initialisationType)
{
    return true;
}

void InterfaceManager::onKeyPressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, unsigned char key, bool isShiftPressed, bool &needArap, EInitialisationType &initType)
{
    std::cout << "pressed Key: " << key << " " << (unsigned int)key << std::endl;
    shiftIsPressed = isShiftPressed;
}

void InterfaceManager::onKeyReleased()
{
    std::cout << "Key Released" << std::endl;
    shiftIsPressed = false;
}