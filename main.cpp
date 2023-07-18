#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "InterfaceManager.h"

int main(int argc, char *argv[])
{
    Mesh mesh = Mesh();

    mesh.InitMesh(Mesh::MeshType::SKELETON);
    bool needToPerformArap = false;
    EInitialisationType initialisationType = EInitialisationType::e_LastFrame;

    // Set up interface
    igl::opengl::glfw::Viewer viewer;
    InterfaceManager interfaceManager = InterfaceManager();

    viewer.callback_mouse_down = [&interfaceManager, &mesh](igl::opengl::glfw::Viewer &viewer, int, int modifier) -> bool
    {
        interfaceManager.onMousePressed(viewer, mesh, modifier & 0x00000001);
        return false;
    };
    viewer.callback_mouse_up = [&interfaceManager](igl::opengl::glfw::Viewer &viewer, int, int) -> bool
    {
        interfaceManager.onMouseReleased();
        return false;
    };
    viewer.callback_mouse_move = [&interfaceManager, &mesh, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, int, int modifier) -> bool
    {
        return interfaceManager.onMouseMoved(viewer, mesh, needToPerformArap, initialisationType);
    };
    viewer.callback_key_down = [&interfaceManager, &mesh, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) -> bool
    {
        interfaceManager.onKeyPressed(viewer, mesh, key, modifier & 0x00000001, needToPerformArap, initialisationType);
        return false;
    };
    viewer.callback_key_up = [&interfaceManager, &mesh, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) -> bool
    {
        interfaceManager.onKeyReleased();
        return false;
    };

    viewer.data().set_mesh(mesh.V, mesh.F);
    viewer.data().set_face_based(true);
    viewer.launch();
}
