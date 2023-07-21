#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "InterfaceManager.h"
#include "ARAP.h"

void performARAP(Mesh& mesh, const EInitialisationType& initialisationType, igl::opengl::glfw::Viewer& viewer, const InterfaceManager& interfaceManager)
{
    mesh.V = arap(mesh, 100, initialisationType);
    interfaceManager.displaySelectedPoints(viewer, mesh);
    viewer.data().set_mesh(mesh.V, mesh.F);
}

int main(int argc, char *argv[])
{
    Mesh mesh = Mesh();

    mesh.InitMesh(Mesh::MeshType::SKELETON);

    // Center mesh
    mesh.V = mesh.V.rowwise() - mesh.V.colwise().mean();

    bool needToPerformArap = false;
    EInitialisationType initialisationType = EInitialisationType::e_LastFrame;

    // Compute neightbours and weights and matrix
    mesh.computeL_W_N();
    const Eigen::MatrixXd V_save = mesh.V;

    // Set up interface
    igl::opengl::glfw::Viewer viewer;
    InterfaceManager interfaceManager = InterfaceManager();

    viewer.callback_pre_draw = [&interfaceManager, &mesh, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer& viewer)->bool
    {
        if (needToPerformArap)
        {
            std::cout << "Current mesh.V: " << mesh.V << std::endl;
            performARAP(mesh, initialisationType, viewer, interfaceManager);
            needToPerformArap = false;
            std::cout << "mesh.V after ARAP: " << mesh.V << std::endl;
        }

        return false;
    };
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
