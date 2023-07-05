#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"

int main(int argc, char *argv[])
{
    Mesh mesh = Mesh();

    mesh.InitMesh(Mesh::MeshType::SKELETON);

    igl::opengl::glfw::Viewer viewer;

    viewer.data().set_mesh(mesh.V, mesh.F);
    viewer.data().set_face_based(true);
    viewer.launch();
}