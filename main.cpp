#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "InterfaceManager.h"
#include "ARAP.h"
#include <fstream>
#include <string>
using namespace std;


struct Face {
    int v1, v2, v3;
};

// Read vertices and faces 
std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_off()
{
    string filePath = "./../mesh/pseudo_mesh.off";
    ifstream file(filePath);
    if (!file) {
        cerr << "cannot open file" << endl;
    }
    string header;
    int numVertices, numFaces, numEdges;
    
    file >> header >> numVertices >> numFaces >> numEdges;

    Eigen::MatrixXd Vi(numVertices, 3);
    Eigen::MatrixXi F(numFaces,3);

    if (header != "COFF") {
        cerr << "Not off file!" << endl;
    }
    // Read vertices
    for (int i = 0; i < numVertices; ++i) {
        Eigen::Vector3d vertex;
        file >> vertex(0) >> vertex(1) >> vertex(2);
        // cout << vertex[0] << " " << vertex[1] << " " << vertex[2] << endl;
        Vi.row(i) = vertex;
    } 
    // Read faces
    for (int i = 0; i < numFaces; ++i) {
        int numVerticesInFace;
        file >> numVerticesInFace;
        if (numVerticesInFace != 3) {
            cerr << "only support traingular mesh" << endl;
        }
        Eigen::Vector3i face;
        file >> face[0] >> face[1] >> face[2];
        F.row(i) = face;
    }
    file.close();
    return std::tuple<Eigen::MatrixXd, Eigen::MatrixXi>{Vi,F};
}


int main(int argc, char *argv[])
{
    Mesh mesh = Mesh();
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> offFile = read_off();
    mesh.V = get<0>(offFile);
    mesh.F = get<1>(offFile);
    cout << "Reading complete !" << endl;

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

    // Center the mesh
    mesh.V = mesh.V.rowwise() - mesh.V.colwise().mean();

    cout << mesh.V.rows() << " vertices loaded." << endl;
    cout << mesh.F.rows() << " faces loaded" << endl;

    //Check read file
    // cout << "mesh.V" << endl;
    // for (int i = 0; i < mesh.V.rows(); i++)
    // {
    //     cout << i << endl;
    //     cout << mesh.V(i,0) << " " << mesh.V(i,1) << " " << mesh.V(i,2) << " " << endl;
    // }
    // cout << "mesh.F" << endl;
    // for (int i = 0; i < mesh.F.rows(); i++)
    // {
    //     cout << i << endl;
    //     cout << mesh.F(i,0) << " " << mesh.F(i,1) << " " << mesh.F(i,2) << " " << endl;
    // }
    bool needToPerformArap = false;
    EInitialisationType initialisationType = EInitialisationType::e_LastFrame;

    // Initialize neighbors, weights and Laplace-Beltrami matrix
    mesh.computeL_W_N();
    
    const Eigen::MatrixXd V_save = mesh.V;


}
