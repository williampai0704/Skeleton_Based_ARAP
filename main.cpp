#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "InterfaceManager.h"
#include "ARAP.h"
#include <fstream>
#include <string>
#include "Libs/Pinocchio/Pinocchio.h"

using namespace std;


struct Face {
    int v1, v2, v3;
};

// Read vertices and faces 
tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_off()
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

Eigen::MatrixXd read_attachment()
{
    string filePath = "./../mesh/attachment.out";
    ifstream file(filePath);
    if (!file) {
        cerr << "cannot open file" << endl;
    }
    // Read the data from the file and store it in a dynamic matrix
    std::vector<std::vector<double>> data;
    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> row;
        row.push_back(0.0);
        std::istringstream iss(line);
        double value;
        while (iss >> value) {
            row.push_back(value);
        }
        data.push_back(row);
    }

    // Close the file after reading
    file.close();

    // Determine the dimensions of the matrix
    size_t rows = data.size();
    size_t cols = (rows > 0) ? data[0].size() : 0;

    // Create an Eigen matrix and populate it with data
    Eigen::MatrixXd matrix(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            matrix(i, j) = data[i][j];
        }
    }
    return matrix;
}


int main(int argc, char **argv)
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

    // // Check read file
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

    // // // Check N metrix
    // cout << "N" << endl;
    // for (int i = 0; i < mesh.N.size(); i++)
    // {
    //     cout << "face" << i << endl;
    //     std::list<int> n = mesh.N[i];
    //     for (std::list<int>::iterator it = n.begin(); it != n.end(); ++it)
    //     {
    //         cout << *it << " ";
    //     }
    //     cout << endl;
    // }

    // //Check W matrix
    // cout << "W" << endl;
    // for (int i = 0; i < mesh.W.rows(); i++)
    // {
    //     for (int j = 0; j < mesh.W.cols(); j++)
    //     {
    //         cout << mesh.W(i,j) << " ";
    //     }
    //     cout << endl;
    // }

    const Eigen::MatrixXd V_save = mesh.V;

    // // Check M matrix
    // Eigen::MatrixXd M = read_attachment();
    // for (int i = 0; i < M.rows(); i++)
    // {
    //     for(int j = 0; j < M.cols(); j++)
    //     {
    //         cout << M(i,j) << " ";
    //     }
    //     cout << endl;
    // } 

}
