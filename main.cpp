#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "ARAP.h"
#include <fstream>
#include <string>
#include "Libs/Pinocchio/Pinocchio.h"
#include "AnimalUI/processor.h"

using namespace std;


struct Face {
    int v1, v2, v3;
};

// Read vertices and faces 
tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_off()
{
    string filePath = "./../mesh/pseudo_mesh_1.off";
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


int main(int argc, char **argv)
{
    Mesh mesh = Mesh();
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> offFile = read_off();
    mesh.V = get<0>(offFile);
    mesh.F = get<1>(offFile);
    cout << "Reading complete !" << endl;

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

    // // Check N metrix
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

    // // Check W matrix
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

     vector<string> args;
    for (int i = 0; i < argc; ++i)
        args.push_back(argv[i]);
    
    Eigen::MatrixXd c = process(args);
}
