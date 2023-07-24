#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "InterfaceManager.h"
#include "ARAP.h"
#include <fstream>
#include <string>
#include <algorithm>

using namespace std;

Eigen::MatrixXd compute_bone(Eigen::MatrixXd V_mesh)
{
    Eigen::MatrixXd V_bone = Eigen::MatrixXd::Zero(V_mesh.rows()/5, 3);
    for (int i = 0; i < V_bone.rows(); ++i)
    {
        V_bone.row(i) = V_mesh.row(i*5);
        // cout << "i: " << i << ", i*5: " << i/5 << ", mesh.V.row(i*5): " << mesh.V.row(i*5) << endl;
    }
    return V_bone;
}

void performARAP(Mesh &mesh, Mesh &bone, const EInitialisationType& initialisationType, igl::opengl::glfw::Viewer& viewer, const InterfaceManager& interfaceManager)
{
    mesh.V = arap(mesh, 100, initialisationType);
    bone.V = compute_bone(mesh.V);
    interfaceManager.displaySelectedPoints(viewer, mesh, bone);
    viewer.data().set_mesh(mesh.V, mesh.F);    // TODO: rebuild the code structure surrounding InterfaceManager::isBone
}

Eigen::Matrix4d compute_trans_matrix(Eigen::Vector3d& vector1, Eigen::Vector3d& vector2)
{
    // Step 1: Normalize both vectors
    vector1.normalize();
    vector2.normalize();

    if (vector1.norm() == 0 || vector2.norm() == 0) {
        std::cerr << "Error: Zero vectors are not allowed as input." << std::endl;
    }

    // Step 2: Find the axis of rotation
    Eigen::Vector3d axis = vector1.cross(vector2);
    
    // Step 3: Check if the input vectors are collinear

    double dotProductVal = vector1.dot(vector2);
    double angle = std::acos(dotProductVal); // Angle between the vectors

    if (std::isnan(angle)) {
        // Vectors are collinear
        if (vector1.isApprox(vector2)) 
        {
            // Vectors are identical, no transformation is needed
            std::cout << "Vectors are identical" << std::endl;
            // Step 5: Calculate the translation vector
            Eigen::Vector3d translation = vector2 - vector1;
            // Step 6: Compute the translation matrix
            Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
            translationMatrix.block<3, 1>(0, 3) = translation;

            Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
            transformationMatrix(0, 0) = 1.0;
            transformationMatrix(1, 1) = 1.0;
            transformationMatrix(2, 2) = 1.0;
            transformationMatrix.block<3, 1>(0, 3) = translation;
            return transformationMatrix;
        } 
        else 
        {
            // Vectors are collinear with different magnitudes
            // Calculate the scaling factor to transform one vector to the other
            double scalingFactor = vector2.norm() / vector1.norm();

            // Construct the scaling transformation matrix
            Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
            transformationMatrix(0, 0) = scalingFactor;
            transformationMatrix(1, 1) = scalingFactor;
            transformationMatrix(2, 2) = scalingFactor;

            // Print the transformation matrix
            std::cout << "Transformation Matrix (Scaling):" << std::endl;
            std::cout << transformationMatrix << std::endl;

            return transformationMatrix;
        }
    }
    
    Eigen::Matrix3d skewSymmetricMat;
    skewSymmetricMat << 0, -axis[2], axis[1],
                        axis[2], 0, -axis[0],
                        -axis[1], axis[0], 0;

    Eigen::Matrix3d rotationMat = Eigen::Matrix3d::Identity() +
                                 skewSymmetricMat * std::sin(angle) +
                                 (skewSymmetricMat * skewSymmetricMat) * (1 - std::cos(angle));

    // Step 5: Calculate the translation vector
    Eigen::Vector3d translation = vector2 - vector1;

    // Step 6: Compute the translation matrix
    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix.block<3, 1>(0, 3) = translation;

    // Step 7: Combine the rotation and translation matrices to get the final transformation matrix
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMat;
    transformationMatrix.block<3, 1>(0, 3) = translation;

    if (transformationMatrix.hasNaN()) {
        std::cerr << "Error: Computed transformation matrix contains NaN values." << std::endl;
    }
    
    return transformationMatrix;
    // // Print the transformation matrix
    // std::cout << "Transformation Matrix:" << std::endl;
    // std::cout << transformationMatrix << std::endl;
}

// Compute Linear Blend Skinning
void compute_LBS(Eigen::MatrixXd V_previous, MatrixXd V_after, MatrixXd W)
{
    std::vector<Eigen::Matrix4d> LSB;
    for (int i = 0; i < V_previous.rows()-5; i+= 5)
    {
        Eigen::Vector3d previousBone = V_previous.row(i+5) - V_previous.row(i);
        Eigen::Vector3d newBone = V_after.row(i+5) - V_after.row(i);
        Eigen::Matrix4d M = compute_trans_matrix(previousBone,newBone);
        LSB.push_back(M);
        std::cout << i/5 << std::endl;
        std::cout << M << std::endl;
    }
}

struct Face {
    int v1, v2, v3;
};

// Read vertices and faces 
tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_off()
{
    string filePath = "./../mesh/pseudo_mesh_3.off";
    ifstream file(filePath);
    if (!file) {
        cerr << "cannot open file in read_off()" << endl;
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

tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_bone_txt()
{
    string filePath = "./../mesh/pinocchino_skeleton.txt";
    ifstream infile(filePath);
    if (!infile) {
        cerr << "cannot open file in read_bone_txt()" << endl;
    }
    string header1, header2, header3, header4, header5;
    // int numVertices, numFaces, numEdges;
    int numFaces = 0;
    int numVertices = std::count(std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>(), '\n') - 1;
    cout << "bone numVertices: " << numVertices << endl;
    
    ifstream file(filePath);
    if (!file) {
        cerr << "cannot open file in read_bone_txt()" << endl;
    }
    file >> header1 >> header2 >> header3 >> header4 >> header5;

    Eigen::MatrixXd Vi(numVertices, 3);
    Eigen::MatrixXi F(numFaces,3);

    // Read vertices
    for (int i = 0; i < numVertices; ++i) {
        Eigen::Vector3d vertex;
        int bummer1;
        string bummer2, bummer_last;
        file >> bummer1 >> bummer2 >> vertex(0) >> vertex(1) >> vertex(2) >> bummer_last;
        // cout << "file: " << bummer1 << " " << bummer2 << " " << vertex[0] << " " << vertex[1] << " " << vertex[2] << " " << bummer_last << endl;
        Vi.row(i) = vertex;
    } 
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

int main(int argc, char *argv[])
{
    Mesh mesh = Mesh();

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> offFile = read_off();
    mesh.V = get<0>(offFile);
    mesh.F = get<1>(offFile);
    Eigen::MatrixXd W = read_attachment();

    Mesh bone = Mesh();

    // get bone V method 1: read bone txt (DEPRECIATED! DON'T USE THIS)
    // std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> boneFile = read_bone_txt();
    // bone.V = get<0>(boneFile);
    // bone.F = get<1>(boneFile);

    // get bone V method 2: read from off file (first line of 5 lines is bone)
    bone.V = compute_bone(mesh.V);

    cout << "Reading complete !" << endl;

    // mesh.InitMesh(Mesh::MeshType::SKELETON);

    // // Center the mesh
    // mesh.V = mesh.V.rowwise() - mesh.V.colwise().mean();
    // bone.V = bone.V.rowwise() - bone.V.colwise().mean();

    cout << mesh.V.rows() << " vertices loaded." << endl;
    cout << mesh.F.rows() << " faces loaded" << endl;

    bool needToPerformArap = false;
    EInitialisationType initialisationType = EInitialisationType::e_LastFrame;

    // Compute neightbours and weights and matrix
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

    const Eigen::MatrixXd V_save = mesh.V;

    // Set up interface
    igl::opengl::glfw::Viewer viewer;
    InterfaceManager interfaceManager = InterfaceManager();

    viewer.callback_pre_draw = [&interfaceManager, &mesh, &bone, &needToPerformArap, &initialisationType, &W](igl::opengl::glfw::Viewer& viewer)->bool
    {
        if (needToPerformArap)
        {
            // std::cout << "Current mesh.V: " << mesh.V << std::endl;
            Eigen::MatrixXd V_previous = mesh.V;
            performARAP(mesh, bone, initialisationType, viewer, interfaceManager);
            Eigen::MatrixXd V_after = mesh.V;
            needToPerformArap = false;
            // std::cout << "mesh.V after ARAP: " << mesh.V << std::endl;
            compute_LBS(V_previous,V_after,W);
        }

        return false;
    };
    viewer.callback_mouse_down = [&interfaceManager, &mesh, &bone](igl::opengl::glfw::Viewer &viewer, int, int modifier) -> bool
    {
        interfaceManager.onMousePressed(viewer, mesh, bone, modifier & 0x00000001);
        return false;
    };
    viewer.callback_mouse_up = [&interfaceManager](igl::opengl::glfw::Viewer &viewer, int, int) -> bool
    {
        interfaceManager.onMouseReleased();
        return false;
    };
    viewer.callback_mouse_move = [&interfaceManager, &mesh, &bone, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, int, int modifier) -> bool
    {
        return interfaceManager.onMouseMoved(viewer, mesh, bone, needToPerformArap, initialisationType);
    };
    viewer.callback_key_down = [&interfaceManager, &mesh, &bone, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) -> bool
    {
        interfaceManager.onKeyPressed(viewer, mesh, bone, key, modifier & 0x00000001, needToPerformArap, initialisationType);
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
