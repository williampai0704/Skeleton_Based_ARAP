#include <iostream>
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "InterfaceManager.h"
#include "ARAP.h"
#include <fstream>
#include <string>
#include <algorithm>

using namespace std;

void performARAP(Mesh &pseudo_mesh, Mesh &bone, const EInitialisationType& initialisationType, igl::opengl::glfw::Viewer& viewer, const InterfaceManager& interfaceManager)
{
    pseudo_mesh.V = arap(pseudo_mesh, 100, initialisationType);
}

Eigen::Matrix4d compute_trans_matrix(Eigen::Vector3d vector1, Eigen::Vector3d vector2)
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
            // std::cout << "Transformation Matrix (Scaling):" << std::endl;
            // std::cout << transformationMatrix << std::endl;

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
void compute_LBS(Eigen::MatrixXd& B_previous, MatrixXd& B_after, MatrixXd& A, Mesh& surface)
{
    std::vector<Eigen::Matrix4d> LBS;
    for (int i = 0; i < B_previous.rows()-1; ++i)
    {
        Eigen::Vector3d previousBone = B_previous.row(i+1) - B_previous.row(i);
        Eigen::Vector3d newBone = B_after.row(i+1) - B_after.row(i);
        Eigen::Matrix4d M = compute_trans_matrix(previousBone,newBone);
        LBS.push_back(M);
        // std::cout << i << std::endl;
        // std::cout << M << std::endl;
    }

    MatrixXd new_Surface(surface.V.rows(),surface.V.cols());
    for (int i = 0; i < surface.V.rows(); ++i)
    {
        // cout << i << endl;
        Eigen::Matrix4d L = Matrix4d::Zero();
        for(int j = 0; j < B_previous.rows()-1; ++j)
        {
            // cout << j << endl;
            L += LBS[j]* A(i,j);
        }
        // cout << L << endl;
        Eigen::Vector3d vec3D = surface.V.row(i);
        Eigen::Vector4d input_temp(vec3D[0],vec3D[1],vec3D[2], 1.0);
        Eigen::Vector4d output_temp = L * input_temp;
        new_Surface.row(i) = output_temp.head<3>();
        // cout << "finish" << endl;
        // cout << new_Surface.row(i) << endl;
    }
    // cout << "new surface" << new_Surface << endl;
    surface.V = new_Surface;

}

struct Face {
    int v1, v2, v3;
};

// Read vertices and faces 
tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_pseudo_off()
{
    string filePath = "./../mesh/pseudo_mesh_5.off";
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

tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_surface_off()
{
    string filePath = "./../mesh/bear_surface.off";
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

tuple<Eigen::MatrixXd, Eigen::MatrixXd> get_Bone(Eigen::MatrixXd V)
{
    string filePath = "./../mesh/pinocchino_match.txt";
    ifstream file(filePath);
    int numBones;
    file >> numBones;
    Eigen::MatrixXd Bi(numBones,2);
    for (int i = 0; i < numBones; ++i) 
    {
        Eigen::Vector2d vertex;
        file >> vertex(0) >> vertex(1);
        // cout << vertex[0] << " " << vertex[1] << " " << vertex[2] << endl;
        Bi.row(i) = vertex;
    } 
    Eigen::MatrixXd B(numBones,3);
    for (int i = 0; i < numBones; ++i)
    {
        Eigen::Vector2d index = Bi.row(i);
        B.row(index[0]) = V.row(index[1]);
    }
    return std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>{B, Bi};
}

Eigen::MatrixXd read_attachment()
{
    string filePath = "./../mesh/attachment_2.out";
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
    Mesh pseudo_mesh = Mesh();
    Mesh surface = Mesh();
    Mesh bone = Mesh();

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> poffFile = read_pseudo_off();
    pseudo_mesh.V = get<0>(poffFile);
    pseudo_mesh.F = get<1>(poffFile);

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> offFile = read_surface_off();
    surface.V = get<0>(offFile);
    surface.F = get<1>(offFile);

    Eigen::MatrixXd A = read_attachment();
    
    cout << "Reading complete !" << endl;

    // mesh.InitMesh(Mesh::MeshType::SKELETON);

    // // Center the mesh
    // pseudo_mesh.V = pseudo_mesh.V.rowwise() - pseudo_mesh.V.colwise().mean();
    // surface.V = surface.V.rowwise() - surface.V.colwise().mean();

    tuple<Eigen::MatrixXd, Eigen::MatrixXd> bone_tuple = get_Bone(pseudo_mesh.V);
    bone.V = get<0>(bone_tuple);
    Eigen::MatrixXd bone_index = get<1>(bone_tuple);

    cout << pseudo_mesh.V.rows() << " vertices loaded." << endl;
    cout << pseudo_mesh.F.rows() << " faces loaded" << endl;
    cout << surface.V.rows() << " vertices loaded." << endl;
    cout << surface.F.rows() << " faces loaded" << endl;
    // cout << A.rows() << " " <<  A.cols() << " attachments loaded." << endl;

    bool needToPerformArap = false;
    EInitialisationType initialisationType = EInitialisationType::e_LastFrame;

    // Compute neightbours and weights and matrix
    pseudo_mesh.computeL_W_N();

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

    const Eigen::MatrixXd V_save = pseudo_mesh.V;

    // Set up interface
    igl::opengl::glfw::Viewer viewer;
    InterfaceManager interfaceManager = InterfaceManager();

    viewer.callback_pre_draw = [&interfaceManager, &pseudo_mesh, &bone, &surface, &needToPerformArap, &initialisationType, &A](igl::opengl::glfw::Viewer& viewer)->bool
    {
        if (needToPerformArap)
        {
            // std::cout << "Current mesh.V: " << mesh.V << std::endl;
            Eigen::MatrixXd B_previous = get<0>(get_Bone(pseudo_mesh.V));
            performARAP(pseudo_mesh, bone, initialisationType, viewer, interfaceManager);
            Eigen::MatrixXd B_after = get<0>(get_Bone(pseudo_mesh.V));
            compute_LBS(B_previous,B_after, A, surface);
            bone.V = get<0>(get_Bone(pseudo_mesh.V));
            // viewer.data().clear();
            interfaceManager.displaySelectedPoints(viewer, pseudo_mesh, bone);
            // viewer.data().set_mesh(pseudo_mesh.V, pseudo_mesh.F);
            if (!interfaceManager.isBone)
            {
                viewer.data(0).set_mesh(pseudo_mesh.V, pseudo_mesh.F);
                // viewer.append_mesh();
                viewer.data(1).set_mesh(surface.V, surface.F);
            }
            needToPerformArap = false;
            // std::cout << "mesh.V after ARAP: " << mesh.V << std::endl;
            
        }

        return false;
    };
    viewer.callback_mouse_down = [&interfaceManager, &pseudo_mesh, &bone, &bone_index](igl::opengl::glfw::Viewer &viewer, int, int modifier) -> bool
    {
        interfaceManager.onMousePressed(viewer, pseudo_mesh, bone, modifier & 0x00000001, bone_index);
        return false;
    };
    viewer.callback_mouse_up = [&interfaceManager](igl::opengl::glfw::Viewer &viewer, int, int) -> bool
    {
        interfaceManager.onMouseReleased();
        return false;
    };
    viewer.callback_mouse_move = [&interfaceManager, &pseudo_mesh, &bone, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, int, int modifier) -> bool
    {
        return interfaceManager.onMouseMoved(viewer, pseudo_mesh, bone, needToPerformArap, initialisationType);
    };
    viewer.callback_key_down = [&interfaceManager, &pseudo_mesh, &bone, &surface, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) -> bool
    {
        interfaceManager.onKeyPressed(viewer, pseudo_mesh, bone, surface, key, modifier & 0x00000001, needToPerformArap, initialisationType);
        return false;
    };
    viewer.callback_key_up = [&interfaceManager, &pseudo_mesh, &needToPerformArap, &initialisationType](igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) -> bool
    {
        interfaceManager.onKeyReleased();
        return false;
    };

    viewer.data().set_mesh(pseudo_mesh.V, pseudo_mesh.F);
    viewer.append_mesh();
    viewer.data().set_mesh(surface.V, surface.F);
    viewer.data().set_face_based(true);
    viewer.launch();
}
