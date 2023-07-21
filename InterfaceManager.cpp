#include "InterfaceManager.h"
#include <igl/unproject_onto_mesh.h>
#include "ARAP.h"

void InterfaceManager::onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool isShiftPressed)
{
    std::cout << "Mouse pressed" << std::endl;
    mouseIsPressed = true;

    int fid;
    Eigen::Vector3d bc;
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
                                 viewer.core().proj, viewer.core().viewport, mesh.V, mesh.F, fid, bc))
    {
        int closestVertex = 0;
        for (int i = 1; i < 3; i++)
            if (bc[i] > bc[closestVertex])
                closestVertex = i;

        if (isShiftPressed)
        {
            int selectedVertexIndex = mesh.F.row(fid)[closestVertex];
            int indexOnVectorIfExists = -1;
            for (int i = 0; i < selection.size(); i++)
                if (selection[i] == selectedVertexIndex)
                {
                    indexOnVectorIfExists = i;
                    break;
                }
            if (indexOnVectorIfExists < 0)
                selection.push_back(selectedVertexIndex); // not in selection : add it
            else
                selection.erase(std::next(selection.begin(), indexOnVectorIfExists)); // already in the selection : remove it
        }
        else
        {
            selection.clear();
            selection.push_back(mesh.F.row(fid)[closestVertex]);
        }
    }
    else
        selection.clear();

    displaySelectedPoints(viewer, mesh);
}

void InterfaceManager::onMouseReleased()
{
    std::cout << "Mouse released" << std::endl;
    mouseIsPressed = false;
}

bool InterfaceManager::onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, bool &needArap, const EInitialisationType &initialisationType)
{
    if (mouseIsPressed && needArap)
    {
        // Get the control points based on the current selection
        std::vector<int> selectedControlPoints = getSelectedControlPointsIndex(mesh);

        // Perform ARAP deformation only on the selected control points
        int iterations = 10; // Set the number of iterations as needed
        EInitialisationType initType = EInitialisationType::e_LastFrame; // Set the desired initialisation type
        MatrixXd deformedVertices = arap(mesh, iterations, initType);

        // Update the selected control points' positions in the deformed mesh
        mesh.updateVertices(deformedVertices, selectedControlPoints);
    }
    return false;
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

std::vector<int> InterfaceManager::getSelectedControlPointsIndex(const Mesh &mesh, bool invert) const
{
    std::vector<int> selection_cp = std::vector<int>();
    for (const auto &i : selection)
        if (mesh.isAControlPoint(i) ^ invert)
            selection_cp.push_back(i);

    return selection_cp;
}

std::vector<int> InterfaceManager::getNonSelectedControlPointsIndex(const Mesh &mesh) const
{
    std::vector<int> selection_cp = std::vector<int>();
    std::vector<int> all_cp = mesh.getControlPointsIndex();
    for (const auto &i : all_cp)
    {
        bool notInSelection = true;
        for (const auto &j : selection)
            if (i == j)
            {
                notInSelection = false;
                break;
            }
        if (notInSelection)
            selection_cp.push_back(i);
    }

    return selection_cp;
}

void InterfaceManager::displaySelectedPoints(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh) const
{
    // retrieve the control points not selected
    Eigen::MatrixXd cpNotSelected = mesh.getVerticesFromIndex(getNonSelectedControlPointsIndex(mesh));
    Eigen::MatrixXd cppNotSelected = mesh.getControlPointsWantedPositionBySelection(selection, true);
    // retrieve the control points selected
    Eigen::MatrixXd cpSelected = mesh.getVerticesFromIndex(getSelectedControlPointsIndex(mesh));
    Eigen::MatrixXd cppSelected = mesh.getControlPointsWantedPositionBySelection(selection);
    // retrieve the standard points selected
    Eigen::MatrixXd notCpSelected = mesh.getVerticesFromIndex(getSelectedControlPointsIndex(mesh, true));

    viewer.data().set_points(cpNotSelected, Eigen::RowVector3d(0, 0.5, 0));
    viewer.data().add_points(cppNotSelected, Eigen::RowVector3d(0, 0.5, 0));
    viewer.data().add_points(cpSelected, Eigen::RowVector3d(0, 1, 0));
    viewer.data().add_points(cppSelected, Eigen::RowVector3d(0, 1, 0));
    viewer.data().add_points(notCpSelected, Eigen::RowVector3d(1, 0, 0));
}