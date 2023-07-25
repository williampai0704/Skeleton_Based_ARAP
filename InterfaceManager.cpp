#include "InterfaceManager.h"
#include <igl/unproject_onto_mesh.h>
#include <igl/unproject_on_line.h>
#include <igl/unproject_on_plane.h>
// #include "ARAP.h"

void InterfaceManager::onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, bool isShiftPressed, Eigen::MatrixXd bone_index)
{
    std::cout << "Mouse pressed" << std::endl;
    mouseIsPressed = true;
    if (moveSelectionMode)
    {
        projectOnMoveDirection(viewer, lastProjectedPoint);
        return;
    }

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

        int selectedVertexIndex = mesh.F.row(fid)[closestVertex];
        for (int i = 0; i < bone_index.rows(); ++i)
        {
            Eigen::Vector2d index = bone_index.row(i);
            if (selectedVertexIndex - index[1] < 5 && selectedVertexIndex - index[1] >= 0)
            {
                selectedVertexIndex = index[1];
                break;
            }
        }

        if (isShiftPressed)
        {
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
            selection.push_back(selectedVertexIndex);
        }
    }
    else if (isShiftPressed)
        selection.clear();

    displaySelectedPoints(viewer, mesh, bone);
}

void InterfaceManager::onMouseReleased()
{
    std::cout << "Mouse released" << std::endl;
    mouseIsPressed = false;
}

bool InterfaceManager::onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, bool &needArap, const EInitialisationType &initialisationType)
{
    if (!mouseIsPressed)
        return false;

    if (moveSelectionMode)
    {
        // TODO: update Mesh when vertex move
        // TODO: the moevment of the vertex seems wrong
        Eigen::Vector3d projPoint = Eigen::Vector3d();
        projectOnMoveDirection(viewer, projPoint);
        // std::cout << lastProjectedPoint - projPoint << std::endl;
        Eigen::RowVector3d mouseMovement = (lastProjectedPoint - projPoint).transpose();
        lastProjectedPoint = projPoint;
        for (auto &cpp : getSelectedControlPoints(mesh))
        {
            // std::cout << cpp << std::endl;
            cpp->wantedVertexPosition += mouseMovement;
            needArap = initialisationType == EInitialisationType::e_LastFrame;
        }
        displaySelectedPoints(viewer, mesh, bone);
        return true;
    }
}

void InterfaceManager::projectOnMoveDirection(igl::opengl::glfw::Viewer &viewer, Eigen::Vector3d &projectionReceiver) const
{
    double x = viewer.current_mouse_x;
    double y = viewer.current_mouse_y;

    // move control points that are in the selection
    if (moveOnLine)
    {
        // Move along a line
        igl::unproject_on_line(Eigen::Vector2f(x, y), viewer.core().proj, viewer.core().viewport, Eigen::Vector3d(1, 1, 1), firstMoveDirection, projectionReceiver);
    }
    else
    {
        // Mode along a plane
        Eigen::Vector4f planeEquation = Eigen::Vector4f(firstMoveDirection.x(), firstMoveDirection.y(), firstMoveDirection.z(), 3);
        igl::unproject_on_plane(Eigen::Vector2f(x, y), viewer.core().proj, viewer.core().viewport, planeEquation, projectionReceiver);
    }
}

void InterfaceManager::onKeyPressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, Mesh &surface, unsigned char key, bool isShiftPressed, bool &needArap, EInitialisationType &initType)
{
    // std::cout << "pressed Key: " << key << " " << (unsigned int)key << std::endl;
    shiftIsPressed = isShiftPressed;
    if (key == 'M')
    {
        moveSelectionMode = !moveSelectionMode;
        std::cout << "pressed Key: M / mode: " << moveSelectionMode << std::endl;
    }
    else if (key == 'C')
    {
        for (const auto &i : selection)
        {
            mesh.addControlPoint(i);
        }

        displaySelectedPoints(viewer, mesh, bone);
    }
    else if (key == 'R')
    {
        int nbCP = mesh.getControlPointCount();
        for (const auto &i : selection)
            mesh.removeControlPoint(i);
        displaySelectedPoints(viewer, mesh, bone);
    }
    else if (key == 'X')
    {
        setMoveDirection(Eigen::Vector3d(1, 0, 0), isShiftPressed, viewer, mesh, bone);
        std::cout << "pressed Key: X" << std::endl;
    }
    else if (key == 'Y')
    {
        setMoveDirection(Eigen::Vector3d(0, 1, 0), isShiftPressed, viewer, mesh, bone);
        std::cout << "pressed Key: Y" << std::endl;
    }
    else if (key == 'Z')
    {
        setMoveDirection(Eigen::Vector3d(0, 0, 1), isShiftPressed, viewer, mesh, bone);
        std::cout << "pressed Key: Z" << std::endl;
    }
    else if (key == 'B')
    {
        displayBones(viewer, mesh, bone, surface);
        std::cout << "pressed Key: B" << std::endl;
    }
}

void InterfaceManager::onKeyReleased()
{
    // std::cout << "Key Released" << std::endl;
    shiftIsPressed = false;
}

std::vector<ControlPoint *> InterfaceManager::getSelectedControlPoints(Mesh &mesh) const
{
    std::vector<ControlPoint *> cp = mesh.getControlPointsW();
    std::vector<ControlPoint *> selection_cp = std::vector<ControlPoint *>();
    for (const auto &ptr : cp)
        for (const auto &i : selection)
            if (ptr->vertexIndexInMesh == i)
            {
                selection_cp.push_back(ptr);
                break;
            }

    return selection_cp;
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

void InterfaceManager::displaySelectedPoints(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh, const Mesh &bone) const
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

    if (moveOnLine && moveSelectionMode)
    {
        displayMoveAxis(viewer, firstMoveDirection, cppSelected);
    }

    if (isBone)
        viewer.data().add_points(bone.V, Eigen::RowVector3d(1, 1, 1));
}

void InterfaceManager::displayBones(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh, const Mesh &bone, const Mesh &surface)
{
    isBone = !isBone;
    // std::cout << bone.V << std::endl;
    if (isBone)
    {
        viewer.data().clear();
        displaySelectedPoints(viewer, mesh, bone);
        // viewer.data().add_points(bone.V, Eigen::RowVector3d(0, 0, 0));    // done in displaySelectedPoints() already
    }
    else
    {
        viewer.data().clear();
        viewer.data().set_mesh(mesh.V, mesh.F);
        viewer.append_mesh();
        viewer.data().set_mesh(surface.V, surface.F);
        displaySelectedPoints(viewer, mesh, bone);
    }
}

void InterfaceManager::displayMoveAxis(igl::opengl::glfw::Viewer &viewer, const Eigen::Vector3d &axisVector, const Eigen::MatrixXd &cppSelected) const
{
    const Eigen::RowVector3d axisTransposed = axisVector.transpose();
    Eigen::RowVector3d origin = Eigen::RowVector3d::Zero();
    for (int i = 0; i < cppSelected.rows(); i++)
        origin += cppSelected.row(i);

    Eigen::RowVector3d start = origin + axisTransposed * 5;
    Eigen::RowVector3d end = origin + axisTransposed * -5;
    viewer.data().add_edges(start, end, axisTransposed);
}

void InterfaceManager::setMoveDirection(const Eigen::Vector3d& direction, const bool& isShiftPressed, igl::opengl::glfw::Viewer& viewer, const Mesh& mesh, const Mesh& bone)
{
    firstMoveDirection = direction;
    moveOnLine = !isShiftPressed;
    displaySelectedPoints(viewer, mesh, bone);
}
