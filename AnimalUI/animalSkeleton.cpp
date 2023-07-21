#include "animalSkeleton.h"
#include <fstream>
#include <sstream>

AnimalSkeleton::AnimalSkeleton(string filepath)
{
    std::ifstream file(filepath);
    string line;
    while (getline(file, line))
    {
        int idx;
        string jointName, previousJointName;
        double posX, posY, posZ;

        std::stringstream ss(line);
        ss >> idx >> jointName >> posX >> posY >> posZ >> previousJointName;
        // std::cout << idx << " " << getJointForName(previousJointName) << std::endl;

        if (previousJointName == string("None"))
            makeJoint(jointName, Vector3(posX, posY, posZ));
        else
            makeJoint(jointName, Vector3(posX, posY, posZ), previousJointName);
    }

    // makeSymmetric("pelvis.L", "pelvis.R");
    // makeSymmetric("thigh.L", "thigh.R");
    // makeSymmetric("shin.L", "shin.R");
    // makeSymmetric("foot.L", "foot.R");

    // makeSymmetric("shoulder.L", "rshoulder.R");
    // makeSymmetric("front_thigh.L", "front_thigh.R");
    // makeSymmetric("front_shin.L", "front_shin.R");
    // makeSymmetric("front_foot.L", "front_foot.R");

    initCompressed();

    // setFoot("foot.L");
    // setFoot("foot.R");
    // setFoot("front_foot.L");
    // setFoot("front_foot.R");

    file.close();
}
