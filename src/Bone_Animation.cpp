#include "Bone_Animation.h"
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/constants.hpp>

using glm::vec3;
using glm::vec4;
using glm::mat3;
using glm::mat4;

Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
    root_position = { 2.0f, 0.5f, 2.0f };

    scale_vector = {
        {1.0f, 1.0f, 1.0f},
        {0.5f, 4.0f, 0.5f},
        {0.5f, 3.0f, 0.5f},
        {0.5f, 2.0f, 0.5f}
    };

    rotation_degree_vector = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 30.0f, 0.0f},
        {0.0f, 30.0f, 0.0f},
        {0.0f, 30.0f, 0.0f}
    };

    colors = {
        {0.7f, 0.0f, 0.0f, 1.0f},
        {0.7f, 0.7f, 0.0f, 1.0f},
        {0.7f, 0.0f, 0.7f, 1.0f},
        {0.0f, 0.7f, 0.7f, 1.0f}
    };

    target = { 3.0f, 8.0f, 3.0f };
    target_colors = { 0.0f, 1.0f, 0.0f, 1.0f };

    const size_t N = scale_vector.size();
    rotateX.assign(N, mat4(1.0f));
    rotateY.assign(N, mat4(1.0f));
    rotateZ.assign(N, mat4(1.0f));
    rotateMatrix.assign(N, mat4(1.0f));
    translateMatrix_origin.assign(N, mat4(1.0f));
    translateMatrix_link.assign(N, mat4(1.0f));
    translateMatrix_end.assign(N, mat4(1.0f));
    worldMatrix.assign(N, mat4(1.0f));
    bone_mat.assign(N, mat4(1.0f));

    isMoving = false;
    tree_depth = static_cast<int>(N);
    joint_num = tree_depth - 1;

    Rotation();
}

void Bone_Animation::update(float delta_time)
{
    pivot.clear();
    axisX.clear();
    axisY.clear();
    axisZ.clear();

    if (isMoving) {
        JacobianTranspose();
        Rotation();
    } else {
        Rotation();
    }
}

void Bone_Animation::resetRotation()
{
    isMoving = false;

    rotation_degree_vector = {
        {0.0f, 0.0f, 0.0f},
        {0.0f, 30.0f, 0.0f},
        {0.0f, 30.0f, 0.0f},
        {0.0f, 30.0f, 0.0f}
    };

    bone_mat.assign(bone_mat.size(), mat4(1.0f));
}

void Bone_Animation::resetTarget()
{
    isMoving = false;
    target = { 3.0f, 8.0f, 3.0f };
}

void Bone_Animation::Rotation()
{
    const int N = tree_depth;
    std::vector<float> angleX(N), angleY(N), angleZ(N);
    for (int i = 0; i < N; ++i) {
        angleX[i] = glm::radians(rotation_degree_vector[i][2]);
        angleY[i] = glm::radians(rotation_degree_vector[i][0]);
        angleZ[i] = glm::radians(rotation_degree_vector[i][1]);
    }

    rotateX = { glm::eulerAngleX(angleX[0]), glm::eulerAngleX(angleX[1]), glm::eulerAngleX(angleX[2]), glm::eulerAngleX(angleX[3]) };
    rotateY = { glm::eulerAngleX(angleY[0]), glm::eulerAngleY(angleY[1]), glm::eulerAngleY(angleY[2]), glm::eulerAngleY(angleY[3]) };
    rotateZ = { glm::eulerAngleX(angleZ[0]), glm::eulerAngleZ(angleZ[1]), glm::eulerAngleZ(angleZ[2]), glm::eulerAngleZ(angleZ[3])};

    for (int i = 0; i < N; ++i) {
        const int j = i - 1;
        rotateMatrix[i] = rotateX[i] * rotateZ[i] * rotateY[i];

        if (i == 0) {
            translateMatrix_origin[i] = glm::translate(mat4(1.0f), vec3(0.0f, scale_vector[i].y / 2.0f, 0.0f));
            translateMatrix_link[i]   = glm::translate(mat4(1.0f), vec3(root_position.x, (root_position.y - scale_vector[i].y / 2.0f), root_position.z));
            translateMatrix_end[i]    = glm::translate(mat4(1.0f), vec3(0.0f, scale_vector[i].y, 0.0f));

            worldMatrix[i] = translateMatrix_link[i] * rotateMatrix[i];
            bone_mat[i] = glm::translate(mat4(1.0f), root_position);
        } else {
            translateMatrix_origin[i] = glm::translate(mat4(1.0f), vec3(0.0f, scale_vector[i].y / 2.0f, 0.0f));
            translateMatrix_link[i]   = glm::translate(mat4(1.0f), vec3(0.0f, scale_vector[i - 1].y, 0.0f));
            translateMatrix_end[i]    = glm::translate(mat4(1.0f), vec3(0.0f, scale_vector[i].y, 0.0f));

            worldMatrix[i] = worldMatrix[i - 1] * translateMatrix_link[i] * rotateMatrix[i];
            bone_mat[i] = worldMatrix[i] * translateMatrix_origin[i];

            
            pivot.push_back(glm::vec3(worldMatrix[j] * translateMatrix_end[j] * vec4(0.0f, 0.0f, 0.0f, 1.0f)));

            axisX.push_back(glm::normalize(glm::vec3(worldMatrix[j] * vec4(1.0f, 0.0f, 0.0f, 0.0f))));
            axisY.push_back(glm::normalize(glm::vec3(worldMatrix[j] * rotateX[i] * rotateZ[i] * vec4(0.0f, 1.0f, 0.0f, 0.0f))));
            axisZ.push_back(glm::normalize(glm::vec3(worldMatrix[j] * rotateX[i] * vec4(0.0f, 0.0f, 1.0f, 0.0f))));
        }
    }

    endEffector = glm::vec3(worldMatrix[3] * translateMatrix_end[3] * vec4(0.0f, 0.0f, 0.0f, 1.0f));
    distance = glm::distance(target, endEffector);
}

void Bone_Animation::JacobianTranspose()
{
    std::vector<mat3> iJacobian;
    std::vector<mat3> iJacobianTrans;
    std::vector<vec3> deltaTheta;
    std::vector<vec3> numeratorRoot;

    if (distance < threshold) 
		return;

    for (int i = 0; i < joint_num; ++i) {
        vec3 c1 = glm::cross(axisX[i], (endEffector - pivot[i]));
        vec3 c2 = glm::cross(axisY[i], (endEffector - pivot[i]));
        vec3 c3 = glm::cross(axisZ[i], (endEffector - pivot[i]));

        iJacobian.push_back(mat3(c1, c2, c3));
        iJacobianTrans.push_back(glm::transpose(iJacobian.back()));
        numeratorRoot.push_back(iJacobianTrans.back() * (target - endEffector));
    }

    vec3 denominatorRoot(0.0f);
    for (int i = 0; i < joint_num; ++i) {
        denominatorRoot += iJacobian[i] * numeratorRoot[i];
    }

    float numer = 0.0f;
    for (int i = 0; i < joint_num; ++i) {
        numer += glm::dot(numeratorRoot[i], numeratorRoot[i]);
    }

    const float denom = glm::dot(denominatorRoot, denominatorRoot);
    if (denom <= std::numeric_limits<float>::epsilon()) return;

    alpha = numer / denom;

    for (int i = 0; i < joint_num; ++i) {
        vec3 dtheta = alpha * (iJacobianTrans[i] * (target - endEffector));
        deltaTheta.push_back(dtheta);

        rotation_degree_vector[i + 1] = rotation_degree_vector[i + 1] +
            vec3(deltaTheta[i].y, deltaTheta[i].z, deltaTheta[i].x);
    }
}
