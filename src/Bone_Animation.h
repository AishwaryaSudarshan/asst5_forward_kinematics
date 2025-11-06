#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Bone_Animation
{
public:
    Bone_Animation();
    ~Bone_Animation();

    void init();
    void update(float delta_time);

    std::vector<glm::vec3> scale_vector;
    std::vector<glm::vec3> rotation_degree_vector;
    std::vector<glm::vec4> colors;
    std::vector<glm::mat4> bone_mat;

    glm::vec3 root_position;
    glm::vec3 target;
    glm::vec3 endEffector;
    glm::vec4 target_colors;

    bool isMoving;
    int tree_depth;

    void resetRotation();
    void resetTarget();

private:
    std::vector<glm::mat4> rotateX;
    std::vector<glm::mat4> rotateY;
    std::vector<glm::mat4> rotateZ;
    std::vector<glm::mat4> rotateMatrix;
    std::vector<glm::mat4> translateMatrix_origin;
    std::vector<glm::mat4> translateMatrix_link;
    std::vector<glm::mat4> translateMatrix_end;
    std::vector<glm::mat4> worldMatrix;

    std::vector<glm::vec3> axisX;
    std::vector<glm::vec3> axisY;
    std::vector<glm::vec3> axisZ;
    std::vector<glm::vec3> pivot;

    int joint_num = 0;
    float threshold = 1e-6f;
    float distance = 0.0f;
    float alpha = 0.0f;

    void Rotation();
    void JacobianTranspose();
};

