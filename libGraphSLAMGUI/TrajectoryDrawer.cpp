//
// Created by sc on 8/24/20.
//

#include "TrajectoryDrawer.h"

using namespace PSLAM;

TrajectoryDrawer::~TrajectoryDrawer() {
    if(bInited) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }
}

void TrajectoryDrawer::Init() {
    const std::string shaderPath = std::string(GUI_FOLDER_PATH) + "Shaders/";
    mShader = std::make_unique<glUtil::Shader>(shaderPath+"camera_shader.vs",shaderPath+"camera_shader.fs");
    mShader->use();
    mShader->set("color", Eigen::Vector4f{0, 1, 0, 1.f});
    mShader->set("model", glm::mat4(1.f));
    UpdateBuffer(2<<8);
}

void TrajectoryDrawer::Add(glm::vec3 point, float interval){
    if(std::isnan(point.x) || std::isinf(point.x)) return;
    const std::string name = "Trajectory";
    auto new_size = mvTrajectories.size()+1;
    UpdateBuffer(new_size);

    if (mvTrajectories.empty()) {
        mvTrajectories.emplace_back(point);
    } else {
        const auto &prev = mvTrajectories.back();
        if (glm::distance(point, prev) > interval) {
            mvTrajectories.push_back(point);
            glBindVertexArray(VAO);
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0,
                            mvTrajectories.size() * 3 * sizeof(GLfloat), mvTrajectories.data());
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }
    }
}

void TrajectoryDrawer::Draw(const Eigen::Matrix4f &projection, const Eigen::Matrix4f &viewMatrix) {
    mShader->use();
    mShader->set("projection", projection);
    mShader->set("view", viewMatrix);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINE_STRIP, 0, mvTrajectories.size());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void TrajectoryDrawer::UpdateBuffer(size_t newSize, bool force) {
    if(bInited) {
        if (!force) {
            if (mBufferSize > newSize) return;
        } else {
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
        }
    }
    mBufferSize = (std::floor(newSize / 1e6) + 1) * 1e6;
    {
        auto tmp = mvTrajectories;
        mvTrajectories.clear();
        mvTrajectories.reserve(mBufferSize);
        mvTrajectories.resize(tmp.size());
        std::copy(tmp.begin(),tmp.end(),mvTrajectories.begin());
    }
//    SCLOG(DEBUG) << "new surfel buffer size: " << mBufferSize;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * mBufferSize * 3, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    bInited=true;
}