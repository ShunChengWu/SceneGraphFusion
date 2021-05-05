//
// Created by sc on 6/5/20.
//
#include "edge_drawer.h"

#include <memory>
using namespace PSLAM;

EdgeDrawer::EdgeDrawer():
        bInited(false), mTotalSize(0),mIndiceSize(0),mVertexBufferSize(1e6), mIndiceBufferSize(1e6),
        mLineWith(3){}

void EdgeDrawer::Init(){
    // Shader
    mShader = std::make_unique<glUtil::Shader>(vsShader,fsShader);
//    mShader.Init("EdgeDrawer", vsShader,fsShader);
    mShader->use();

    //TOOD: make color depends on the edge label
    mShader->set("color",Eigen::Vector4f(5/255.f,205/255.f,212/255.f,1));
    Eigen::Matrix4f modelmat = Eigen::Matrix4f::Identity();
    mShader->set("model", modelmat);

    // Buffer
    size_t totalSize = mVertexBufferSize * 3;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize * 3, NULL, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * mIndiceBufferSize, NULL, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void EdgeDrawer::Draw(const Eigen::Matrix4f& projection,  const Eigen::Matrix4f& viewMatrix){
    if(mTotalSize==0 || mIndiceSize==0) return;
    mShader->use();
    mShader->set("projection",projection);
    mShader->set("view",viewMatrix);
    glBindVertexArray(VAO);
    glLineWidth(mLineWith);
    glDrawElements(GL_LINES, mIndiceSize, GL_UNSIGNED_INT, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void EdgeDrawer::Update(const VecEigenf(3) *points, const std::vector<unsigned int> *indices){
    mTotalSize = points->size();
    mIndiceSize = indices->size();
    if(mTotalSize==0 || mIndiceSize==0) return;
    updateVertexBuffer(mTotalSize);
    updateIndiceBuffer(mIndiceSize);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    mTotalSize * 3 * sizeof(GLfloat), points->data());

    glBindBuffer(GL_ARRAY_BUFFER, EBO);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                    mIndiceSize * sizeof(unsigned int), indices->data());

    glLineWidth(mLineWith);
//    glDrawElements(GL_LINES, mIndiceSize, GL_UNSIGNED_INT, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void EdgeDrawer::draw_impl(const VecEigenf(3) *points, const std::vector<unsigned int> *indices) {
    uint totalPoints = points->size();
    updateVertexBuffer(totalPoints);
    updateIndiceBuffer(indices->size());

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    totalPoints * 3 * sizeof(GLfloat), points->data());

    glBindBuffer(GL_ARRAY_BUFFER, EBO);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                    indices->size() * sizeof(unsigned int), indices->data());

    glLineWidth(mLineWith);
    glDrawElements(GL_LINES, indices->size(), GL_UNSIGNED_INT, nullptr);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void EdgeDrawer::updateVertexBuffer(size_t newSize, bool force) {
    if (!force)
        if (mVertexBufferSize > newSize) return;
    mVertexBufferSize = (std::floor(newSize / 1e6) + 1) * 1e6;

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    uint numStride = 8;
    size_t totalSize = mVertexBufferSize * numStride;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void EdgeDrawer::updateIndiceBuffer(size_t newSize, bool force) {
    if (!force)
        if (mIndiceBufferSize > newSize) return;
    mIndiceBufferSize = (std::floor(newSize / 1e6) + 1) * 1e6;

    glDeleteBuffers(1,&EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * mIndiceBufferSize, NULL, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}