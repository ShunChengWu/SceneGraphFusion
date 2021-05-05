//
// Created by sc on 6/5/20.
//

#include "node_drawer.h"

#include <memory>
using namespace PSLAM;

NodeDrawer::NodeDrawer():
        mfPointSize(1), bInited(false), mTotalPoints(0), mPointBufferSize(1e3)
{}

const std::string vsShader =
        "#version 330\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aColor;\n"
        "out vec3 FragPos;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "out VS_OUT {\n"
        "    \tvec4 color;\n"
        "} vs_out;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = projection * view * model * vec4(aPos,1);\n"
        "    FragPos = vec3(view * model * vec4(aPos,1));\n"
        "    vs_out.color = vec4(aColor,1);\n"
        "}";
const std::string fsShader =
        "#version 330\n"
        "out vec4 FragColor;\n"
        "in vec3 FragPos;\n"
        "in vec4 fColor;\n"
        "in vec2 v_coord;\n"
        "uniform sampler2D texture1;\n"
        "void main()\n"
        "{\n"
        "    float sq_norm = dot(v_coord, v_coord);\n"
        "    if (sq_norm > 1.f) discard;\n"
        "    FragColor = (fColor);\n"
        "} ";
const std::string gsShader =
        "#version 330\n"
        "layout (points) in;\n"
        "layout (triangle_strip, max_vertices = 8) out;\n"
        "in VS_OUT {\n"
        "    \tvec4 color;\n"
        "} gs_in[];\n"
        "out vec4 fColor;\n"
        "out vec2 v_coord;\n"
        "uniform mat4 modelPose;\n"
        "uniform float radius;\n"
        "\n"
        "//const float radius = 0.01;\n"
        "void build_house(vec4 position)\n"
        "{    \n"
        "    fColor = gs_in[0].color; // gs_in[0] since there's only one input vertex\n"
        "    fColor.w = 0.8;"
        "    gl_Position = position + vec4(-radius, -radius, 0.0, 0.0); // 1:bottom-left   \n"
        "    v_coord = vec2(-1.0,-1.0);\n"
        "    EmitVertex();   \n"
        "    gl_Position = position + vec4( radius, -radius, 0.0, 0.0); // 2:bottom-right\n"
        "    v_coord = vec2(1.0,-1.0);\n"
        "    EmitVertex();\n"
        "    gl_Position = position + vec4(-radius,  radius, 0.0, 0.0); // 3:top-left\n"
        "    v_coord = vec2(-1.0,1.0);\n"
        "    EmitVertex();\n"
        "    gl_Position = position + vec4( radius,  radius, 0.0, 0.0); // 4:top-right\n"
        "    v_coord = vec2(1.0,1.0);\n"
        "    EmitVertex();\n"
        "    EndPrimitive();\n"
        "}\n"
        "void main() {    \n"
        "    build_house(gl_in[0].gl_Position);\n"
        "}";

void NodeDrawer::Init(){
    // load shader
    mShader = std::make_unique<glUtil::Shader>(vsShader,fsShader,gsShader);

    // create buffer
    updateVertexBuffer(mPointBufferSize);
//    uint numStride = 6;
//    size_t totalSize = mPointBufferSize * numStride;
//    glGenVertexArrays(1, &VAO);
//    glGenBuffers(1, &VBO);
//    glBindVertexArray(VAO);
//    glBindBuffer(GL_ARRAY_BUFFER, VBO);
//    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize, nullptr, GL_DYNAMIC_DRAW);
//    glEnableVertexAttribArray(0);
//    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), nullptr);
//    glEnableVertexAttribArray(1);
//    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat),
//                          (void *) (mPointBufferSize * 3 * sizeof(GLfloat)));
//    glBindBuffer(GL_ARRAY_BUFFER, 0);
//    glBindVertexArray(0);
//
//    bInited = true;

    mShader->use();
    Eigen::Matrix4f modelmat = Eigen::Matrix4f::Identity();
    mShader->set("model", modelmat);
    SetPointSize(mfPointSize);
}

void NodeDrawer::Update(const VecEigenf(3) *points, const VecEigenf(3) *colors){
    mTotalPoints = points->size();
    updateVertexBuffer(mTotalPoints);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    mTotalPoints * 3 * sizeof(GLfloat), points->data());
    glBufferSubData(GL_ARRAY_BUFFER, mPointBufferSize * 3 * sizeof(GLfloat),
                    mTotalPoints * 3 * sizeof(GLfloat), colors->data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void NodeDrawer::SetPointSize(float size){
    mfPointSize = size;
    mShader->use();
    mShader->set("radius", mfPointSize);
}

void NodeDrawer::Draw(const Eigen::Matrix4f& projection, const Eigen::Matrix4f& viewMatrix) {
    if(!bInited) throw std::runtime_error("NodeDrawer: Draw was called before Init!.\n");
    mShader->use();
    mShader->set("projection",projection);
    mShader->set("view",viewMatrix);
    glBindVertexArray(VAO);
    glDrawArrays(GL_POINTS, 0, mTotalPoints);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void NodeDrawer::updateVertexBuffer(size_t newSize, bool force) {
    if(bInited) {
        if (!force) {
            if (mPointBufferSize > newSize) return;
        } else {
            glDeleteVertexArrays(1, &VAO);
            glDeleteBuffers(1, &VBO);
        }
    }
    mPointBufferSize = (std::floor(newSize / 1e6) + 1) * 1e6;

    uint numStride = 8;
    size_t totalSize = mPointBufferSize * numStride;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * totalSize, nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat),
                          (void *) (mPointBufferSize * 3 * sizeof(GLfloat)));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    bInited=true;
}