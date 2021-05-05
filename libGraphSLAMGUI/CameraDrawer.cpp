//
// Created by sc on 8/23/20.
//

#include "CameraDrawer.h"

using namespace PSLAM;

CameraDrawer::~CameraDrawer(){
    if(bInited) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &EBO);
    }
}

void CameraDrawer::Init() {
    const std::string vsShader = "#version 330\n"
                                 "layout(location = 0) in vec3 position;\n"
                                 "uniform mat4 model;\n"
                                 "uniform mat4 view;\n"
                                 "uniform mat4 projection;\n"
                                 "void main() {\n"
                                 "    gl_Position = projection * view * model * vec4(position, 1.0);\n"
                                 "}";

    const std::string fsShader = "#version 330 core\n"
                                 "out vec4 FragColor;\n"
                                 "uniform vec4 color;\n"
                                 "void main()\n"
                                 "{\n"
                                 "    FragColor = color;   \n"
                                 "}";

    unsigned int indices[] = {0, 1, 0, 2, 0, 3, 0, 4,
                              1, 2, 2, 3, 3, 4, 4, 1};
    const float size = 0.1f;
    float vertices[] = { 0, 0, 0,
                         -0.5f*size, -0.5f*size, 1*size,
                         0.5f*size, -0.5f*size, 1*size,
                         0.5f*size,  0.5f*size, 1*size,
                         -0.5f*size,  0.5f*size, 1*size};

    // screen quad VAO
    unsigned int &quadVAO = VAO;
    unsigned int &quadVBO = VBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(indices), &indices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) (2 * sizeof(float)));


    glGenBuffers(1, &VBO);
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);

    // Lines
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glBindVertexArray(0);


    // Init shader
    mShader = std::make_unique<glUtil::Shader>(vsShader,fsShader);
    mShader->use();
    mShader->set("color", color);
    mShader->set("model", glm::mat4(1.f));
    bInited=true;
}

void CameraDrawer::Draw(Eigen::Matrix4f model,
        const Eigen::Matrix4f &projection,
        const Eigen::Matrix4f &viewMatrix) {
    assert(bInited);
    model = model * mScale;// glm::scale(model,glm::vec3(scale, scale, scale));    // it's a bit too big for our scene, so scale it down

    mShader->use();
    mShader->set("color", color);
    mShader->set("model",model);
    mShader->set("view",viewMatrix);
    mShader->set("projection",projection);

    glBindVertexArray(VAO);
    glLineWidth(3);
    glDrawElements(GL_LINES, 16, GL_UNSIGNED_INT, 0);
    glLineWidth(1);
}