//
// Created by sc on 8/23/20.
//
#include "ImageDrawer.h"

using namespace PSLAM;
#include <string>


ImageDrawer::~ImageDrawer(){
    if(bInited){
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        glDeleteBuffers(1, &textID);
    }
}
void ImageDrawer::Init(unsigned int textureWidth, unsigned int textureHeight, GLenum format) {
    mFormat = format;
    const std::string vsShader = "#version 330\n"
                                 "layout(location = 0) in vec3 position;\n"
                                 "layout(location = 1) in vec2 vertexUV;\n"
                                 "out vec2 UV;\n"
                                 "void main() {\n"
                                 "    gl_Position = vec4(position, 1.0);\n"
                                 "    UV = vertexUV;\n"
                                 "}";
    const std::string fsShader = "#version 330\n"
                                 "in vec2 UV;\n"
                                 "uniform sampler2D myTextureSampler;\n"
                                 "out vec4 color;\n"
                                 "void main() {\n"
                                 "    color = vec4(texture(myTextureSampler, UV).rgb, 1.0);\n"
                                 "}";

    float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
            // positions   // texCoords
            -1.0f, 1.0f, 0.0f, 1.0f,
            -1.0f, -1.0f, 0.0f, 0.0f,
            1.0f, -1.0f, 1.0f, 0.0f,

            -1.0f, 1.0f, 0.0f, 1.0f,
            1.0f, -1.0f, 1.0f, 0.0f,
            1.0f, 1.0f, 1.0f, 1.0f
    };

    // Init shader
    mShader = std::make_unique<glUtil::Shader>(vsShader,fsShader);
    mShader->use();

    // screen quad VAO
    unsigned int &quadVAO = VAO;
    unsigned int &quadVBO = VBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) 0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *) (2 * sizeof(float)));

    glGenTextures(1, &textID);
    glBindTexture(GL_TEXTURE_2D, textID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, nullptr);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // For Alpha texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, mFormat == GL_RGBA ? GL_CLAMP_TO_EDGE
                                                                        : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, mFormat == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
}

void ImageDrawer::Update(const unsigned char *data, int width, int height) {
    glBindTexture(GL_TEXTURE_2D, textID);
    glTexImage2D(GL_TEXTURE_2D, 0, mFormat, width, height, 0, mFormat, GL_UNSIGNED_BYTE, data);
}

void ImageDrawer::Draw(const Eigen::Matrix4f& projection, const Eigen::Matrix4f& viewMatrix){
    mShader->use();
    /// Draw Image
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(VAO);
    glBindTexture(GL_TEXTURE_2D, textID);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}