//
// Created by sc on 8/23/20.
//

#ifndef GRAPHSLAM_IMAGEDRAWER_H
#define GRAPHSLAM_IMAGEDRAWER_H

#include "../libGUI3D/libGUI3D/glShader.hpp"
#include <memory>
namespace PSLAM {
    class ImageDrawer {
    public:
        ~ImageDrawer();
        void Init(unsigned int textureWidth, unsigned int textureHeight, GLenum format = GL_RGBA);
        void Update(const unsigned char *data, int width, int height);
        void Draw(const Eigen::Matrix4f& projection, const Eigen::Matrix4f& viewMatrix);
    private:
        GLenum mFormat;
        unsigned int VAO, VBO, textID;
        std::unique_ptr<glUtil::Shader> mShader;
        bool bInited=false;
    };
}

#endif //GRAPHSLAM_IMAGEDRAWER_H
