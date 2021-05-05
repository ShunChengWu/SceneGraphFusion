//
// Created by sc on 11/1/20.
//

#ifndef RENDERVIEW_MESHRENDERER_H
#define RENDERVIEW_MESHRENDERER_H


#include "../libGUI3D/libGUI3D/glModel.hpp"
#include <memory>
#include <string>
#include <opencv2/imgproc.hpp>

namespace PSLAM {
    class MeshRenderer{
    public:
        GLFWwindow *mWindow = nullptr;
        MeshRenderer(int width, int height, const std::string &path){
            mWidth=width;
            mHeight=height;
            mShader = std::make_unique<glUtil::Shader>(vs, fs);
            mModel = std::make_unique<glUtil::Model>( path );
            mModel->setShader(mShader.get());
            Init(width,height);
        }
        
        void Render(const Eigen::Matrix4f &projection, const Eigen::Matrix4f &view, float near, float far,
                cv::Mat &rgb, cv::Mat &depth){
            glEnable(GL_DEPTH_TEST);

            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
            glClearColor(0.00f, 0.00f, 0.00f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glViewport(0,0,mWidth,mHeight);

            mShader->use();
            mShader->set("view", view);
            mShader->set("projection", projection);
            mShader->set("model", glm::mat4(1.f));
            mModel->Draw();

            RenderRGB(rgb);
            cv::flip(rgb, rgb, 0);
            cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);

            RenderDepth(depth,near,far);
            cv::flip(depth, depth, 0);
            depth.convertTo(depth, CV_16U);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        template<typename T>
        void Render(const T &projection, const T &view){
            mShader->use();
            mShader->set("view", view);
            mShader->set("projection", projection);
            mShader->set("model", glm::mat4(1.f));
            mModel->Draw();
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
        
        ~MeshRenderer(){
            glDeleteFramebuffers(1,&framebuffer);
            glDeleteRenderbuffers(1, &rbo);
            glDeleteTextures(1,&textureColorbuffer);
        }

//        float mNearPlane, mFarPlane;
    private:
        GLuint framebuffer,textureColorbuffer,rbo;
        int mWidth, mHeight;
        std::unique_ptr<glUtil::Shader> mShader;
        std::unique_ptr<glUtil::Model> mModel;


        void Init(int width, int height){
            glGenFramebuffers(1, &framebuffer);
            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

            glGenTextures(1, &textureColorbuffer);
            glBindTexture(GL_TEXTURE_2D, textureColorbuffer);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorbuffer, 0);

            // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
            glGenRenderbuffers(1, &rbo);
            glBindRenderbuffer(GL_RENDERBUFFER, rbo);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32F, width, height); // use a single renderbuffer object for both a depth AND stencil buffer.
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo); // now actually attach it

            if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
                std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        void RenderRGB(cv::Mat &img) {
            img = cv::Mat(mHeight, mWidth, CV_8UC3);
            glReadPixels(0, 0, mWidth,mHeight, GL_RGB, GL_UNSIGNED_BYTE,
                         img.data);
        }

        void RenderDepth(cv::Mat &img, float near, float far) {
            img.create(mHeight, mWidth, CV_32F);
            glReadPixels(0, 0,mWidth,mHeight, GL_DEPTH_COMPONENT, GL_FLOAT, img.data);
            for (size_t i = 0; i < (size_t) img.rows * img.cols; ++i) {
                if (img.at<float>(i) == 1 || img.at<float>(i) == 0) img.at<float>(i) = -1;
                else {
                    const float zn = (img.at<float>(i) * 2 - 1);
                    const float ze = 2.0f * near * far / (far + near - zn * (far - near));
                    img.at<float>(i) = ze * 1000;;
                }
            }
        }

        std::string vs =
                "#version 330 core\n"
                "layout (location = 0) in vec3 aPos;\n"
                "layout (location = 1) in vec3 aColor;\n"
                "out vec3 color;\n"
                "uniform mat4 model;\n"
                "uniform mat4 view;\n"
                "uniform mat4 projection;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
                "    color = aColor;\n"
                "}";

        std::string fs =
                "#version 330 core\n"
                "out vec4 FragColor;\n"
                "\n"
                "in vec3 color;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    FragColor = vec4(color,1);\n"
                "}";
    };
}

#endif //RENDERVIEW_MESHRENDERER_H
