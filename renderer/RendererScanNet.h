//
// Created by sc on 5/13/21.
//

#ifndef GRAPHSLAM_RENDERERSCANNET_H
#define GRAPHSLAM_RENDERERSCANNET_H
#include "Renderer.h"
#include <memory>

namespace PSLAM {
    class MeshRendererScanNet : public MeshRendererInterface {
    public:
        MeshRendererScanNet(int width, int height, const std::string &dataPath, const std::string &sequence, bool toReference):
                MeshRendererInterface(width,height,dataPath,sequence) {
            Init();
        }
    private:
        std::unique_ptr<glUtil::Shader> mShader;
        std::unique_ptr<glUtil::Model> mModel;
        void Init(){
            mShader = std::make_unique<glUtil::Shader>(vs, fs);
            mModel = std::make_unique<glUtil::Model>( m_folder+"/"+m_scanId+"/"+m_scanId+"_vh_clean_2.ply" );
            mModel->setShader(mShader.get());
        }

        void Render(const Eigen::Matrix4f &projection, const Eigen::Matrix4f &view, float near, float far) override{
            glEnable(GL_DEPTH_TEST);

            glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
            glClearColor(0.00f, 0.00f, 0.00f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glViewport(0,0,m_width,m_height);

            mShader->use();
            mShader->set("view", view);
            mShader->set("projection", projection);
            mShader->set("model", glm::mat4(1.f));
            mModel->Draw();

            RenderRGB(m_rgb);
            cv::flip(m_rgb, m_rgb, 0);
            cv::cvtColor(m_rgb, m_rgb, cv::COLOR_RGB2BGR);

            RenderDepth(m_depth,near,far);
            cv::flip(m_depth, m_depth, 0);
            m_depth.convertTo(m_depth, CV_16U);

            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        void RenderRGB(cv::Mat &img) {
            img = cv::Mat(m_height, m_width, CV_8UC3);
            glReadPixels(0, 0, m_width,m_height, GL_RGB, GL_UNSIGNED_BYTE,
                         img.data);
        }

        void RenderDepth(cv::Mat &img, float near, float far) {
            img.create(m_height, m_width, CV_32F);
            glReadPixels(0, 0,m_width,m_height, GL_DEPTH_COMPONENT, GL_FLOAT, img.data);
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


#endif //GRAPHSLAM_RENDERERSCANNET_H
