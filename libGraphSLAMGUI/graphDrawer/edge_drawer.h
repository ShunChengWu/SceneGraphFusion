#pragma once
#include "../../Utils/define.h"
#include <vector>
#include "../../libGUI3D/libGUI3D/glShader.hpp"
#include <memory>

namespace PSLAM {
    class EdgeDrawer {
    public:
        explicit EdgeDrawer();

        void Init();

        void Update(const VecEigenf(3) *points, const std::vector<unsigned int> *indices);

        void Draw(const Eigen::Matrix4f& projection,  const Eigen::Matrix4f& viewMatrix);

    private:
        bool bInited;
        size_t mTotalSize, mIndiceSize, mVertexBufferSize, mIndiceBufferSize;
        unsigned int mLineWith;
        unsigned int VAO{},VBO{}, EBO{};
        std::unique_ptr<glUtil::Shader> mShader;
//        Shader mShader;

        void draw_impl(const VecEigenf(3) *points, const std::vector<unsigned int> *indices);

        void updateVertexBuffer(size_t newSize, bool force = false);

        void updateIndiceBuffer(size_t newSize, bool force = false);

        const std::string vsShader =
                "#version 330 core\n"
                "layout (location = 0) in vec3 aPos;\n"
                "\n"
                "uniform mat4 model;\n"
                "uniform mat4 view;\n"
                "uniform mat4 projection;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    \tgl_Position = projection * view * model * vec4(aPos,1);\n"
                "}";
        const std::string fsShader =
                "#version 330 core\n"
                "out vec4 FragColor;\n"
                "\n"
                "uniform vec4 color;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    FragColor = color;   \n"
                "}";

    };
}