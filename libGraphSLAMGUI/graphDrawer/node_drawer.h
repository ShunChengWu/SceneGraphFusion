#pragma once
#include "../../Utils/define.h"
#include "../../libGUI3D/libGUI3D/glShader.hpp"
#include <memory>
#include <vector>
#include <memory>
#include <ORUtils/Vector.h>

namespace PSLAM {
    class NodeDrawer {
    public:
        NodeDrawer();

        void Init();

        void Update(const VecEigenf(3) *points, const VecEigenf(3) *colors);

        void Draw(const Eigen::Matrix4f& projection, const Eigen::Matrix4f& viewMatrix);

        void SetPointSize(float size);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        float mfPointSize;
    private:
        bool bInited;
        size_t mTotalPoints;
        std::unique_ptr<glUtil::Shader> mShader;
        size_t mPointBufferSize;
        unsigned int VAO{}, VBO{};

        void updateVertexBuffer(size_t newSize, bool force = false);
    };
}