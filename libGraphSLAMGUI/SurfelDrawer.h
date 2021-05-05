//
// Created by sc on 8/21/20.
//

#ifndef GRAPHSLAM_SURFELDRAWER_H
#define GRAPHSLAM_SURFELDRAWER_H

#include <string>
#include <memory>
#include <set>
#include "../libGUI3D/libGUI3D/glShader.hpp"
#include "../lib/inseg_lib/surfel.h"

namespace PSLAM {
    class SurfelDrawer {
    public:
        SurfelDrawer()=default;
        ~SurfelDrawer();

        void Init(std::function<void(Eigen::Vector3f&, const inseg_lib::Surfel*)> getcolor);

        void Update(const std::vector<std::shared_ptr<inseg_lib::Surfel>>& surfels);
        void Draw(const Eigen::Matrix4f& projection, const Eigen::Matrix4f& viewMatrix);

        float mDiffuseStrength=1.f;
    private:
        bool bInited=false;
        std::function<void(Eigen::Vector3f&, const inseg_lib::Surfel*)> pGetColor = nullptr;
        size_t mPointSize=0, mBufferSize = 1e3;
        unsigned int VAO{},VBO{};
        std::unique_ptr<glUtil::Shader> mShader;

        void UpdateBuffer(size_t newSize, bool force = false);
    };
}

#endif //GRAPHSLAM_SURFELDRAWER_H
