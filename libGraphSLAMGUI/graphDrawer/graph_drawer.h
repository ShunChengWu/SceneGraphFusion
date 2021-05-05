#pragma once
#include "../../Utils/define.h"
#include <map>
#include <vector>
#include <Eigen/Core>
#include "text_drawer.h"
#include "node_drawer.h"
#include "edge_drawer.h"

namespace PSLAM {
    struct TripletForSotring {
        Eigen::Vector3f position3D;
        Eigen::Vector3f position;
        Eigen::Vector3f color;
        std::string text;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class GraphDrawer {
    public:
        GraphDrawer();
        void Init();
        void Draw(float width, float height,
                  const Eigen::Matrix4f &view_matrix,
                  const Eigen::Matrix4f &projection);
        void Update(//float width, float height,
                    const VecEigenf(3) &labelPositions,
                    const VecEigenf(3) &labelColors,
                    const std::vector<std::string> &texts,
                    const std::vector<unsigned int> &edgeIndices
                    );
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        NodeDrawer mNodeDrawer;
        TextDrawer mTextDrawer;
        EdgeDrawer mEdgeDrawer;
        std::vector<TripletForSotring> mTextBuffer;

        bool mbDrawNode, mbDrawEdge, mbDrawLabel;
    private:
        bool mbInited;


        void UpdateAndDrawLabel(float windowWidth, float windowHeight,
                                const VecEigenf(3) &labelPositions,
                                const VecEigenf(3) &labelColors,
                                const std::vector<std::string> &labelIndices,
                                const Eigen::Matrix4f &view_matrix,
                                const Eigen::Matrix4f &projection);

        void UpdateTextBuffer(const VecEigenf(3) &labelPositions,
                         const VecEigenf(3) &labelColors,
                         const std::vector<std::string> &texts);

        void DrawText(float windowWidth, float windowHeight,
                      const Eigen::Matrix4f &view_matrix,
                      const Eigen::Matrix4f &projection);
    };
}