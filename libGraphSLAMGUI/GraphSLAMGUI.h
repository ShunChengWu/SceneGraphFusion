//
// Created by sc on 8/21/20.
//

#ifndef GRAPHSLAM_SURFELGUI_H
#define GRAPHSLAM_GRAPHSLAMGUI_H

#include "../libGUI3D/libGUI3D/GUI3D.h"
#include <dataLoader/dataset_loader.h>
#include "graphDrawer/graph_drawer.h"
#include "../libGraphSLAM/GraphSLAM.h"
#include "../Utils/eigen_glm.h"
#include "SurfelDrawer.h"
#include "ImageDrawer.h"
#include "CameraDrawer.h"
#include "TrajectoryDrawer.h"
#include <random>
#include "Label_NYU40.h"

#ifdef COMPILE_WITH_ASSIMP
#include "../renderer/Renderer.h"
#include "../renderer/RendererFactory.h"
#endif

namespace PSLAM {

    class GraphSLAMGUI : public SC::GUI3D {
    public:
        enum DRAWGRAPHTYPE {
            DRAWGRAPHTYPE_SEGMENTS, DRAWGRAPHTYPE_INSTANCE
        };
        enum COLORTYPE {
            COLOR_LABEL,COLOR_PHONG,COLOR_NORMAL,COLOR_COLOR,COLOR_UPDATED,COLOR_SEMANTIC,COLOR_INSTANCE,COLOR_PANOPTIC
        };
        enum LABELTYPE {
            LABEL_SEGMENT, LABEL_INSTANCE, LABEL_NAME
        };
        enum PROCESS {
            PROCESS_STOP, PROCESS_ONCE, PROCESS_CONTINUE
        };
        enum DRAWEDGETYPE{
            DRAWEDGETYPE_RELATION, DRAWEDGETYPE_NEIGHBOR
        };

        GraphSLAMGUI(GraphSLAM *graphSlam, DatasetLoader_base *dataloader);

        ~GraphSLAMGUI();

        void drawUI() override;

        void drawGL() override;

        void SetRender(int width, int height, const std::string &path, bool align);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        bool bFaceCulling=true;
        GraphSLAM *mpGraphSLAM;
        DatasetLoader_base *mpDataLoader;
        int mProcessMode=PROCESS_STOP;
        CameraDrawer mCameraDrawer;
        TrajectoryDrawer mTrajectoryDrawer;
        int mColorType = COLOR_LABEL;
        int mLabelType = LABEL_NAME;
        int mEdgeType = DRAWEDGETYPE_RELATION;
        int mDrawGraphType = DRAWGRAPHTYPE_SEGMENTS;
        int mSelectedNodeIdx = 0;
        std::map<int,bool> mEdgeUISwitch;
        cv::Mat mRGB, mDepth;

#ifdef COMPILE_WITH_ASSIMP
        std::unique_ptr<MeshRendererInterface> mMeshRender;
#endif

        GraphDrawer mGraphDrawer;
        SurfelDrawer mSurfelDrawer;

        ImageDrawer mImageDrawer[3];
        bool bProcess = true;
        bool bNeedUpdate = false;
        bool bRenderSurfel=true;
        bool bNeedUpdateTexture=false;
        bool bDrawTraj=true;
        bool bDrawCam=true;
        bool bRenderNodeLabel=true;
        bool bRenderEdgeLabel=true;
        bool bRecordImg=false;
        int mNodeFilterSize=512;

        void key_callback_impl(GLFWwindow* window, int key, int scancode, int action, int mods)override {}

        void MainUI();
        void Process();
        bool ProcessSLAM();
        void UpdateGraphRenderer();

        void UI_Class_Relationships();

        void UI_Graph_Info();

        void UI_Graph_Config();

        void UI_Graph_EdgeInfo(size_t node_id);

        void GetSurfelColor(Eigen::Vector3f& surfel_color, const inseg_lib::Surfel *surfel);

        std::string GetNodeLabel(const Node *node) const;

        std::string GetEdgeLabel(const Edge *edge);

        void GetEdgeColor(Eigen::Vector3f& edge_color, const PSLAM::Edge *edge);

        void RecordImg();
    };
}

#endif //GRAPHSLAM_SURFELGUI_H
