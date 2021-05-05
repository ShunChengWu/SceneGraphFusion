//
// Created by sc on 8/21/20.
//

#ifndef GRAPHSLAM_GRAPHSLAM_H
#define GRAPHSLAM_GRAPHSLAM_H
#include "config.h"
#include "disjSet.h"
#include "graph.h"
#include "../Objects/Camera/CameraParameters.h"
#include <inseg_lib/lib.h>
#include <ORUtils/Logging.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <future>

#ifdef COMPILE_WITH_GRAPHPRED
#include "graphPredictor/GraphPredictor.h"
#else
#include <ORUtils/JsonUtil.h>
#endif
namespace PSLAM {
    class GraphSLAM {
    public:
        explicit GraphSLAM(ConfigPSLAM *config, const CameraParameters &camParamD);
        ~GraphSLAM();

        //////////
        /// Frameprocessing
        //////////
        bool Initialize(const CameraParameters &camParamD);

        void ProcessFrame(int idx, const cv::Mat &colorImage, const cv::Mat &depthImage, const Eigen::Matrix4f *pose);

        bool &UseThread(){return mConfig->use_thread;}

        //////////
        /// Access
        //////////
        inseg_lib::InSegLib *GetInSeg() { return inseg_.get(); }

        Graph *GetGraph() { return mGraph.get(); }

        ConfigPSLAM *GetConfig(){return mConfig;}


#ifdef COMPILE_WITH_GRAPHPRED
        GraphPredictor *GetGraphPred() { return mpGraphPredictor.get(); }
#endif

#ifdef COMPILE_WITH_JSON
        json11::Json GetSceneGraph(bool full);
#endif

        //////////
        /// IO
        //////////
        void SaveModel(const std::string &output_folder) const;

        void SaveGraph(const std::string &output_folder, bool fullProb);

        void SaveSurfelsToPLY(int segment_filter, const std::string &output_folder, const std::string &output_name, bool binary);

        enum SAVECOLORMODE {
            SAVECOLORMODE_RGB, SAVECOLORMODE_SEGMENT, SAVECOLORMODE_INSTANCE, SAVECOLORMODE_SEMANTIC, SAVECOLORMODEL_PANOPTIC
        };

        void SaveNodesToPLY(int segment_filter, const std::string &output_folder, SAVECOLORMODE saveColorMode, bool binary=false);

        void SaveSurfelsToPLY(const std::string &output_folder, const std::string &output_name,
                              const std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels, bool binary);

        //////////
        /// Graph
        //////////
        void RunFullPrediction();
        /// Start backend
        void Start();
        /// Stop backend
        void Stop();

        void AddSelectedNodeToUpdate(int idx);

        void LoadPredictModel(const std::string &path);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::set<int> mLastUpdatedSegments{};
    private:
        // system
        size_t mTimeStamp=0;
        bool mbInitMap = false;

        Eigen::Matrix4f pose_;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> trajectory_{};

        // Inseg Standard configuration.
        std::shared_ptr<inseg_lib::InSegLib> inseg_;

        // Graph
        ConfigPSLAM *mConfig;
        std::shared_ptr<Graph> mGraph;
#ifdef COMPILE_WITH_GRAPHPRED
        GraphPredictorPtr mpGraphPredictor;
#endif

        std::vector<std::shared_ptr<inseg_lib::Surfel>> GetUpdatedSurfels();

        std::vector<std::shared_ptr<inseg_lib::Surfel>> FilterSegment(
                int segment_filter, const std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels);



    };
}

#endif //GRAPHSLAM_GRAPHSLAM_H
