//
// Created by sc on 8/24/20.
//

#pragma once
//#include "PointNetGCN.h"
#include <memory>
#include <inseg_lib/lib.h>
#include <random>
#include "../node.h"
#include "../edge.h"
//#include "../nodeProperties.h"
#include "../graph.h"
#include "../config.h"
#include <json11.hpp>
#include "EatGCN.h"
#include <future>

namespace PSLAM {
#ifdef COMPILE_WITH_ONNX
    typedef std::tuple<MemoryBlock3D,MemoryBlock2D,MemoryBlock2D, std::map<size_t,size_t>,std::map<int, std::pair<size_t,size_t>> > GCNINPUTDATA;
    class GraphPredictor {
    public:
        typedef std::tuple< std::map<size_t,std::map<std::string,float>>, std::vector<EdgePtr>,std::map<int, std::pair<size_t,size_t>> > Prediction;

        explicit GraphPredictor( const ConfigPSLAM *configPslam);
        ~GraphPredictor();
//        std::tuple<std::map<size_t,size_t>,std::vector<Edge>> Predict(std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels);
        void RUN(Graph *graph);

        void Stop();

        void SetThread(bool option);

        /// Check if thread is dead
        bool Pin();

        bool Updated();

        void SetUpdate(bool option);

        std::shared_ptr<Prediction> Predict(
                const std::shared_ptr<GCNINPUTDATA>& data);

        const std::map<std::string, json11::Json>& GetParams();

        std::map<std::string,std::vector<std::pair<int,float>>> GetTimes();

        bool mbVerbose=false;
        std::map<size_t,std::string> mLabels, mRelationships;
        std::map<std::string,size_t> mLabelsName2Idx,mRelationshipsName2Idx;
    private:
        const ConfigPSLAM *mConfigPslam;
        std::unique_ptr<Ort::MemoryInfo> mMemoryInfo;
        std::unique_ptr<EatGCN> mEatGCN;
        Graph *mpGraph = nullptr;
        std::future<void> mThread;
        bool mbShouldStop = false;
        bool mbThread = true;
        bool source_to_target;
        bool mDebug = false;
        std::atomic_bool mUpdated;
        void Process_IMPL();
        void UpdateNodeFeature(const std::map<int,NodePtr> &selected_nodes);
        void UpdateEdgeFeature(const std::map<int,NodePtr> &selected_nodes);
        void UpdateGCNFeature(const std::map<int,NodePtr> &selected_nodes, size_t level);
        void UpdatePrediction(const std::map<int,NodePtr> &selected_nodes,
                const std::map<int,std::pair<size_t,size_t>> &sizeAndEdge);
        void UpdateShapeFeature(const std::vector<NodePtr> &selected_nodes);

        std::map<int, NodePtr> getNeighbor(const std::map<int,NodePtr> &vNodes, size_t min_pts);

        std::map<std::string,std::vector<std::pair<int,float>>> mTimes;
    };
#else
    class GraphPredictor {
    public:
    public:
        explicit GraphPredictor(const std::string &path_onnx){};

        std::tuple<std::map<size_t,size_t>,std::vector<Edge>> Predict(std::vector<std::shared_ptr<inseg_lib::Surfel>> &surfels){};

        bool mbVerbose=true;
        std::map<size_t,std::string> mLabels, mRelationships;
    private:
    };
#endif
    typedef std::shared_ptr<GraphPredictor> GraphPredictorPtr;
    static inline GraphPredictorPtr MakeGraphPredictor(const ConfigPSLAM *configPslam) {return std::make_shared<GraphPredictor>(configPslam);}
}