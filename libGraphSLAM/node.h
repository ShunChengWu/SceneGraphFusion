#pragma once
#include "declaration.h"
#ifdef COMPILE_WITH_GRAPHPRED
#include "graphPredictor/MemoryBlock.h"
#endif
#include <inseg_lib/surfel.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <queue>
#include <mutex>
#include <atomic>
#include <iterator>

namespace PSLAM {
    class Node {
        typedef std::unique_lock<std::mutex> Lock;
    public:
        inline static std::string Unknown() { return "unknown"; };

        Node(int label);

        /// Thread
        int Add(const SurfelPtr &surfel);

        void Remove(const int index_in_graph_old);

        void Update(const int label,
                    const int index,
                    const Eigen::Vector3f &pos,
                    const Eigen::Vector3f &normal,
                    const std::array<unsigned char, 3> &color);

        void RemoveEdge(const EdgePtr &edge);

        void UpdatePrediction(const std::map<std::string, float> &pd,
                              const std::map<std::string, std::pair<size_t, size_t>> &sizeAndEdge, bool fusion);

        bool CheckConnectivity(Node *nodeP, float margin, bool modify);

        const std::string GetLabel() const;

        void UpdateSelectedNode(const size_t time, const size_t filter_size, const size_t num_pts, bool force);

        int GetPointSize();
    public:
        std::unordered_map<int, SurfelPtr> surfels;
        std::unordered_set<EdgePtr> edges;
        /// The idx of this node (label)
        int idx;
        std::atomic_int instance_idx;
        bool mDebug;
        /// The last predicted semantic label of this node
        size_t time_stamp;

        // threads
        std::vector<SurfelPtr> selected_surfels;
        Eigen::Vector3f mCentroid, mStd, mBBox_min, mBBox_max;
        size_t lastUpdatePropertySize;
        std::atomic_bool mbNeedUpdateNodeFeature;

        Eigen::Vector3f centroid, pos_sum;
        Eigen::Vector3f bbox_max, bbox_min;

        /// Shared between
        std::unordered_set<size_t> neighbors;

        /// Only access by thread_gcc
#ifdef COMPILE_WITH_GRAPHPRED
        std::map<std::string, std::shared_ptr<MemoryBlock>> mFeatures;
#endif
        bool bNeedCheckNeighbor = false;
        std::mutex mMutNode;
        mutable std::mutex mMutSurfel, mMutEdge, mMutNN, mMutSelected;

        /// Prediction related members
        // stores the class and its probability
        mutable std::mutex mMutPred;
        std::map<std::string, float> mClsProb;
        std::map<std::string, float> mClsWeight;
        std::map<std::string, std::pair<size_t, size_t>> mSizeAndEdge;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        /// This contains indices to be reused.
        std::queue<int> mqFreeIndices;
        /// The unique index for each input surfels.
        size_t mIdxCounter;

        /// Prediction related members
        mutable std::mutex mMutPDLabel;
        std::string last_class_predicted = Unknown();
    };
    typedef std::shared_ptr<Node> NodePtr;
    static inline NodePtr MakeNodePtr(int label) { return std::make_shared<Node>(label); }
}  // namespace inseg_lib