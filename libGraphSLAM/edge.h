#pragma once
#ifdef COMPILE_WITH_GRAPHPRED
#include "graphPredictor/MemoryBlock.h"
#endif
#include <memory>
#include <map>
#include <mutex>
namespace PSLAM {
    class Edge {
    public:
        inline static std::string None() {return "none";}
        inline static std::string Same() {return "same part";}
        Edge() = default;
        Edge(Edge &edge);

        int nodeFrom{}, nodeTo{};
        /// Shared
        std::map<std::string, float> labelProp;
        std::map<std::string, float> mClsWeight;
        /// Thread_SG
#ifdef COMPILE_WITH_GRAPHPRED
        std::map<std::string, std::shared_ptr<MemoryBlock>> mFeatures;
#endif
        mutable std::mutex mMutex, mMutLabel;

        std::string GetLabel() const;
        void UpdatePrediction(const std::map<std::string, float> &prop, bool fusion);

        std::string label = None();
    };
    typedef std::shared_ptr<Edge> EdgePtr;
    static inline EdgePtr MakeEdgePtr(){return std::make_shared<Edge>();}
}
