#include "edge.h"
using namespace PSLAM;

Edge::Edge(Edge &edge) {
    this->nodeFrom = edge.nodeFrom;
    this->nodeTo = edge.nodeTo;
    this->labelProp = edge.labelProp;
    this->mClsWeight = edge.mClsWeight;
#ifdef COMPILE_WITH_GRAPHPRED
    this->mFeatures = edge.mFeatures;
#endif
}

std::string Edge::GetLabel() const {
    std::unique_lock<std::mutex> lock(mMutLabel);
    return label;
}

void Edge::UpdatePrediction(const std::map<std::string, float> &prop, bool fusion){
    std::unique_lock<std::mutex> lock(mMutex);
    if(prop.empty()) return;

    for (const auto &pair:prop) {
        const auto &name = pair.first;
        auto new_v = pair.second;
        if (labelProp.find(name) == labelProp.end()) {
            labelProp.insert({name, new_v});
            mClsWeight.insert({name, 1});
        } else {
            if(fusion) {
                const static float max_w = 100;
                const static float new_w = 1;
                const auto old_v = labelProp.at(name);
                const auto old_w = mClsWeight.at(name);
                labelProp.at(name) = (old_v * old_w + new_v * new_w) / float(old_w + new_w);
                mClsWeight.at(name) = (old_w + new_w);
                mClsWeight.at(name) = std::min(max_w, mClsWeight.at(name));
            } else {
                labelProp.at(name) = new_v;
            }
        }
    }

    if (labelProp.empty())return;
    float max_prop = labelProp.begin()->second;
    std::string ll;
    for(const auto &pair:labelProp){
        if(pair.second>=max_prop){
            max_prop = pair.second;
            ll = pair.first;
        }
    }
    std::unique_lock<std::mutex> lock_label(mMutLabel);
    label = ll;
}
