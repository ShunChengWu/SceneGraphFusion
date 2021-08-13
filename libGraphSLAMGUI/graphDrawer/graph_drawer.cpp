//
// Created by sc on 6/5/20.
//
#include "graph_drawer.h"
#include "../../Utils/define.h"
#include <map>
#include <vector>
#include <Eigen/Core>
#include <ORUtils/MathUtils.h>
//TODO: add label display on edges

using namespace PSLAM;

GraphDrawer::GraphDrawer():mbDrawNode(true), mbDrawEdge(true), mbDrawLabel(true),mbInited(false){

}

void GraphDrawer::Init(){
    mNodeDrawer.Init();
    mNodeDrawer.SetPointSize(0.1);
    mEdgeDrawer.Init();
    mTextDrawer.Init();
}

void GraphDrawer::Update(//float width, float height,
            const VecEigenf(3) &labelPositions,
            const VecEigenf(3) &labelColors,
            const std::vector<std::string> &texts,
            const std::vector<unsigned int> &edgeIndices
){
    mNodeDrawer.Update(&labelPositions,&labelColors);
    mEdgeDrawer.Update(&labelPositions, &edgeIndices);
    UpdateTextBuffer(labelPositions,labelColors,texts);
}

void GraphDrawer::Draw(float width, float height,
                       const Eigen::Matrix4f &view_matrix,
                       const Eigen::Matrix4f &projection){
    if(mbDrawNode)
        mNodeDrawer.Draw(projection, view_matrix);
    if(mbDrawLabel)
        DrawText(width,height,view_matrix,projection);
    if(mbDrawEdge)
        mEdgeDrawer.Draw(projection, view_matrix);
}

void GraphDrawer::UpdateTextBuffer(const VecEigenf(3) &labelPositions,
                                     const VecEigenf(3) &labelColors,
                                     const std::vector<std::string> &texts){
//            std::vector<std::pair<size_t,Eigen::Vector3f>, Eigen::aligned_allocator<std::pair<size_t, Eigen::Vector3f>>> labelPositionsOnImage(labelPositions.size());
    mTextBuffer.clear();
    mTextBuffer.reserve(labelPositions.size());
    for(size_t i=0;i<labelPositions.size();++i){
        mTextBuffer.emplace_back(TripletForSotring());
        auto &tripletForSotring = mTextBuffer.back();
        tripletForSotring.position3D = labelPositions[i];
        tripletForSotring.color    = labelColors[i];
        tripletForSotring.text      = texts[i];
    }
}

void GraphDrawer::DrawText(float windowWidth, float windowHeight, const Eigen::Matrix4f &view_matrix,
                           const Eigen::Matrix4f &projection){
    for(auto & buffer : mTextBuffer){
        const Eigen::Vector4f point (buffer.position3D.x(),buffer.position3D.y(),buffer.position3D.z(),1);
        Eigen::Vector4f projectedP = projection * view_matrix * point;
        projectedP.x() = projectedP.x() * windowWidth / 2 / projectedP.z() + windowWidth / 2; // the later windowHeight/ 2 should be cx
        projectedP.y() = projectedP.y() * windowHeight/ 2 / projectedP.z() + windowHeight/ 2;
        buffer.position = Eigen::Vector3f(projectedP.x(),projectedP.y(),projectedP.z());
    }
    // order them so the back text will not block the front
    std::sort(mTextBuffer.begin(), mTextBuffer.end(),
              [](const TripletForSotring &left, const TripletForSotring &right){
                  return left.position.z() < right.position.z();
              });
    for(auto & i : mTextBuffer){
        const auto &point = i.position;
        const auto &color = i.color;
        auto text = i.text;
        if(point.z() < 0) continue;
//        std::cout << text << ": " << point.z() << "\n";
        float scale = CLAMP((5-std::sqrt(point.z()))/5.f,0.1f,1.f);
        mTextDrawer.Draw(text,
                         point.x(),point.y(), float(windowWidth), float(windowHeight),scale, //TODO: add a controllabel scale here
                         {color.x(),color.y(),color.z()});
    }
}