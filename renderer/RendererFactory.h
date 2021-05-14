//
// Created by sc on 5/13/21.
//

#ifndef GRAPHSLAM_RENDERERFACTORY_H
#define GRAPHSLAM_RENDERERFACTORY_H
#include "Renderer.h"
#include "RendererType.h"
#include "Renderer3RScan.h"
#include "RendererScanNet.h"
namespace PSLAM {

    static MeshRendererInterface* MakeMeshRenderer(int width, int height, const std::string &folder, const std::string &scan_id,
                                                   MeshRenderType meshRenderType = MeshRenderType_Detect, bool align = true) {
        if (meshRenderType == MeshRenderType_Detect) {
            auto folder_lower = folder;
            std::for_each(folder_lower.begin(),folder_lower.end(), [](char &c) {
               c = ::tolower(c);
            });

            // find ScanNet
            if (scan_id.find("scene") != std::string::npos ||
                    folder_lower.find("scannet")  != std::string::npos) {
                meshRenderType = MeshRenderType_ScanNet;
            } else if (folder_lower.find("3rscan") != std::string ::npos) {
                meshRenderType = MeshRenderType_3RScan;
            } else {
                throw std::runtime_error("unable to detect type.");
            }
        }


        MeshRendererInterface *renderer = nullptr;
        switch (meshRenderType) {
            case MeshRenderType_ScanNet:{
                renderer = new MeshRendererScanNet(width, height, folder, scan_id, align);
            }
                break;
            case MeshRenderType_3RScan:{
                renderer = new MeshRenderer3RScan(width, height, folder, scan_id, align);
            }
                break;
            case MeshRenderType_Detect:
                throw std::runtime_error("unable to detect renderer type");
                break;
        }
        return renderer;
    }
}

#endif //GRAPHSLAM_RENDERERFACTORY_H
