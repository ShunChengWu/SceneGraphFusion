//
// Created by sc on 10/9/20.
//
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <tinyply.h>
#include <iostream>
#include "MemoryBlock.h"

#ifndef GRAPHSLAM_UTILINSEGPLYLOADER_H
#define GRAPHSLAM_UTILINSEGPLYLOADER_H
namespace Util {
    void print_headers(const tinyply::PlyFile &file){
        for (const auto &e : file.get_elements()) {
            std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")" << std::endl;
            for (const auto &p : e.properties) {
                std::cout << "\t[ply_header] \tproperty: " << p.name << " (type="
                          << tinyply::PropertyTable[p.propertyType].str << ")";
                if (p.isList) std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str << ")";
                std::cout << std::endl;
            }
        }
    }
}
#endif //GRAPHSLAM_UTILINSEGPLYLOADER_H
