//
// Created by sc on 2/5/21.
//

#ifndef RELOCALIZATION3D_SCAN3R_JSON_LOADER_H
#define RELOCALIZATION3D_SCAN3R_JSON_LOADER_H

#include <ORUtils/JsonUtil.h>
namespace PSLAM::io {
    class Scan3RLoader {
    public:
        struct MovedObject {
            explicit MovedObject(int id) : id(id) {}

            int id;
            int symmetry = 0;
            Eigen::Matrix4f transformation;
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef std::shared_ptr<MovedObject> MovedObjectPtr;

        static inline MovedObjectPtr MakeMovedObjectPtr(int id) {
            return std::make_shared<MovedObject>(id);
        }

        struct Ambiguity {
            int instance_source, instance_target;
            /// align source to target
            Eigen::Matrix4f transformation;

            Ambiguity(int source, int target) : instance_source(source), instance_target(target) {}
        };

        typedef std::shared_ptr<Ambiguity> AmbiguityPtr;

        static inline AmbiguityPtr MakeAmbiguityPtr(int source, int target) {
            return std::make_shared<Ambiguity>(source, target);
        }

        struct ScanInfo;
        typedef std::shared_ptr<ScanInfo> ScanInfoPtr;

        struct ScanInfo {
            // general
            std::string scan_id;
            std::string type;

            // as a reference scan
//            SC::DisJointSet<int> ambiguious_sets;
            std::unordered_map<int, std::vector<AmbiguityPtr>> ambiguities;
            std::unordered_map<std::string, ScanInfoPtr> rescans;

            // as a rescan
            /// align rescan its reference scan
            Eigen::Matrix4f transformation{Eigen::Matrix4f::Identity()};
            std::unordered_map<int, MovedObjectPtr> moved_rigid_objects;

            explicit ScanInfo(std::string id) : scan_id(std::move(id)) {}

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        static inline ScanInfoPtr MakeScanInfo(const std::string &scan_id) {
            return std::make_shared<ScanInfo>(scan_id);
        }

        explicit Scan3RLoader(const std::string &path) {
            Load(path);
        }

        bool IsRescan(const std::string &scan_id) {
            return reference2rescans.find(scan_id) == reference2rescans.end();
        }

        bool IsReference(const std::string &scan_id) {
            return reference2rescans.find(scan_id) != reference2rescans.end();
        }

        std::map<std::string, std::string> rescanToReference;
        std::map<std::string, std::vector<std::string> > reference2rescans;
        std::map<std::string, ScanInfoPtr> scaninfos;

    private:
        static void ReadMatrix(Eigen::Matrix4f &matrix, const json11::Json &json_matrix) {
            matrix = Eigen::Matrix4f::Identity();
            int i = 0;
            for (auto &elem: json_matrix.array_items())
                matrix(i++) = elem.number_value();
        }

        void Load(const std::string &path) {
            auto scan3r = ORUtils::JsonUtils::LoadJson(path);
            for (const auto &s : scan3r.array_items()) {

                const std::string &reference_id = s["reference"].string_value();
                const std::string &type = s["type"].string_value();

                auto &refScanInfo = scaninfos[reference_id];
                refScanInfo = MakeScanInfo(reference_id);
                refScanInfo->type = type;

                // Add ambiguity information
                const auto &ambiguities = s["ambiguity"].array_items();
                for (const auto &ambiguity : ambiguities) {
                    const auto &instance_source = ambiguity["instance_source"].int_value();
                    const auto &instance_target = ambiguity["instance_target"].int_value();
                    auto am = MakeAmbiguityPtr(instance_source, instance_target);
                    refScanInfo->ambiguities[instance_source].push_back(am);
                    ReadMatrix(am->transformation, ambiguity["transform"]);

//                    auto &set = refScanInfo->ambiguious_sets;
//                    if (!set.exist(instance_source)) set.add(instance_source);
//                    if (!set.exist(instance_target)) set.add(instance_target);
//                    set.unionSet(instance_source, instance_target);
                }


                const auto &scans = s["scans"].array_items();
                // Add rescan information
                for (auto &scan: scans) {
                    const std::string &scan_id = scan["reference"].string_value();
                    rescanToReference[scan_id] = reference_id;
                    reference2rescans[reference_id].push_back(scan_id);

                    auto scanInfo = MakeScanInfo(scan_id);
                    refScanInfo->rescans[scan_id] = scanInfo;
                    ReadMatrix(scanInfo->transformation, scan["transform"]);

                    for (const auto &rigid : scan["rigid"].array_items()) {
                        const int instance_id = rigid["instance_reference"].int_value();
                        MovedObjectPtr &movedObject = scanInfo->moved_rigid_objects[instance_id] = MakeMovedObjectPtr(
                                instance_id);
                        Eigen::Matrix4f &rigid_transform = movedObject->transformation;
                        ReadMatrix(rigid_transform, rigid["transform"]);
                        // The transformation in the json is actually from reference to rescan
                        rigid_transform = rigid_transform.inverse().eval();
                    }
                }
            }
        }
    };
}

#endif //RELOCALIZATION3D_SCAN3R_JSON_LOADER_H
