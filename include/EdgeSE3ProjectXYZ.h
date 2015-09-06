#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/se3_ops.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Geometry>

#include "ORB_SLAM_export.h"

namespace g2o {
class ORB_SLAM_EXPORT EdgeSE3ProjectXYZ : public  BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        _error = obs - cam_project(v1->estimate().map(v2->estimate()));
    }

    bool isDepthPositive() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        return (v1->estimate().map(v2->estimate()))(2) > 0.0;
    }


    virtual void linearizeOplus();

    Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

    double fx, fy, cx, cy;
};

} // namespace g2o