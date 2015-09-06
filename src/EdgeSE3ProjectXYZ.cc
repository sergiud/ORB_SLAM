#include "EdgeSE3ProjectXYZ.h"

namespace g2o {
namespace {
    Eigen::Vector2d project2d(const Eigen::Vector3d& v) {
        Eigen::Vector2d res;
        res(0) = v(0) / v(2);
        res(1) = v(1) / v(2);
        return res;
    }

    Eigen::Vector3d unproject2d(const Eigen::Vector2d& v) {
        Eigen::Vector3d res;
        res(0) = v(0);
        res(1) = v(1);
        res(2) = 1;
        return res;
    }
}

    EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZ::read(std::istream& is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

        for (int i = 0; i < 2; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZ::linearizeOplus() {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;

        Eigen::Matrix<double, 2, 3> tmp;
        tmp(0, 0) = fx;
        tmp(0, 1) = 0;
        tmp(0, 2) = -x / z*fx;

        tmp(1, 0) = 0;
        tmp(1, 1) = fy;
        tmp(1, 2) = -y / z*fy;

        _jacobianOplusXi = -1. / z * tmp * T.rotation().toRotationMatrix();

        _jacobianOplusXj(0, 0) = x*y / z_2 *fx;
        _jacobianOplusXj(0, 1) = -(1 + (x*x / z_2)) *fx;
        _jacobianOplusXj(0, 2) = y / z *fx;
        _jacobianOplusXj(0, 3) = -1. / z *fx;
        _jacobianOplusXj(0, 4) = 0;
        _jacobianOplusXj(0, 5) = x / z_2 *fx;

        _jacobianOplusXj(1, 0) = (1 + y*y / z_2) *fy;
        _jacobianOplusXj(1, 1) = -x*y / z_2 *fy;
        _jacobianOplusXj(1, 2) = -x / z *fy;
        _jacobianOplusXj(1, 3) = 0;
        _jacobianOplusXj(1, 4) = -1. / z *fy;
        _jacobianOplusXj(1, 5) = y / z_2 *fy;
    }

    Eigen::Vector2d EdgeSE3ProjectXYZ::cam_project(const Eigen::Vector3d & trans_xyz) const {
        Eigen::Vector2d proj = project2d(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0] * fx + cx;
        res[1] = proj[1] * fy + cy;
        return res;
    }

} // namespace g2o