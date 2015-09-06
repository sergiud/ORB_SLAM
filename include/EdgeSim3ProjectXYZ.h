#include <g2o/types/sim3/types_seven_dof_expmap.h>

namespace g2o {

  /**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class VertexSim3ExpmapTwoCameras : public BaseVertex<7, Sim3>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexSim3ExpmapTwoCameras();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
      _estimate = Sim3();
    }

    virtual void oplusImpl(const double* update_)
    {
      Eigen::Map<Vector7d> update(const_cast<double*>(update_));

      if (_fix_scale)
        update[6] = 0;

      Sim3 s(update);
      setEstimate(s*estimate());
    }

    Eigen::Vector2d _principle_point1, _principle_point2;
    Eigen::Vector2d _focal_length1, _focal_length2;

    Eigen::Vector2d cam_map1(const Eigen::Vector2d & v) const
    {
        Eigen::Vector2d res;
      res[0] = v[0]*_focal_length1[0] + _principle_point1[0];
      res[1] = v[1]*_focal_length1[1] + _principle_point1[1];
      return res;
    }

    Eigen::Vector2d cam_map2(const Eigen::Vector2d & v) const
    {
      Eigen::Vector2d res;
      res[0] = v[0]*_focal_length2[0] + _principle_point2[0];
      res[1] = v[1]*_focal_length2[1] + _principle_point2[1];
      return res;
    }

    bool _fix_scale;


  protected:
  };

  class EdgeSim3TwoCameras : public BaseBinaryEdge<7, Sim3, VertexSim3ExpmapTwoCameras, VertexSim3ExpmapTwoCameras>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSim3TwoCameras() { }
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
    void computeError()
    {
      const VertexSim3ExpmapTwoCameras* v1 = static_cast<const VertexSim3ExpmapTwoCameras*>(_vertices[0]);
      const VertexSim3ExpmapTwoCameras* v2 = static_cast<const VertexSim3ExpmapTwoCameras*>(_vertices[1]);

      Sim3 C(_measurement);
      Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
      _error = error_.log();
    }

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
    {
      VertexSim3Expmap* v1 = static_cast<VertexSim3Expmap*>(_vertices[0]);
      VertexSim3Expmap* v2 = static_cast<VertexSim3Expmap*>(_vertices[1]);
      if (from.count(v1) > 0)
  v2->setEstimate(measurement()*v1->estimate());
      else
  v1->setEstimate(measurement().inverse()*v2->estimate());
    }
  };

/**/
class EdgeInverseSim3ProjectXYZ : public  BaseBinaryEdge<2, Eigen::Vector2d,  VertexSBAPointXYZ, VertexSim3ExpmapTwoCameras>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeInverseSim3ProjectXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    void computeError()
    {
      const VertexSim3ExpmapTwoCameras* v1 = static_cast<const VertexSim3ExpmapTwoCameras*>(_vertices[1]);
      const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

      Eigen::Vector2d obs(_measurement);
      _error = obs-v1->cam_map2(project(v1->estimate().inverse().map(v2->estimate())));
    }

   // virtual void linearizeOplus();

};

} // end namespace
