#include <Eigen/Dense>
#include <memory>

#include <vtkPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkCubeSource.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkStringArray.h>
#include <vtkFieldData.h>
#include <vtkMath.h>
#include <vtkProperty.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLMultiBlockDataReader.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkJPEGReader.h>
#include <vtkPNGReader.h>
#include <vtkImageData.h>

#include <QList>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>

#include <urdf/model.h>


const int SPACE_DIMENSION = 3;
const int TWIST_SIZE = 6;

class RigidBody;
template <typename Scalar>
class KinematicsCache {
private:
  Eigen::VectorXd jointPositions;
  QList<QString> jointNames;
  Eigen::Isometry3d baseTransform;

public:
  KinematicsCache(const std::vector<std::shared_ptr<RigidBody> > & bodies)
  {
  }
  void initialize(const Eigen::VectorXd& q) {
    jointPositions = q;
  }

  const Eigen::VectorXd& getJointPositions() const {
    return jointPositions;
  }

  const QList<QString>& getJointNames() const {
    return jointNames;
  }

  void setJointNames(const QList<QString>& names) {
    jointNames = names;
  }

  void setBaseTransform(const Eigen::Isometry3d& transform) {
    baseTransform = transform;
  }
  const Eigen::Isometry3d& getBaseTransform() const {
    return baseTransform;
  }

};

class DrakeJoint {

public:
  enum FloatingBaseType {
    FIXED        = 0,
    ROLLPITCHYAW = 1,
    QUATERNION   = 2
  };
  DrakeJoint(const std::string& name_joint, int joint_type);
  DrakeJoint(const std::string& name_joint, FloatingBaseType joint_type);

  const std::string getName() const {
    return name;
  }
  int getNumPositions() const {
    return num_positions;
  }
  int getNumVelocities() const {
    return num_velocities;
  }

protected:
  std::string name;
  int num_positions;
  int num_velocities;

};

namespace DrakeShapes {
class Geometry;
}

namespace DrakeShapes {
class VisualElement;
}
class RigidBody {
public:
  RigidBody();
  bool hasParent() const;
  const DrakeJoint& getJoint() const;
  int getPositionNumStart() const;

  Eigen::Matrix3Xd contact_pts;

  std::shared_ptr<RigidBody> parent;

  int position_num_start;
  int velocity_num_start;
  std::vector< DrakeShapes::VisualElement, Eigen::aligned_allocator<DrakeShapes::VisualElement> > visual_elements;
  int body_index; // position in vector of bodies
  std::string linkname;
  std::unique_ptr<DrakeJoint> joint;
  double mass;
  Eigen::Vector3d com;
};

class RigidBodyTree {
public:

  // load urdf
  void addRobotFromURDFString(const std::string &xml_string, std::map<std::string,std::string>& package_map, const std::string &root_dir = ".",
                              const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);

  template<typename Scalar>
  Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobian(const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, bool in_terms_of_qdot = false, std::vector<int>* v_indices = nullptr) const {
    Eigen::MatrixXd jac;
    jac.setZero(TWIST_SIZE, 6);
    /*boost::shared_ptr<KDL::TreeJntToJacSolver> jacsolver = boost::shared_ptr<KDL::TreeJntToJacSolver>(new KDL::TreeJntToJacSolver(my_tree_));
    KDL::Jacobian jacobian;
    if (end_effector_body_or_frame_ind < 0 || end_effector_body_or_frame_ind >= bodies.size()) {
      return jac;
    }
    KDL::JntArray q_in;
    q_in.resize(my_tree_.getNrOfJoints());
    const Eigen::VectorXd jointPositions = cache.getJointPositions();
    const QList<QString>& jointNames = cache.getJointNames();
    if (jointPositions.size() != jointNames.size())
    {
      std::cout << "RigidBodyTree::geometricJacobian(): jointPositions size "
                << jointPositions.size() << " != " << jointNames.size() << std::endl;
      return jac;
    }

    //std::string segmentName = bodies[end_effector_body_or_frame_ind]->linkname;
    std::string segmentName = "LF_SHANK";
    //filling q_in
    int count = 0;
    for(auto& segment : my_tree_.getSegments()) {
      double position;
      for(int i = 0; i < jointNames.size(); ++i) {
        std::string jointName = segment.second.segment.getJoint().getName();
        if (jointNames[i].toUtf8().data() == jointName) {
          position = jointPositions[i];
          break;
        }
      }
      q_in.data(count) = position;
      ++count;
    }
    int solver_status = jacsolver->JntToJac(q_in, jacobian, segmentName);
    std::cout << "jacobian " << jacobian.data << std::endl;
    std::cout << "End " << std::endl;*/
    return jac;
  }

  //transformation between base frame and body frame ( frame of the robot)
  template<typename Scalar>
  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> relativeTransform(const KinematicsCache<Scalar>& cache, int base_or_frame_ind, int body_or_frame_ind) const {
    // Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> = Isometry3d
    Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> tf_out;
    tf_out.setIdentity();
    const Eigen::Isometry3d baseTransform = cache.getBaseTransform();
    Eigen::Vector3d baseTranslation = baseTransform.matrix().block<3, 1>(0, 3);
    Eigen::Matrix3d baseRotation = baseTransform.matrix().block<3, 3>(0, 0);
    tf_out.matrix() << baseRotation, baseTranslation, 0, 0, 0, 1;
    if (body_or_frame_ind >= 0 && body_or_frame_ind < bodies.size()) {
      if (links_pos.find(bodies.at(body_or_frame_ind)->linkname) != links_pos.end()) {
        Eigen::Isometry3d trans = links_pos.at(bodies.at(body_or_frame_ind)->linkname);
        trans.prerotate(baseRotation);
        trans.pretranslate(baseTranslation);
        return trans;
      }
    }
    return tf_out;
  }

  template <typename Scalar>
  void doKinematics(KinematicsCache<Scalar>& cache, bool compute_JdotV = false) {
    std::map<std::string, double> jointpos_in;
    const Eigen::VectorXd jointPositions = cache.getJointPositions();
    const QList<QString>& jointNames = cache.getJointNames();
    if (jointPositions.size() != jointNames.size())
    {
      std::cout << "RigidBodyTree::doKinematics(): jointPositions size "
                << jointPositions.size() << " != " << jointNames.size() << std::endl;
      return;
    }
    for(int i = 0; i < jointPositions.rows(); ++i) {
        jointpos_in.insert(std::make_pair(jointNames[i].toUtf8().data(), jointPositions(i)));
    }

    bool kinematics_status;
    std::map<std::string, KDL::Frame > cartpos_out;
    bool flatten_tree = true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver = boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive>(new KDL::TreeFkSolverPosFull_recursive(my_tree_));
    kinematics_status = fksolver->JntToCart(jointpos_in, cartpos_out, flatten_tree);
    for (auto link_pos : cartpos_out) {
      links_pos[link_pos.first] = KDLToEigen(link_pos.second);
    }
    // set base transform
    Eigen::Vector3d translation, rpy;
    Eigen::Matrix3d rotation;
    for (int i = 0; i < jointNames.size(); ++i) {
      if (jointNames[i] == "base_x") {
        translation(0) = jointPositions[i];
      } else if (jointNames[i] == "base_y") {
        translation(1) = jointPositions[i];
      } else if (jointNames[i] == "base_z") {
        translation(2) = jointPositions[i];
      } else if (jointNames[i] == "base_roll") {
        rpy(0) = jointPositions[i];
      } else if (jointNames[i] == "base_pitch") {
        rpy(1) = jointPositions[i];
      } else if (jointNames[i] == "base_yaw") {
        rpy(2) = jointPositions[i];
      }
    }
    rotation = rpy2rotmat(rpy);
    Eigen::Isometry3d transform;
    transform.setIdentity();
    transform.matrix() << rotation, translation, 0, 0, 0, 1;
    cache.setBaseTransform(transform);
  }

  Eigen::VectorXd joint_limit_min;
  Eigen::VectorXd joint_limit_max;

  //return position of linkname in vector bodies
  int findLinkId(const std::string& linkname, int robot = -1) const;
  std::string getBodyOrFrameName(int id);

  template <typename Scalar>
  Eigen::Matrix<Scalar, SPACE_DIMENSION, 1> centerOfMass(KinematicsCache<Scalar> &cache) {
    Eigen::Matrix<Scalar, SPACE_DIMENSION, 1> m;
    m.setZero();
    double sumMass = 0.;
    for (int i = 0; i < bodies.size(); ++i) {
      Eigen::Isometry3d transform = relativeTransform(cache, 0, bodies[i]->body_index);
      Eigen::Vector3d translation = transform.matrix().block<3, 1>(0, 3);
      if (bodies[i]->mass > 0) {
        sumMass += bodies[i]->mass;
        m += bodies[i]->mass * (transform * bodies[i]->com);
      }
    }
    if (sumMass > 0)
      m /= sumMass;
    return m;
  }
  std::shared_ptr<RigidBody> findLink(std::string linkname, int robot=-1) const;

  // Rigid body objects
  std::vector<std::shared_ptr<RigidBody> > bodies;
  int num_positions; // sum of num_positions of all joints
  int num_velocities;

private:
  std::shared_ptr<DrakeShapes::Geometry> getGeometry(boost::shared_ptr<urdf::Geometry> &urdf_geometry);
  KDL::Tree my_tree_;
  urdf::Model my_model_;
  std::map<std::string, Eigen::Isometry3d > links_pos; //TODO move that in KinematicsCache

  template<typename Derived>
  Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(const Eigen::MatrixBase<Derived>& rpy) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
    auto rpy_array = rpy.array();
    auto s = rpy_array.sin();
    auto c = rpy_array.cos();

    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    R.row(0) << c(2) * c(1), c(2) * s(1) * s(0) - s(2) * c(0), c(2) * s(1) * c(0) + s(2) * s(0);
    R.row(1) << s(2) * c(1), s(2) * s(1) * s(0) + c(2) * c(0), s(2) * s(1) * c(0) - c(2) * s(0);
    R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);

    return R;
  };

  static Eigen::Isometry3d KDLToEigen(const KDL::Frame& tf);
};

namespace DrakeShapes
{
   enum Shape {
    UNKNOWN     = 0,
    BOX         = 1,
    SPHERE      = 2,
    CYLINDER    = 3,
    MESH        = 4,
    MESH_POINTS = 5,
    CAPSULE     = 6
  };

  const double MIN_RADIUS = 1e-7;
  class  Geometry {
    public:
      Geometry();
      Geometry(const Geometry& other);
      virtual ~Geometry() {}

      const Shape getShape() const {
        return shape;
      }

    protected:
      Geometry(Shape shape);
      Shape shape;  
  };

  class VisualElement {
  public:
      VisualElement(const Eigen::Isometry3d& T_element_to_local)
    : T_element_to_local_(T_element_to_local), material(Eigen::Vector4d(0.7, 0.7, 0.7, 1)) {};

      VisualElement(const std::shared_ptr<Geometry>& geometry,
                    const Eigen::Isometry3d& T_element_to_local,
                    const Eigen::Vector4d& material)
    : T_element_to_local_(T_element_to_local), geometry_(geometry), material(material) {};

      virtual ~VisualElement(){};

      const Eigen::Vector4d& getMaterial() const { return material;}

      const Shape getShape() const;

      const Geometry& getGeometry() const;
      const Eigen::Isometry3d& getLocalTransform() const {
        return T_element_to_local_;
      }


    protected:
      Eigen::Vector4d material;
      std::shared_ptr<Geometry> geometry_;
      Eigen::Isometry3d T_element_to_local_;
  };

  class Sphere: public Geometry {
    public:
      Sphere(double radius);
      virtual ~Sphere() {}
      
      double radius;
  };

  class Box : public Geometry {
    public:
      Box(const Eigen::Vector3d& size);
      virtual ~Box() {}
      Eigen::Vector3d size;
      
  };

  class  Cylinder : public Geometry {
    public:
      Cylinder(double radius, double length);
      virtual ~Cylinder() {}

      double radius;
      double length;
  };

  class Mesh : public Geometry {
    public:
      Mesh(const std::string& filename, double scale);
      virtual ~Mesh() {}

      double scale; // Eigen::Vector3d
      std::string filename;
  };

  class Capsule : public Geometry {
    public:
      Capsule(double radius, double length);
      virtual ~Capsule() {}
      double radius;
      double length;
  };

}



