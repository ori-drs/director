#include <memory>

#include <Eigen/Dense>

#include <vtkPolyData.h>
#include <vtkRenderer.h>

#include <QList>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>

#include <urdf/model.h>

const int SPACE_DIMENSION = 3;
const int TWIST_SIZE = 6;

class RigidBody;
template <typename Scalar>
// KinematicsCache is a collection of data used to do forward kinematics on a RigidBodyTree
class KinematicsCache {
private:
  Eigen::VectorXd jointPositions_;
  QList<QString> jointNames_;
  Eigen::Isometry3d baseTransform_;
  std::map<std::string, Eigen::Isometry3d > linksTransformations_;

public:
  KinematicsCache(const std::vector<std::shared_ptr<RigidBody> > & bodies)
  {
  }
  void initialize(const Eigen::VectorXd& q) {
    jointPositions_ = q;
  }

  const Eigen::VectorXd& getJointPositions() const {
    return jointPositions_;
  }

  const QList<QString>& getJointNames() const {
    return jointNames_;
  }

  void setJointNames(const QList<QString>& names) {
    jointNames_ = names;
  }

  void setBaseTransform(const Eigen::Isometry3d& transform) {
    baseTransform_ = transform;
  }
  const Eigen::Isometry3d& getBaseTransform() const {
    return baseTransform_;
  }

  std::map<std::string, Eigen::Isometry3d >& getLinksTransformations() {
    return linksTransformations_;
  }

  const std::map<std::string, Eigen::Isometry3d >& getLinksTransformations() const {
    return linksTransformations_;
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
    return name_;
  }
  int getNumPositions() const {
    return num_positions_;
  }
  int getNumVelocities() const {
    return num_velocities_;
  }

protected:
  std::string name_;
  int num_positions_;
  int num_velocities_;

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

  //this method is not used
  template<typename Scalar>
  Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobian(const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, bool in_terms_of_qdot = false, std::vector<int>* v_indices = nullptr) const {
    Eigen::MatrixXd jac;
    jac.setZero(TWIST_SIZE, 6);
    return jac;
  }

  //return the transformation of the link given by body_or_frame_ind
  template<typename Scalar>
  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> relativeTransform(const KinematicsCache<Scalar>& cache, int base_or_frame_ind, int body_or_frame_ind) const {
    // Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> = Isometry3d
    const Eigen::Isometry3d& baseTransform = cache.getBaseTransform();
    Eigen::Vector3d baseTranslation = baseTransform.matrix().block<3, 1>(0, 3);
    Eigen::Matrix3d baseRotation = baseTransform.matrix().block<3, 3>(0, 0);

    const std::map<std::string, Eigen::Isometry3d >& linksTransformation = cache.getLinksTransformations();
    if (body_or_frame_ind >= 0 && body_or_frame_ind < bodies.size()) {
      if (linksTransformation.find(bodies.at(body_or_frame_ind)->linkname) != linksTransformation.end()) {
        Eigen::Isometry3d trans = linksTransformation.at(bodies.at(body_or_frame_ind)->linkname);
        trans.prerotate(baseRotation);
        trans.pretranslate(baseTranslation);
        return trans;
      }
    }
    return baseTransform;
  }

  //do the forward kinematics of all the links of the body given the joints positions stored in the cache
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
    std::map<std::string, Eigen::Isometry3d >& linksTransformation = cache.getLinksTransformations();
    for (auto link_pos : cartpos_out) {
      linksTransformation[link_pos.first] = KDLToEigen(link_pos.second);
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
  //build a Drake geometry form a urdf Geometry
  std::shared_ptr<DrakeShapes::Geometry> getGeometry(boost::shared_ptr<urdf::Geometry> &urdf_geometry);
  KDL::Tree my_tree_;
  urdf::Model my_model_;

  // Convert a vector (roll, pitch, yaw) into a rotation matrix
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
    : T_element_to_local_(T_element_to_local), material_(Eigen::Vector4d(0.7, 0.7, 0.7, 1)) {};

  VisualElement(const std::shared_ptr<Geometry>& geometry,
                const Eigen::Isometry3d& T_element_to_local,
                const Eigen::Vector4d& material)
    : T_element_to_local_(T_element_to_local), geometry_(geometry), material_(material) {};

  virtual ~VisualElement(){};

  const Eigen::Vector4d& getMaterial() const { return material_;}

  const Shape getShape() const;

  const Geometry& getGeometry() const;
  const Eigen::Isometry3d& getLocalTransform() const {
    return T_element_to_local_;
  }


protected:
  Eigen::Vector4d material_;
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

  double scale;
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



