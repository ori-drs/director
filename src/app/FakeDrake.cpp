#include "FakeDrake.h"
#include <exception>

DrakeJoint::DrakeJoint(const std::string& name_joint, int joint_type)
  :name(name_joint) {
  switch(joint_type) {
  case int(urdf::Joint::REVOLUTE):
  case int(urdf::Joint::CONTINUOUS):
  case int(urdf::Joint::PRISMATIC):
  case int(urdf::Joint::PLANAR):
    num_positions = 1;
    num_velocities = 1;
    break;
  case int(urdf::Joint::FLOATING):
    num_positions = 6;
    num_velocities = 6;
    break;
  default:
    num_positions = 0;
    num_velocities = 0;
  }
}

DrakeJoint::DrakeJoint(const std::string& name_joint, FloatingBaseType joint_type)
  :name(name_joint){
  switch(joint_type) {
  case FloatingBaseType::FIXED:
    num_positions = 0;
    num_velocities = 0;
    break;
  case FloatingBaseType::ROLLPITCHYAW:
    num_positions = 6;
    num_velocities = 6;
    break;
  case FloatingBaseType::QUATERNION:
    num_positions = 7;
    num_velocities = 6;
    break;
  default:
    num_positions = 0;
    num_velocities = 0;
  }
}

RigidBody::RigidBody()
  :mass(0.){
  com.setZero();
}

void RigidBodyTree::addRobotFromURDFString(const std::string &xml_string, std::map<std::string,std::string>& package_map, const std::string &root_dir,
                                           const DrakeJoint::FloatingBaseType floating_base_type) {
  if (!my_model_.initString(xml_string)){
    ROS_ERROR("Failed to parse urdf robot model");
    return;
  }
  if (!kdl_parser::treeFromUrdfModel(my_model_, my_tree_)){
    ROS_ERROR("Failed to construct kdl tree");
    return;
  }
  // building data structures of FakeDrake
  std::vector<boost::shared_ptr<urdf::Link> > links;
  my_model_.getLinks(links);
  std::map<std::string, int> body_index;
  //put a link called world at the front of bodies
  std::shared_ptr<RigidBody> body = std::make_shared<RigidBody>();
  body->linkname = "world";
  body->body_index = 0;
  bodies.push_back(body);
  for(int i = 0; i < links.size(); ++i) {
    std::shared_ptr<RigidBody> body = std::make_shared<RigidBody>();
    body->linkname = links[i]->name;
    body->body_index = i + 1;
    body_index[body->linkname] = i + 1;
    if (links[i]->parent_joint) {
      body->joint.reset(new DrakeJoint(links[i]->parent_joint->name, links[i]->parent_joint->type));
    }
    //mass and com
    if (links[i]->inertial) {
      body->mass = links[i]->inertial->mass;
      body->com << links[i]->inertial->origin.position.x, links[i]->inertial->origin.position.y, links[i]->inertial->origin.position.z;
    }
    //visual
    if (links[i]->visual) {
      Eigen::Isometry3d T_element_to_local;
      Eigen::Vector3d rpy;
      double roll, pitch, yaw;
      links[i]->visual->origin.rotation.getRPY(roll, pitch, yaw);
      rpy << roll, pitch, yaw;
      Eigen::Vector3d xyz;
      xyz << links[i]->visual->origin.position.x, links[i]->visual->origin.position.y, links[i]->visual->origin.position.z;
      T_element_to_local.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
      Eigen::Vector4d material(Eigen::Vector4d(0.7, 0.7, 0.7, 1));
      if (links[i]->visual->material) {
        material = Eigen::Vector4d(links[i]->visual->material->color.r, links[i]->visual->material->color.g,
                                 links[i]->visual->material->color.b, links[i]->visual->material->color.a);
      }
      //geometry
      std::shared_ptr<DrakeShapes::Geometry> geometry = getGeometry(links[i]->visual->geometry);
      DrakeShapes::VisualElement visual_element(geometry, T_element_to_local, material);
      body->visual_elements.push_back(visual_element);
    }
    bodies.push_back(body);
  }
  // setting parent
  for(int i = 0; i < links.size(); ++i) {
    if (!links[i]->getParent()) {
      continue;
    }
    int parent_index = body_index[links[i]->getParent()->name];
    bodies[i+1]->parent = bodies[parent_index];
  }
  // attach the root nodes to the world with a floating base joint
  for(int i = 1; i < bodies.size(); ++i) {
    if (!bodies[i]->hasParent()) {
      bodies[i]->parent = bodies[0];
      bodies[i]->joint.reset(new DrakeJoint("base", floating_base_type));
    }
  }

  //compute joint_limit_min, joint_limit_max, num_positions, num_velocities
  num_positions = 0;
  num_velocities = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      body.position_num_start = num_positions;
      num_positions += body.getJoint().getNumPositions();
      body.velocity_num_start = num_velocities;
      num_velocities += body.getJoint().getNumVelocities();
    }
    else {
      body.position_num_start = 0;
      body.velocity_num_start = 0;
    }
  }
  joint_limit_min = Eigen::VectorXd::Constant(num_positions, -std::numeric_limits<double>::infinity());
  joint_limit_max = Eigen::VectorXd::Constant(num_positions, std::numeric_limits<double>::infinity());
}

std::shared_ptr<DrakeShapes::Geometry> RigidBodyTree::getGeometry(boost::shared_ptr<urdf::Geometry>& urdf_geometry) {
  std::shared_ptr<DrakeShapes::Geometry> geometry;
  if (!urdf_geometry) {
    return geometry;
  }
  switch(urdf_geometry->type) {
  case urdf::Geometry::SPHERE:
  {
    boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere>(urdf_geometry);
    geometry = std::make_shared<DrakeShapes::Sphere>(sphere->radius);
  }
    break;
  case urdf::Geometry::BOX:
  {
    boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box>(urdf_geometry);
    Eigen::Vector3d size;
    size << box->dim.x, box->dim.y, box->dim.z;
    geometry = std::make_shared<DrakeShapes::Box>(size);
  }
    break;
  case urdf::Geometry::CYLINDER:
  {
    boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder>(urdf_geometry);
    geometry = std::make_shared<DrakeShapes::Cylinder>(cylinder->radius, cylinder->length);
  }
    break;
  case urdf::Geometry::MESH:
  {
    boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh>(urdf_geometry);
    geometry = std::make_shared<DrakeShapes::Mesh>(mesh->filename, mesh->scale.x);
  }
    break;
  default:
    break;
  }
  return geometry;
}

 Eigen::Isometry3d RigidBodyTree::KDLToEigen(const KDL::Frame& tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  tf_out.rotate(q);
  return tf_out;
}

//return position of linkname in vector bodies
int RigidBodyTree::findLinkId(const std::string& linkname, int robot) const {
  for (auto& body : bodies) {
    if (body->linkname == linkname){
      return body->body_index;
    }
  }
  throw std::runtime_error("could not find link id: " + linkname);
}

std::string RigidBodyTree::getBodyOrFrameName(int id) {
  if (id >= 0) {
    return bodies[id]->linkname;
  } else {
    ROS_ERROR("Failed to get body id");
  }
}

std::shared_ptr<RigidBody> RigidBodyTree::findLink(std::string linkname, int robot) const {
  for (auto& link : bodies) {
    if (link->linkname == linkname) {
      return link;
    }
  }
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint) {
    return (*joint);
  }
  else {
    throw std::runtime_error("Joint is not initialized");
  }
}

bool RigidBody::hasParent() const {
  return parent != nullptr;
}

int RigidBody::getPositionNumStart() const {
  return position_num_start;
}

namespace DrakeShapes {

Geometry::Geometry(Shape shape)
  :shape(shape){

}

const Shape VisualElement::getShape() const {
  return geometry_->getShape();
}

const Geometry& VisualElement::getGeometry() const {
  return *geometry_;
}

Sphere::Sphere(double radius)
  :Geometry(SPHERE), radius(radius)
{}

Box::Box(const Eigen::Vector3d& size)
  :Geometry(BOX), size(size)
{
}

Cylinder::Cylinder(double radius, double length)
  :Geometry(CYLINDER), radius(radius), length(length)
{
}

Mesh::Mesh(const std::string& filename, double scale)
  :Geometry(MESH), filename(filename), scale(scale)
{
}

}
