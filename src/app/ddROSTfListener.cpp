#include "ddROSTfListener.h"

std::shared_ptr<tf2_ros::TransformListener> ddROSTfListener::mTfListener = nullptr;
std::shared_ptr<tf2_ros::Buffer> ddROSTfListener::mTfBuffer = nullptr;
