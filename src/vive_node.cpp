#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include "vive_ros/vr_interface.h"

void handleDebugMessages(const std::string &msg) { ROS_DEBUG(" [VIVE] %s", msg.c_str()); }
void handleInfoMessages(const std::string &msg) { ROS_INFO(" [VIVE] %s", msg.c_str()); }
void handleErrorMessages(const std::string &msg) { ROS_ERROR(" [VIVE] %s", msg.c_str()); }

// #define USE_IMAGE

// #define USE_OPENGL
#define USE_VULKAN

// import from opengl sample
std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
  if (unRequiredBufferLen == 0)
    return "";

  char *pchBuffer = new char[unRequiredBufferLen];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
  std::string sResult = pchBuffer;
  delete[] pchBuffer;
  return sResult;
}

class VIVEnode
{
public:
  VIVEnode(int rate);
  ~VIVEnode();
  bool Init();
  void Run();
  void Shutdown();
  bool setOriginCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void set_feedback(sensor_msgs::JoyFeedbackConstPtr msg);
  ros::NodeHandle nh_;
  VRInterface vr_;

private:
  ros::Rate loop_rate_;
  std::vector<double> world_offset_;
  double world_yaw_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  ros::ServiceServer set_origin_server_;
  std::map<std::string, ros::Publisher> button_states_pubs_map;
  std::map<std::string, ros::Publisher> controller_states_pubs_map;
  ros::Subscriber feedback_sub_;
  ros::Publisher hmd_pub_;
  ros::Publisher controller_pose_pub_;
  ros::Publisher controller_vel_pub_;
};

VIVEnode::VIVEnode(int rate)
    : loop_rate_(rate), nh_(), tf_broadcaster_(), tf_listener_(), vr_(), world_offset_({0, 0, 0}), world_yaw_(0)
{
  nh_.getParam("/vive/world_offset", world_offset_);
  nh_.getParam("/vive/world_yaw", world_yaw_);
  ROS_INFO(" [VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
  set_origin_server_ = nh_.advertiseService("/vive/set_origin", &VIVEnode::setOriginCB, this);
  feedback_sub_ = nh_.subscribe("/vive/set_feedback", 10, &VIVEnode::set_feedback, this);
  hmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/vive/hmd_pose", 1);
  return;
}

VIVEnode::~VIVEnode()
{
  return;
}

bool VIVEnode::Init()
{
  //  Set logging functions
  vr_.setDebugMsgCallback(handleDebugMessages);
  vr_.setInfoMsgCallback(handleInfoMessages);
  vr_.setErrorMsgCallback(handleErrorMessages);

  if (!vr_.Init())
  {
    return false;
  }

  return true;
}

void VIVEnode::Shutdown()
{
  vr_.Shutdown();
}

bool VIVEnode::setOriginCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  double tf_matrix[3][4];
  int index = 1, dev_type;
  while (dev_type != 2)
  {
    dev_type = vr_.GetDeviceMatrix(index++, tf_matrix);
  }
  if (dev_type == 0)
  {
    ROS_WARN(" [VIVE] Coulnd't find controller 1.");
    return false;
  }

  tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                           tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                           tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
  tf::Vector3 c_z;
  c_z = rot_matrix * tf::Vector3(0, 0, 1);
  c_z[1] = 0;
  c_z.normalize();
  double new_yaw = acos(tf::Vector3(0, 0, 1).dot(c_z)) + M_PI_2;
  if (c_z[0] < 0)
    new_yaw = -new_yaw;
  world_yaw_ = -new_yaw;

  tf::Vector3 new_offset;
  tf::Matrix3x3 new_rot;
  new_rot.setRPY(0, 0, world_yaw_);
  new_offset = new_rot * tf::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

  world_offset_[0] = new_offset[0];
  world_offset_[1] = new_offset[1];
  world_offset_[2] = new_offset[2];

  nh_.setParam("/vive/world_offset", world_offset_);
  nh_.setParam("/vive/world_yaw", world_yaw_);
  ROS_INFO(" [VIVE] New world offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);

  return true;
}

void VIVEnode::set_feedback(sensor_msgs::JoyFeedbackConstPtr msg)
{
  if (msg->type == 1 /* TYPE_RUMBLE */)
  {
    vr_.TriggerHapticPulse(msg->id, 0, (int)(msg->intensity));
    for (int i = 0; i < 16; i++)
      vr_.TriggerHapticPulse(i, 0, (int)(msg->intensity));
  }
}

void VIVEnode::Run()
{
  double tf_matrix[3][4];
  double linear_velocity[3], angular_velocity[3];

  int run_hz_count = 0;

  while (ros::ok())
  {
    // do stuff
    vr_.Update();

    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
    {
      int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);

      // No device
      if (dev_type == 0)
        continue;

      tf::Transform tf;
      geometry_msgs::PoseStamped msg;
      tf.setOrigin(tf::Vector3(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]));

      msg.pose.position.x = tf_matrix[0][3];
      msg.pose.position.y = tf_matrix[1][3];
      msg.pose.position.z = tf_matrix[2][3];

      tf::Quaternion quat;
      tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                               tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                               tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

      rot_matrix.getRotation(quat);
      tf.setRotation(quat);

      msg.pose.orientation.x = quat.x();
      msg.pose.orientation.y = quat.y();
      msg.pose.orientation.z = quat.z();
      msg.pose.orientation.w = quat.w();
      msg.header.frame_id = "world_vive";
      msg.header.stamp = ros::Time::now();
      // get device serial number
      std::string cur_sn = GetTrackedDeviceString(vr_.pHMD_, i, vr::Prop_SerialNumber_String);
      std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');

      // It's a HMD
      if (dev_type == 1)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "hmd"));
        hmd_pub_.publish(msg);
      }
      // It's a controller
      if (dev_type == 2)
      {
        // Create publishers for each detected controller
        if (controller_states_pubs_map.count(cur_sn) == 0)
        {
          controller_states_pubs_map[cur_sn + "_pose"] = nh_.advertise<geometry_msgs::PoseStamped>("/vive/controller_" + cur_sn + "/pose", 10);
          controller_states_pubs_map[cur_sn + "_twist"] = nh_.advertise<geometry_msgs::Twist>("/vive/controller_" + cur_sn + "/twist", 10);
          controller_states_pubs_map[cur_sn] = nh_.advertise<std_msgs::Int8>("/vive/controller_" + cur_sn + "/id", 10);
        }

        // Publish the index of the controller to know where to send haptic feedback
        std_msgs::Int8 controller_id;
        controller_id.data = i;
        controller_states_pubs_map[cur_sn].publish(controller_id);

        // Publish controller velocity
        vr_.GetDeviceVel(i, linear_velocity, angular_velocity);
        geometry_msgs::Twist controller_twist;

        controller_twist.linear.x = linear_velocity[0];
        controller_twist.linear.y = linear_velocity[1];
        controller_twist.linear.z = linear_velocity[2];
        controller_twist.angular.x = angular_velocity[0];
        controller_twist.angular.y = angular_velocity[1];
        controller_twist.angular.z = angular_velocity[2];

        controller_states_pubs_map[cur_sn + "_twist"].publish(controller_twist);

        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "controller_" + cur_sn));

        // Temporary, need to do some other way
        tf::Quaternion quat_correct(0.687, -0.679, -0.200, -0.162), quat_baselink(0.0, 0.0, 0.0, 1.0);
        tf::Transform tf_correct, tf_baselink;
        tf_correct.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf_correct.setRotation(quat_correct);
        tf_baselink.setOrigin(tf::Vector3(0.24, 0, -0.50)); // z = -0.745 for the other controller
        tf_baselink.setRotation(quat_baselink);
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_correct, ros::Time::now(), "controller_" + cur_sn, "controller_" + cur_sn + '_'));
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf_baselink, ros::Time::now(), "controller_" + cur_sn + '_', "base_link_controller_" + cur_sn));

        geometry_msgs::PoseStamped msg_correct, out_pose;
        msg_correct.pose.orientation.w = 1;
        msg_correct.header.frame_id = "base_link_controller_" + cur_sn;
        msg_correct.header.stamp = ros::Time::now();

        out_pose.header.frame_id = "map";
        out_pose.header.stamp = ros::Time::now();
        out_pose.pose.position.z = 0.0;

        // Convert pose to target frame (map)
        try
        {
          tf_listener_.transformPose("map", ros::Time(0), msg_correct, msg_correct.header.frame_id, out_pose);
        }
        catch (tf::TransformException ex)
        {
          ROS_WARN("%s", ex.what());
          loop_rate_.sleep();
          continue;
        }

        // Transform from geometry_msg quaternion to tf quaternion
        tf::Quaternion q_out;
        tf::quaternionMsgToTF(out_pose.pose.orientation, q_out);
        double roll, pitch, yaw;
        // Get euler angles from quaternion
        tf::Matrix3x3(q_out).getRPY(roll, pitch, yaw);
        q_out.setRPY(0.0, 0.0, yaw);
        // Transform from tf quaternion to geometry msgs quaternion
        tf::quaternionTFToMsg(q_out, out_pose.pose.orientation);

        controller_states_pubs_map[cur_sn + "_pose"].publish(out_pose);
        // Temporary, need to do some other way

        vr::VRControllerState_t state;
        vr_.HandleInput(i, state);
        sensor_msgs::Joy joy;
        joy.header.stamp = ros::Time::now();
        joy.header.frame_id = "controller_" + cur_sn;
        joy.buttons.assign(BUTTON_NUM, 0);
        joy.axes.assign(AXES_NUM, 0.0); // x-axis, y-axis
        if ((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
          joy.buttons[0] = 1;
        if ((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
          joy.buttons[1] = 1;
        if ((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
          joy.buttons[2] = 1;
        if ((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
          joy.buttons[3] = 1;
        // TrackPad's axis
        joy.axes[0] = state.rAxis[0].x;
        joy.axes[1] = state.rAxis[0].y;
        // Trigger's axis
        joy.axes[2] = state.rAxis[1].x;
        //        #include <bitset> // bit debug
        //        std::cout << static_cast<std::bitset<64> >(state.ulButtonPressed) << std::endl;
        //        std::cout << static_cast<std::bitset<64> >(state.ulButtonTouched) << std::endl;
        if (button_states_pubs_map.count(cur_sn) == 0)
        {
          button_states_pubs_map[cur_sn] = nh_.advertise<sensor_msgs::Joy>("/vive/controller_" + cur_sn + "/joy", 10);
        }
        button_states_pubs_map[cur_sn].publish(joy);
      }
      // It's a tracker
      if (dev_type == 3)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "tracker_" + cur_sn));
      }
      // It's a lighthouse
      if (dev_type == 4)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "lighthouse_" + cur_sn));
      }
    }

    // Publish corrective transform
    tf::Transform tf_world;
    tf_world.setOrigin(tf::Vector3(world_offset_[0], world_offset_[1], world_offset_[2]));
    tf::Quaternion quat_world;
    quat_world.setRPY(M_PI / 2, 0, world_yaw_);
    tf_world.setRotation(quat_world);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_world, ros::Time::now(), "world", "world_vive"));

    ROS_DEBUG_THROTTLE(1.0, "Run() @ %d [fps]", [](int &cin)
                       {int ans = cin; cin=0; return ans; }(run_hz_count));
    run_hz_count++;
    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vive_node");

  VIVEnode nodeApp(100); // old rate : 1000, changed due to cpu overload

  if (!nodeApp.Init())
  {
    nodeApp.Shutdown();
    return 1;
  }

  nodeApp.Run();
  nodeApp.Shutdown();

  return 0;
};
