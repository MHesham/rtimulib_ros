// RTIMULib ROS Node
// Copyright (c) 2015, Romain Reignier
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

static const double G_TO_MPSS = 9.80665;
static const double uT_TO_T = 1000000.0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtimulib_node");
  ROS_INFO("Imu driver is now running");
  ros::NodeHandle n;
  ros::NodeHandle private_n("~");
  const uint32_t queue_depth = 1;

  bool publish_data;
  if (!private_n.getParam("publish_data", publish_data))
  {
    ROS_WARN("No publish_data provided - default: true");
    publish_data = true;
  }

  ros::Publisher data_pub;
  if (publish_data)
  {
    std::string data_topic_name;
    if (!private_n.getParam("data_topic_name", data_topic_name))
    {
      ROS_WARN("No data_topic_name provided - default: data");
      data_topic_name = "data";
    }

    data_pub =
        n.advertise<sensor_msgs::Imu>(data_topic_name.c_str(), queue_depth);
  }

  bool publish_data_raw;
  if (!private_n.getParam("publish_data_raw", publish_data_raw))
  {
    ROS_WARN("No publish_data provided - default: true");
    publish_data_raw = true;
  }

  ros::Publisher data_raw_pub;
  if (publish_data_raw)
  {
    std::string data_raw_topic_name;
    if (!private_n.getParam("data_raw_topic_name", data_raw_topic_name))
    {
      ROS_WARN("No data_raw_topic_name provided - default: data_raw");
      data_raw_topic_name = "data_raw";
    }

    data_raw_pub =
        n.advertise<sensor_msgs::Imu>(data_raw_topic_name.c_str(), queue_depth);
  }

  bool publish_mag;
  if (!private_n.getParam("publish_mag", publish_mag))
  {
    ROS_WARN("No publish_mag provided - default: false");
    publish_mag = false;
  }

  ros::Publisher mag_pub;
  if (publish_mag)
  {
    std::string mag_topic_name;
    if (!private_n.getParam("mag_topic_name", mag_topic_name))
    {
      ROS_WARN("No mag_topic_name provided - default: mag");
      mag_topic_name = "mag";
    }

    mag_pub = n.advertise<sensor_msgs::MagneticField>(mag_topic_name.c_str(),
                                                      queue_depth);
  }

  bool publish_euler;
  if (!private_n.getParam("publish_euler", publish_euler))
  {
    ROS_WARN("No publish_euler provided - default: false");
    publish_euler = false;
  }

  ros::Publisher euler_pub;
  if (publish_euler)
  {
    std::string euler_topic_name;
    if (!private_n.getParam("euler_topic_name", euler_topic_name))
    {
      ROS_WARN("No euler_topic_name provided - default: euler");
      euler_topic_name = "euler";
    }

    euler_pub = n.advertise<geometry_msgs::Vector3>(euler_topic_name.c_str(),
                                                    queue_depth);
  }

  std::string calibration_file_path;
  if (!private_n.getParam("calibration_file_path", calibration_file_path))
  {
    ROS_ERROR("The calibration_file_path parameter must be set to use a "
              "calibration file.");
    ROS_BREAK();
  }

  std::string calibration_file_name;
  if (!private_n.getParam("calibration_file_name", calibration_file_name))
  {
    ROS_WARN("No calibration_file_name provided - default: RTIMULib.ini");
    calibration_file_name = "RTIMULib";
  }

  std::string frame_id;
  if (!private_n.getParam("frame_id", frame_id))
  {
    ROS_WARN("No frame_id provided - default: imu_link");
    frame_id = "imu_link";
  }

  // Load the RTIMULib.ini config file
  RTIMUSettings* settings = new RTIMUSettings(calibration_file_path.c_str(),
                                              calibration_file_name.c_str());

  RTIMU* imu = RTIMU::createIMU(settings);

  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
  {
    ROS_ERROR("No Imu found");
    ROS_BREAK();
  }

  // Initialise the imu object
  if (!imu->IMUInit())
  {
    ROS_FATAL("Failed to init the IMU");
    ROS_BREAK();
  }

  // Set the Fusion coefficient
  imu->setSlerpPower(0.02);
  // Enable the sensors
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(true);

  double update_rate;
  if (!private_n.getParam("update_rate", update_rate))
  {
    update_rate = 1000.0 / (double)imu->IMUGetPollInterval();
    ROS_WARN(
        "No update_rate provided - default to IMU recommended poll rate: %f Hz",
        update_rate);
  }

  ROS_INFO("IMU Poll Rate: %f Hz", update_rate);

  ros::Rate loop_rate(update_rate);
  ros::Time current_time;
  sensor_msgs::Imu data_msg;
  sensor_msgs::Imu data_raw_msg;
  sensor_msgs::MagneticField mag_msg;
  geometry_msgs::Vector3 euler_msg;

  data_msg.header.frame_id = frame_id;
  data_raw_msg.header.frame_id = frame_id;
  mag_msg.header.frame_id = frame_id;

  while (ros::ok())
  {
    if (imu->IMURead())
    {
      current_time = ros::Time::now();

      RTIMU_DATA imu_data = imu->getIMUData();

      if (publish_data)
      {
        data_msg.header.stamp = current_time;

        data_msg.orientation.x = imu_data.fusionQPose.x();
        data_msg.orientation.y = imu_data.fusionQPose.y();
        data_msg.orientation.z = imu_data.fusionQPose.z();
        data_msg.orientation.w = imu_data.fusionQPose.scalar();

        data_msg.angular_velocity.x = imu_data.gyro.x();
        data_msg.angular_velocity.y = imu_data.gyro.y();
        data_msg.angular_velocity.z = imu_data.gyro.z();

        // Convert linear acceleration from G to m/s^2
        data_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
        data_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
        data_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

        data_pub.publish(data_msg);
      }

      if (publish_data_raw)
      {
        data_raw_msg.header.stamp = current_time;

        data_raw_msg.angular_velocity.x = imu_data.gyro.x();
        data_raw_msg.angular_velocity.y = imu_data.gyro.y();
        data_raw_msg.angular_velocity.z = imu_data.gyro.z();

        // Convert linear acceleration from G to m/s^2
        data_raw_msg.linear_acceleration.x = imu_data.accel.x() * G_TO_MPSS;
        data_raw_msg.linear_acceleration.y = imu_data.accel.y() * G_TO_MPSS;
        data_raw_msg.linear_acceleration.z = imu_data.accel.z() * G_TO_MPSS;

        data_raw_pub.publish(data_raw_msg);
      }

      if (publish_mag && imu_data.compassValid)
      {
        mag_msg.header.stamp = current_time;

        // Convert linear acceleration from uT to T
        mag_msg.magnetic_field.x = imu_data.compass.x() / uT_TO_T;
        mag_msg.magnetic_field.y = imu_data.compass.y() / uT_TO_T;
        mag_msg.magnetic_field.z = imu_data.compass.z() / uT_TO_T;

        mag_pub.publish(mag_msg);
      }

      if (publish_euler)
      {
        euler_msg.x = imu_data.fusionPose.x() * RTMATH_RAD_TO_DEGREE;
        euler_msg.y = imu_data.fusionPose.y() * RTMATH_RAD_TO_DEGREE;
        euler_msg.z = imu_data.fusionPose.z() * RTMATH_RAD_TO_DEGREE;
        euler_pub.publish(euler_msg);
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
