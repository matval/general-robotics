#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1);
    ros::Rate loop_rate(100);

    sensor_msgs::Imu imu_msg;

    /*
    Orientation covariance estimation:
    Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
    Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
    Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
    cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
    i.e. variance in yaw: 0.0025
    Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
    static roll/pitch error of 0.8%, owing to gravity orientation sensing
    error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
    so set all covariances the same.
    */
    imu_msg.orientation_covariance = {
        0.0025 , 0 , 0,
        0, 0.0025, 0,
        0, 0, 0.0025};

    /*
    # Angular velocity covariance estimation:
    # Observed gyro noise: 4 counts => 0.28 degrees/sec
    # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
    # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
    */
    imu_msg.angular_velocity_covariance = {
        0.02, 0 , 0,
        0, 0.02, 0,
        0, 0 , 0.02};

    /*
    # linear acceleration covariance estimation:
    # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
    # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
    # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
    */
    imu_msg.linear_acceleration_covariance = {
        0.04, 0, 0,
        0, 0.04, 0,
        0, 0, 0.04};

    const char *default_port = "/dev/ttyACM0";
    // Configure Serial communication =====================
    struct termios toptions;
    int fd;

    fd = open(default_port, O_RDWR | O_NOCTTY);

    if (fd == -1)
    {
        ROS_ERROR("serialport_init: Unable to open port ");
        return -1;
    }
    else if (tcgetattr(fd, &toptions) < 0)
    {
        ROS_ERROR("serialport_init: Couldn't get term attributes");
        return -1;
    }
    else
    {
        // Set the baud rates to 115200...
        cfsetispeed(&toptions, B115200);
        cfsetospeed(&toptions, B115200);

        toptions.c_lflag |= ICANON;

        //toptions.c_lflag |= (~ICANON | ECHO | ECHOE);
        toptions.c_cc[VTIME] = 10;  // Set timeout of 1.0 second
        toptions.c_cc[VMIN] = 0;    // No min number of chars is set
        
        // Activate the settings for the port
        tcsetattr(fd, TCSANOW, &toptions);
    }

    double roll=0, pitch=0, yaw=0;
    int seq=0;

    tf::Quaternion q;

    double accel_factor = 9.806;    // sensor reports accel as 1G (9.8m/s^2). Convert to m/s^2.
    double deg2rad = M_PI/180;
    ROS_INFO("Giving the razor IMU board 5 seconds to boot...");
    ros::Duration(5).sleep();       // Sleep for 5 seconds to wait for the board to boot
    
    while (ros::ok())
    {
        char buf[255];

        int res = read(fd, &buf, sizeof(buf)-1);
        if (res > 0)
            buf[res]=0;     // Set end of string, so we can print
        else
            ROS_WARN("Line was not read properly");

        //ROS_INFO("The entire string is:\n%s", buf);

        // Split string using "," token
        std::vector<char*> values;
        char *value;

        value = strtok(buf, ",");
        values.push_back(value);
        
        //ROS_INFO("Values[0] = %s", values[0]);

		//int i = 1;
        while (value != NULL)
        {
            value = strtok(NULL, ",");
            values.push_back(value);
            //ROS_INFO("Values[%d] = %s", i, values[i]);
            //i++;
        }

        //ROS_INFO("Values size: %d", values.size());

        if (values.size() == 14)
        {
            // Publish message
            imu_msg.linear_acceleration.x = atof(values[1]) * accel_factor;
            imu_msg.linear_acceleration.y = atof(values[2]) * accel_factor;
            imu_msg.linear_acceleration.z = atof(values[3]) * accel_factor;

            imu_msg.angular_velocity.x = atof(values[4])*deg2rad;
            imu_msg.angular_velocity.y = atof(values[5])*deg2rad;
            imu_msg.angular_velocity.z = atof(values[6])*deg2rad;

            roll = atof(values[10])*deg2rad;
            pitch = atof(values[11])*deg2rad;
            yaw = atof(values[12])*deg2rad;

            q.setRPY(roll, pitch, yaw);

            imu_msg.orientation.x = q[0];
            imu_msg.orientation.y = q[1];
            imu_msg.orientation.z = q[2];
            imu_msg.orientation.w = q[3];
            
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "base_imu_link";
            imu_msg.header.seq = seq;
            seq++;

            imu_pub.publish(imu_msg);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    close(fd);      // Close Serial

    return 0;
}
