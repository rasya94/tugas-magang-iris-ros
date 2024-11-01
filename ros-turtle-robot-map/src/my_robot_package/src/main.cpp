#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h> 

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_opencv_controller");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("/kill");

    VideoCapture cap("/home/rasya/Documents/ROBOTIK/ROS/tugas_turtle/turtle_map_ws/src/my_robot_package/src/Video.mp4");
    if (!cap.isOpened()) {
        cerr << "No Camera";
        return -1;
    }

    int h_min = 0, s_min = 150, v_min = 150;
    int h_max = 15, s_max = 210, v_max = 255;

    Mat frame, hsv_frame, mask;
    Point2f initial_pos(960, 540);
    Point2f robot_position(0, 0);
    Point2f prev_robot_position(0, 0);
    bool spawned = false;

    while (ros::ok() && cap.read(frame)) {
        if (frame.empty()) {
            break;
        }

        cvtColor(frame, hsv_frame, COLOR_BGR2HSV);
        Scalar min_hsv(h_min, s_min, v_min);
        Scalar max_hsv(h_max, s_max, v_max);

        inRange(hsv_frame, min_hsv, max_hsv, mask);
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            vector<Point> largest_contour = contours[0];
            for (size_t i = 1; i < contours.size(); i++) {
                if (contourArea(contours[i]) > contourArea(largest_contour)) {
                    largest_contour = contours[i];
                }
            }

            Rect bounding_rect = boundingRect(largest_contour);
            rectangle(frame, bounding_rect, Scalar(0, 0, 255), 2);
            Point2f square_center = (bounding_rect.tl() + bounding_rect.br()) / 2;

            int offset = 40;
            circle(frame, bounding_rect.tl() + Point(offset, offset), 3, Scalar(255, 0, 0), -1);
            circle(frame, Point(bounding_rect.br().x, bounding_rect.tl().y) + Point(-offset, offset), 3, Scalar(255, 0, 0), -1);
            circle(frame, bounding_rect.br() + Point(-offset, -offset), 3, Scalar(255, 0, 0), -1);
            circle(frame, Point(bounding_rect.tl().x, bounding_rect.br().y) + Point(offset, -offset), 3, Scalar(255, 0, 0), -1);

            Point2f rel_position = square_center - initial_pos;
            Point2f real_world_position(rel_position.x * 10, rel_position.y * 10); // 1px = 10cm

            if (prev_robot_position.x == 0 && prev_robot_position.y == 0) {
                prev_robot_position = real_world_position;
            }

            Point2f delta_pos = Point2f(- (real_world_position.x - prev_robot_position.x), real_world_position.y - prev_robot_position.y);
            robot_position += delta_pos;

            string position_label_r = "Posisi Robot: ( X: " + to_string(static_cast<int>(robot_position.x)) + " cm, Y: " + to_string(static_cast<int>(robot_position.y)) + " cm)";
            putText(frame, position_label_r, Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);

            // string position_label_b = "Posisi Bola: (0, 0)";
            string position_label_b = "Posisi Robot: ( X: " + to_string(static_cast<int>(square_center.x)) + " cm, Y: " + to_string(static_cast<int>(square_center.y)) + " cm)";
            putText(frame, position_label_b, Point(square_center.x + 50, square_center.y + 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);

            prev_robot_position = real_world_position;
            Point2f adj_square_center(square_center.x, -square_center.y); 
            Point2f pos_offset = adj_square_center - initial_pos;
            Point2f robot_initial_pos = initial_pos - pos_offset;

            float x_scale = 11.0 / 1920;
            float y_scale = 11.0 / 1080;
            float spawn_x = ((robot_initial_pos.x - initial_pos.x) * x_scale)*1.9 + 5.5;
            float spawn_y = ((initial_pos.y - robot_initial_pos.y) * y_scale) + 5.5;

            geometry_msgs::Twist move_cmd;

            if (!spawned) {
                turtlesim::Kill kill_srv;
                kill_srv.request.name = "turtle1";
                kill_client.call(kill_srv);
                
                turtlesim::Spawn spawn_srv;
                spawn_srv.request.x = (spawn_x);
                spawn_srv.request.y = (spawn_y);
                spawn_srv.request.theta = 0.0;
                spawn_srv.request.name = "turtle1";
                spawn_client.call(spawn_srv);

                spawn_srv.request.x = 5.5;
                spawn_srv.request.y = 5.5;
                spawn_srv.request.name = "turtle2";
                spawn_client.call(spawn_srv);
                spawned = true;
            }

            move_cmd.linear.x = delta_pos.x/100; // scale 
            move_cmd.linear.y = delta_pos.y/100; // scale 
            pub.publish(move_cmd);

            imshow("Frame", frame);
            // imshow("Mask", mask);
        }

        if (waitKey(1) == 'q') {
            break;
        }

        ros::spinOnce();

        ros::Rate loop_rate(10);
        loop_rate.sleep();
    }

    cap.release();
    return 0;
}
