#include "scala_rviz_display.h"

using namespace std;
double scala_color_alpha = 0.4;

double detection_scalar_x = 1.0;
double detection_scalar_y = 1.0;
double detection_scalar_z = 3.0;

int step = 0;
int objects_number_previous = 0;
int objects_eff_number_counter = 0;

void split(const string& src, const string& delim, vector<string>& dest) {
    string str = src;
    string::size_type start = 0, index;
    string substr;
    index = str.find_first_of(delim, start);   
    while (index != string::npos) {
        substr = str.substr(start, index - start);
        dest.push_back(substr);
        start = str.find_first_not_of(delim, index);   
        if (start == string::npos) return;
        index = str.find_first_of(delim, start);
    }
}

void ScalaRvizDisplay::Init(){
    sub_pointcloud = nh.subscribe("/scala_points", 1, &ScalaRvizDisplay::PointCloudCallback, this);
    sub_dynamic_objects = nh.subscribe("/scala_objects", 1, &ScalaRvizDisplay::ObjectsCallback, this);
    pub_marker_text = nh.advertise<visualization_msgs::MarkerArray>("objects_text", 1);
    pub_marker_arrow = nh.advertise<visualization_msgs::MarkerArray>("objects_arrow", 1);
    pub_marker_scala = nh.advertise<visualization_msgs::MarkerArray>("objects_scala", 1);
    pub_points = nh.advertise<sensor_msgs::PointCloud2>("points",1);
    tf_listener_map = new tf::TransformListener();

}

void ScalaRvizDisplay::PointCloudCallback(const sensor_msgs::PointCloud2& msg){
    step++;
    if(step==100)step=0;
    sensor_msgs::PointCloud2 points(msg);
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "bumper_scala";
    pub_points.publish(points);

}

void ScalaRvizDisplay::ObjectsCallback(const ibeo_scala::ObjectArray& msg){
    visualization_msgs::MarkerArray text_array;
    visualization_msgs::Marker text_object;
    text_object.header.frame_id = "bumper_scala";
    text_object.header.stamp = ros::Time::now();
    text_object.ns = "object_state";
    text_object.action = visualization_msgs::Marker::MODIFY;
    text_object.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_object.action = visualization_msgs::Marker::ADD;
    text_object.scale.z = 1;
    text_object.color.r = 1;
    text_object.color.g = 1;
    text_object.color.b = 1;
    text_object.color.a = 1;

    visualization_msgs::MarkerArray arrow_array;
    visualization_msgs::Marker arrow_object;
    arrow_object.header.frame_id = "bumper_scala";
    arrow_object.header.stamp = ros::Time::now();
    arrow_object.type = visualization_msgs::Marker::ARROW;
    arrow_object.action = visualization_msgs::Marker::ADD;
    arrow_object.scale.y = 0.05;
    arrow_object.scale.z = 0.05;
    arrow_object.color.r = 180;
    arrow_object.color.g = 0;
    arrow_object.color.b = 180;
    arrow_object.color.a = 1;

    visualization_msgs::MarkerArray scalar_array;
    visualization_msgs::Marker scala_object;
    scala_object.header.frame_id = "bumper_scala";
    scala_object.header.stamp = ros::Time::now();
    scala_object.type = visualization_msgs::Marker::CYLINDER;
    scala_object.action = visualization_msgs::Marker::ADD;
    scala_object.color.r = 100;
    scala_object.color.g = 0;
    scala_object.color.b = 100;
    scala_object.color.a = scala_color_alpha;
    scala_object.scale.x = detection_scalar_x;
    scala_object.scale.y = detection_scalar_y;
    scala_object.scale.z = detection_scalar_z;

    float vx, vy, vz, velocity, distance;
    float roll, pitch, yaw;
    roll = 0.0;
    pitch = 0.0;
    int objects_number = msg.tracks.size();

    for (int i = 0; i < objects_number; i++) {
        ibeo_scala::Object obj = msg.tracks[i];
        if(obj.age<6) continue;
        vector<float> orientation; //w,x,y,z
        vx = obj.velocity_absolute.x;
        vy = obj.velocity_absolute.y;
        velocity = sqrt(vx*vx+vy*vy);

        std::string obj_class;
        switch (obj.classification) {
            case 0: obj_class = "UnCls";
                break;
            case 1: obj_class = "UnknS";
                break;
            case 2: obj_class = "UnknB";
                break;
            case 3: obj_class = "Ped";break;
            case 4: obj_class = "Bike";break;
            case 5: obj_class = "Car";break;
            case 6: obj_class = "Truck";break;
            case 7: obj_class = "Overdrivable";break;
            case 8: obj_class = "Underdrivable";break;
            case 9: obj_class = "Bicyc";break;
            case 10: obj_class = "Motor";break;
            case 11: obj_class = "internal";break;
            default:obj_class = " ";break;
        }

        std::string dynamic_flag;
        switch (obj.dynamic_flag) {
            case 16: dynamic_flag = "D/M/I";break;
            case 32: dynamic_flag = "D/S/I";break;
            case 48: dynamic_flag = "inter/I";break;
            case 64: dynamic_flag = "PS/I";break;
            case 17: dynamic_flag = "D/M/T";break;
            case 33: dynamic_flag = "D/S/T";break;
            case 49: dynamic_flag = "inter/T";break;
            case 65: dynamic_flag = "PS/T";break;
            default:dynamic_flag = "";break;
        }

        //object box 
        yaw = obj.object_box_orientation/180.0*Pi;
        orientation = EulerToQuaternion(roll, pitch, yaw);
        text_object.id= i;
        SetPosition(text_object.pose, obj.reference_points[9].x, obj.reference_points[9].y, (float)1.0);
        distance = sqrt((obj.reference_points[9].x)*(obj.reference_points[9].x) + (obj.reference_points[9].y)*(obj.reference_points[9].y));
        std::ostringstream str,scala_distance,scala_velocity;
        /*str<<"Id:"<<obj.id<<std::endl
           <<"Age:"<<obj.age<<std::endl
           <<"DyF:"<<dynamic_flag<<std::endl
           <<"Vel:"<<velocity<<std::endl
           <<"Cls:"<<obj_class;*/
        scala_distance << std::setprecision(2) << distance;
        scala_velocity << std::setprecision(2) << velocity;
        float object_size = obj.object_box_size.x>obj.object_box_size.y ? obj.object_box_size.x:obj.object_box_size.y;
        str<<"Id:"<<obj.id<<" Age:"<<obj.age<<" Vel:"<<round(velocity)<<" Dist:"<<round(distance)<<" Size:"<<object_size;
        //for debugging
         /*Scala reference point for a obstacle is like below:
            1**5**2
            8**9**6
            4**7**3
        */
        /*str<<"Id:"<<obj.id<<" Age:"<<obj.age<<" Vel:"<<round(velocity)<<" Dist:"<<round(distance)<<"Size"<<object_size<<std::endl
           <<"P0:[x,]:" <<obj.track_shape.points[0].x<<","<<obj.track_shape.points[0].y<<std::endl
           <<"P1:[x,]:" <<obj.track_shape.points[1].x<<","<<obj.track_shape.points[1].y<<std::endl
           <<"P2:[x,]:" <<obj.track_shape.points[2].x<<","<<obj.track_shape.points[2].y<<std::endl
           <<"P3:[x,]:" <<obj.track_shape.points[3].x<<","<<obj.track_shape.points[3].y<<std::endl
           <<"P4:[x,]:" <<obj.track_shape.points[4].x<<","<<obj.track_shape.points[4].y<<std::endl
           <<"P5:[x,]:" <<obj.track_shape.points[5].x<<","<<obj.track_shape.points[5].y<<std::endl
           <<"P6:[x,]:" <<obj.track_shape.points[6].x<<","<<obj.track_shape.points[6].y<<std::endl
           <<"P7:[x,]:" <<obj.track_shape.points[7].x<<","<<obj.track_shape.points[7].y<<std::endl
           <<"P8:[x,]:" <<obj.track_shape.points[8].x<<","<<obj.track_shape.points[8].y<<std::endl
           <<"P9:[x,]:" <<obj.track_shape.points[9].x<<","<<obj.track_shape.points[9].y;*/

        text_object.text=str.str();
        if(dynamic_flag.size()>1){
            text_array.markers.push_back(text_object);
        }

        //object arrow
        arrow_object.id = i;
        SetPosition(arrow_object.pose, obj.reference_points[9].x, obj.reference_points[9].y, (float)0.0);
        orientation = EulerToQuaternion(roll, pitch, atan2(vy, vx));
        SetQuaternion(arrow_object.pose, orientation);
        arrow_object.scale.x = velocity;
        if(dynamic_flag.size()>1) {
            arrow_array.markers.push_back(arrow_object);
        }

        //scala object
        scala_object.id = i;
        SetPosition(scala_object.pose, obj.reference_points[9].x, obj.reference_points[9].y, obj.object_box_height);
        orientation = EulerToQuaternion(roll, pitch, atan2(vy, vx));
        SetQuaternion(scala_object.pose, orientation);

        if(dynamic_flag.size()>1) {
            scalar_array.markers.push_back(scala_object);
        }

    }

    visualization_msgs::MarkerArray markerarray_clean;
    visualization_msgs::Marker marker_clean;
    marker_clean.action = visualization_msgs::Marker::DELETEALL;
    markerarray_clean.markers.push_back(marker_clean);
    pub_marker_text.publish(markerarray_clean);
    pub_marker_text.publish(text_array);
    pub_marker_arrow.publish(markerarray_clean);
    pub_marker_arrow.publish(arrow_array);
    pub_marker_scala.publish(markerarray_clean);
    pub_marker_scala.publish(scalar_array);
}

//EulerToQuaternion, euler in rad
template<typename T>
vector<T> ScalaRvizDisplay::EulerToQuaternion(T roll, T pitch, T yaw)
{
    vector<T> q;
    double x, y, z, w;
    double a = roll/2.0;
    double b = pitch/2.0;
    double g = yaw/2.0;
    w = cos(a)*cos(b)*cos(g) + sin(a)*sin(b)*sin(g);
    x = sin(a)*cos(b)*cos(g) - cos(a)*sin(b)*sin(g);
    y = cos(a)*sin(b)*cos(g) + sin(a)*cos(b)*sin(g);
    z = cos(a)*cos(b)*sin(g) - sin(a)*sin(b)*cos(g);
    q.push_back(w);
    q.push_back(x);
    q.push_back(y);
    q.push_back(z);
    return q;
}

//QuaternionToEuler, euler in rad
template<typename T>
vector<T> ScalaRvizDisplay::QuaternionToEuler(vector<T> q)
{
    vector<T> e;
    T roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
    T pitch = asin(2*(q[0]*q[2]-q[1]*q[3]));
    T yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    e.push_back(roll);
    e.push_back(pitch);
    e.push_back(yaw);
    return e;
}

template<typename T>
void ScalaRvizDisplay::SetPosition(geometry_msgs::Pose &pose, T x, T y, T z)
{
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
}

template<typename T>
void ScalaRvizDisplay::SetQuaternion(geometry_msgs::Pose &pose, vector<T> q)
{
    pose.orientation.x = q[1];
    pose.orientation.y = q[2];
    pose.orientation.z = q[3];
    pose.orientation.w = q[0];
}

double ScalaRvizDisplay::GetDistance(double x1, double y1, double x2, double y2)
{
    double dx = (x1 - x2);
    double dy = (y1 - y2);
    return sqrt(dx*dx+dy*dy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scala_rviz_display");
    ScalaRvizDisplay scala_rviz_dispplay;
    scala_rviz_dispplay.Init();

    ros::MultiThreadedSpinner spinner(8);
    ros::Rate r(20);

    while(ros::ok()){
        spinner.spin();
        r.sleep();
    }
    ros::waitForShutdown();

    scala_rviz_dispplay.road_file.close();

    return 0;
}
