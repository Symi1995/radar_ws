#include "radar_conti_ars408_component.hpp"
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <data.h>

Radar_Conti::Radar_Conti(const ros::NodeHandle &nh) : nh(nh) {};

void Radar_Conti::init(can::DriverInterfaceSharedPtr &driver_)
{
    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("radar_objects_marker",0);
    pub_objects = nh.advertise<radar_conti::ObjectList>("radar_object_list",0);
    collison_obj_pub = nh.advertise<radar_conti::CollisonList>("radar_obj_collison",0);
    pub_cluster = nh.advertise<visualization_msgs::MarkerArray>("radar_cluster_markers",0);
    pub_cluster_list = nh.advertise<radar_conti::ClusterList>("radar_cluster_list",0);
    pub_marker_with_all_data = nh.advertise<visualization_msgs::MarkerArray>("radar_marker_with_all_data",0);
    pub_gps_data = nh.advertise<visualization_msgs::Marker>("gps_data",0);
    pub_closest_marker = nh.advertise<visualization_msgs::MarkerArray>("radar_closest_marker",0);
    pub_closest_str = nh.advertise<std_msgs::String>("radar_closest_distance", 0);
    
    //video
    pub_video_obj_speed = nh.advertise<visualization_msgs::MarkerArray>("radar_video_speed",0);
    pub_video_obj_distance = nh.advertise<visualization_msgs::MarkerArray>("radar_video_distance",0);

    this->driver_ = driver_;
    this->frame_listener_ = driver_->createMsgListenerM(this,&Radar_Conti::can_frame_callback);
}

void Radar_Conti::can_frame_callback(const can::Frame &msg)
{
//handle_object_list(msg);

    if (msg.id == ID_RadarState) {
        operation_mode_ =CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(msg.data),1.0);
    }
    if (operation_mode_ == 0x00)
        return;

    else if(operation_mode_ == 0x01)
        handle_object_list(msg);

    else if(operation_mode_ == 0x02)
        handle_cluster_list(msg);
}

void Radar_Conti::handle_object_list(const can::Frame &msg)
{
    if (msg.id == ID_Obj_0_Status) {
        publish_object_map();

        object_list_.header.stamp = ros::Time::now();
        object_list_.object_count.data = GET_Obj_0_Status_Obj_NofObjects(msg.data);

        object_map_.clear();
        collison_objects.clear();
    }
    else if (msg.id == ID_Obj_1_General) {

        radar_conti::Object o;
        //object ID
        int id = GET_Obj_1_General_Obj_ID(msg.data);
        o.obj_id.data = GET_Obj_1_General_Obj_ID(msg.data);

        // //RCLCPP_INFO(this->get_logger(), "Object_ID: 0x%04x", o.obj_id.data);

        //longitudinal distance
        o.object_general.obj_distlong.data =
                CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(msg.data), 1.0);

        //lateral distance
        o.object_general.obj_distlat.data =
                CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(msg.data), 1.0);

        //relative longitudinal velocity
        o.object_general.obj_vrellong.data =
                CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(msg.data), 1.0);

        //relative lateral velocity
        o.object_general.obj_vrellat.data =
                CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(msg.data), 1.0);

        o.object_general.obj_dynprop.data =
                CALC_Obj_1_General_Obj_DynProp(GET_Obj_1_General_Obj_DynProp(msg.data), 1.0);

        o.object_general.obj_rcs.data = 
                CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(msg.data), 1.0);

        //insert object into map
        
        object_map_.insert(std::pair<int, radar_conti::Object>(id, o));
    }
    else if (msg.id == ID_Obj_2_Quality) {

        int id = GET_Obj_2_Quality_Obj_ID(msg.data);

        object_map_[id].object_quality.obj_distlong_rms.data =
                CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_distlat_rms.data =
                CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_vrellong_rms.data =
                CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_vrellat_rms.data =
                CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(msg.data), 1.0);

        object_map_[id].object_quality.obj_probofexist.data =
                CALC_Obj_2_Quality_Obj_ProbOfExist(GET_Obj_2_Quality_Obj_ProbOfExist(msg.data), 1.0);

    }
    else if (msg.id == ID_Obj_3_Extended) {
        int id = GET_Obj_3_Extended_Obj_ID(msg.data);


        object_map_[id].object_extended.obj_arellong.data =
                CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(msg.data), 1.0);

        object_map_[id].object_extended.obj_arellat.data =
                CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(msg.data), 1.0);

        object_map_[id].object_extended.obj_class.data =
                CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

        int obj_class = CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

        object_map_[id].object_extended.obj_orientationangle.data =
                CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(msg.data),
                                                            1.0);
        
        object_map_[id].object_extended.obj_length.data =
            CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(msg.data), 1.0);

        object_map_[id].object_extended.obj_width.data =
                CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(msg.data), 1.0);
    }
    else if(msg.id == ID_Obj_4_Warning)
    {
        const int obj_war = CALC_Obj_4_Warning_Obj_ID(GET_Obj_4_Warning_Obj_ID(msg.data),1.0);
        collison_objects.insert(obj_war); 
    }
    else if (msg.id == ID_GPS_SPEED) {
        gps_speed = CALC_GPS_SPEED_SPEED_KPH(GET_GPS_SPEED_SPEED_MPH(msg.data), 1.0);
        
        gps_altitude = CALC_GPS_SPEED_ALTITUDE(GET_GPS_SPEED_ALTITUDE(msg.data), 1.0);
        
        gps_true_course = CALC_GPS_SPEED_TRUE_COURSE(GET_GPS_SPEED_TRUE_COURSE(msg.data), 1.0);
        
        gps_satelites = CALC_GPS_SPEED_SATELITES(GET_GPS_SPEED_SATELITES(msg.data), 1.0);
        
        gps_valid = CALC_GPS_SPEED_VALID(GET_GPS_SPEED_VALID(msg.data), 1.0);
    }
    else if (msg.id == ID_GPS_YAW) {
        gps_yaw_x = CALC_GPS_YAW_X_YAW(GET_GPS_YAW_X_YAW(msg.data), 1.0);
        
        gps_yaw_y = CALC_GPS_YAW_Y_YAW(GET_GPS_YAW_Y_YAW(msg.data), 1.0);
        
        gps_yaw_z = CALC_GPS_YAW_Z_YAW(GET_GPS_YAW_Z_YAW(msg.data), 1.0);
    }
    //publish_object_map();
}
void Radar_Conti::handle_cluster_list(const can::Frame &msg)
{
    if(msg.id == ID_Cluster_0_Status)
    {
        publish_cluster_map();

        cluster_list.header.stamp = ros::Time::now();
        cluster_list.cluster_count.data = GET_Cluster_0_Status_Cluster_NofClustersNear(msg.data);

        cluster_map_.clear();
    }
    if(msg.id == ID_Cluster_1_General)
    {
        radar_conti::Cluster c;

        int id = GET_Cluster_1_General_Cluster_ID(msg.data);
        c.cluster_id.data = id;

        c.cluster_general.cluster_distlong.data = 
                CALC_Cluster_1_General_Cluster_DistLong(GET_Cluster_1_General_Cluster_DistLong(msg.data),1.0);
        
        c.cluster_general.cluster_distlat.data = 
                CALC_Cluster_1_General_Cluster_DistLat(GET_Cluster_1_General_Cluster_DistLat(msg.data),1.0);

        c.cluster_general.cluster_vrellong.data = 
                CALC_Cluster_1_General_Cluster_VrelLong(GET_Cluster_1_General_Cluster_VrelLong(msg.data),1.0);

        c.cluster_general.cluster_vrellat.data = 
                CALC_Cluster_1_General_Cluster_VrelLat(GET_Cluster_1_General_Cluster_VrelLat(msg.data),1.0);

        c.cluster_general.cluster_dynprop.data = 
                CALC_Cluster_1_General_Cluster_DynProp(GET_Cluster_1_General_Cluster_DynProp(msg.data),1.0);

        c.cluster_general.cluster_rcs.data = 
                CALC_Cluster_1_General_Cluster_RCS(GET_Cluster_1_General_Cluster_RCS(msg.data),1.0);

        cluster_map_.insert(std::pair<int, radar_conti::Cluster>(id, c));
    }
    if(msg.id == ID_Cluster_2_Quality)
    {
        int id = GET_Cluster_2_Quality_Cluster_ID(msg.data);

        cluster_map_[id].cluster_quality.cluster_distlat_rms.data = 
                CALC_Cluster_2_Quality_Cluster_DistLat_rms(GET_Cluster_2_Quality_Cluster_DistLat_rms(msg.data),1.0);
        cluster_map_[id].cluster_quality.cluster_distlong_rms.data =
                CALC_Cluster_2_Quality_Cluster_DistLong_rms(GET_Cluster_2_Quality_Cluster_DistLong_rms(msg.data),1.0);
        cluster_map_[id].cluster_quality.cluster_vrellong_rms.data =
                CALC_Cluster_1_General_Cluster_VrelLong(GET_Cluster_1_General_Cluster_VrelLong(msg.data),1.0);
        cluster_map_[id].cluster_quality.cluster_vrellat_rms.data = 
                CALC_Cluster_2_Quality_Cluster_VrelLat_rms(GET_Cluster_2_Quality_Cluster_VrelLat_rms(msg.data),1.0);
        cluster_map_[id].cluster_quality.cluster_pdh0.data = 
                CALC_Cluster_2_Quality_Cluster_PdH0(GET_Cluster_2_Quality_Cluster_PdH0(msg.data),1.0);
        cluster_map_[id].cluster_quality.cluster_ambigstate.data = 
                CALC_Cluster_2_Quality_Cluster_AmbigState(GET_Cluster_2_Quality_Cluster_AmbigState(msg.data),1.0);
        cluster_map_[id].cluster_quality.cluster_invalidstate.data = 
                CALC_Cluster_2_Quality_Cluster_InvalidState(GET_Cluster_2_Quality_Cluster_InvalidState(msg.data),1.0);
    }



}

bool Radar_Conti::is_speed_in_threshold(double object_speed) {
        ROS_INFO("%lf", (gps_speed + object_speed));
        return (gps_speed + object_speed) < speed_threshold &&
                (gps_speed + object_speed) > -speed_threshold;
}

double Radar_Conti::prob_of_exist_data(int data)
{
        switch (data) {
                case 7: return 100.0;
                case 6: return  99.9;
                case 5: return  99.0;
                case 4: return  90.0;
                case 3: return  75.0;
                case 2: return  50.0;
                case 1: return  25.0;
                default: return  0.0;
        }
        return 0.0;
}

visualization_msgs::Marker Radar_Conti::createMarker(
        std::map<int, radar_conti::Object>::iterator itr, 
        const std::string topic_name, tf2::Quaternion& myQuaternion,
        const std::string ns_name, int id,
        double x, double y, double z)
{
        visualization_msgs::Marker markertext;

        markertext.header.stamp = ros::Time::now();
        markertext.header.frame_id = topic_name;
        markertext.ns = ns_name;
        markertext.id = id;
        markertext.type = (ns_name == "objects" ? 1 : 9);
        markertext.action = 0; // add/modify
        markertext.pose.position.x = (x == 0 ? itr->second.object_general.obj_distlong.data : x);
        markertext.pose.position.y = (y == 0 ? itr->second.object_general.obj_distlat.data : y);
        markertext.pose.position.z = z; //4.0

        myQuaternion.setRPY(0, 0, 0);
        markertext.pose.orientation.w = myQuaternion.getW();
        markertext.pose.orientation.x = myQuaternion.getX();
        markertext.pose.orientation.y = myQuaternion.getY();
        markertext.pose.orientation.z = myQuaternion.getZ();
        markertext.scale.z = 1.0;
        markertext.color.r = 1.0;
        markertext.color.g = 1.0;
        markertext.color.b = 1.0;
        markertext.color.a = 1.0;
        markertext.lifetime = ros::Duration(1);
        markertext.frame_locked = false;

        if (ns_name == "objects") {
                markertext.scale.x = itr->second.object_extended.obj_length.data/2;
                markertext.scale.y = itr->second.object_extended.obj_width.data/2;
                markertext.scale.z = 1.0;
        }

        return markertext;
}

std::string Radar_Conti::createTextMessage(
        const std::string ss_name, double distance,
        std::map<int, radar_conti::Object>::iterator itr)
{
        std::stringstream ss;
        ss.precision(2);
        ss << std::fixed << "object_" << std::fixed << std::setprecision(1) << itr->first << "\n"
        << " RCS: " << itr->second.object_general.obj_rcs.data << " dBm^2\n"
        << " Distance: " << distance << " m\n"; //closest distance return
        if (ss_name == "radar_all_data") {
                ss << " D_long (x): " << itr->second.object_general.obj_distlong.data << " m\n"
                << " D_lat (y): " << itr->second.object_general.obj_distlat.data << " m\n"
                << " Length (x): " << itr->second.object_extended.obj_length.data << " m\n"
                << " Width (y): " << itr->second.object_extended.obj_width.data << " m\n"
                << " Orientation: " << itr->second.object_extended.obj_orientationangle.data << "°\n"
                << " V_long: " << itr->second.object_general.obj_vrellong.data << "m/s" << " \n" 
                << " V_lat: " << itr->second.object_general.obj_vrellat.data << "m/s" << " \n" 
                << " Class: " << object_classes[itr->second.object_extended.obj_class.data] << "\n"
                << " ProbOfExist: " << prob_of_exist_data((int)itr->second.object_quality.obj_probofexist.data) << "%";
        }
        else if (ss_name == "radar" || ss_name == "video_speed") {
                ss << " V_long: " << (itr->second.object_general.obj_vrellong.data * 3.6) << "km/h" << " \n" 
                << " V_lat: " << (itr->second.object_general.obj_vrellat.data * 3.6)<< "km/h" << " \n";
                if (ss_name == "radar") {
                        ss << " Class: " << object_classes[itr->second.object_extended.obj_class.data] << "\n"
                        << " ProbOfExist: " << prob_of_exist_data((int)itr->second.object_quality.obj_probofexist.data) << "%";
                }
        }

        return ss.str();
}

void Radar_Conti::publish_object_map() {
        std::map<int, radar_conti::Object>::iterator itr;
        std::map<int, radar_conti::Object>::iterator closest_itr = object_map_.begin();

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::MarkerArray marker_array_all_data;
        visualization_msgs::MarkerArray marker_array_closest_object;

        visualization_msgs::Marker closest_obj;
        visualization_msgs::Marker closest_text;

        visualization_msgs::MarkerArray marker_video_obj_speed;
        visualization_msgs::MarkerArray marker_video_obj_distance;
        visualization_msgs::Marker video_obj_distance;
        visualization_msgs::Marker mtext_video_distance;

        tf2::Quaternion myQuaternion;

        std_msgs::String closest_obj_str;

        for (itr = object_map_.begin(); itr != object_map_.end(); ++itr) {
                if (itr->second.object_general.obj_distlong.data != 0 &&
                        itr->second.object_general.obj_distlat.data != 0 //csak autók vizsgálata
                        //&& itr->second.object_extended.obj_class.data != 0 //gyorsaság threshold
                        //&& !is_speed_in_threshold(((double)itr->second.object_general.obj_vrellong.data) * 3.6)
                ){
                        float itr_distance = sqrt(pow(itr->second.object_general.obj_distlong.data, 2) + pow(itr->second.object_general.obj_distlat.data, 2));
                        float closest_distance = sqrt(pow(closest_itr->second.object_general.obj_distlong.data, 2) + pow(closest_itr->second.object_general.obj_distlat.data, 2));

                        visualization_msgs::Marker mobject;
                        visualization_msgs::Marker mtext = createMarker(itr, "/radar", myQuaternion, "text", (itr->first + 100));
                        visualization_msgs::Marker mtext_all = createMarker(itr, "/radar_all_data", myQuaternion, "text", (itr->first + 100));
                        visualization_msgs::Marker mtext_video_speed = createMarker(itr, "/radar_video_speed", myQuaternion, "text", (itr->first + 100));
                        closest_text = createMarker(itr, "/radar_closest_object", myQuaternion, "text", (itr->first + 100), -5, 5, -11);
                        mtext_video_distance = createMarker(itr, "/radar_video_distance", myQuaternion, "text", (itr->first + 100), -5, 2, -6);
                        

                        myQuaternion.setRPY(0, 0, itr->second.object_extended.obj_orientationangle.data);
                        mobject = createMarker(itr, "/radar", myQuaternion, "objects", itr->first, 0, 0, 1);

                        if(collison_objects.find(itr->first) != collison_objects.end())
                        {
                                mobject.color.r = 1.0;
                                mobject.color.g = 0.0;
                                mtext.text= "object_" + std::to_string(itr->first) + ": \n"  
                                + " D_long: " +   std::to_string(itr->second.object_general.obj_distlong.data) + "m\n" 
                                + " D_lat: " + std::to_string(itr->second.object_general.obj_distlat.data) + "m\n";
                                radar_conti::CollisonObj collison_item;
                                collison_item.obj_id = itr->second.obj_id;
                                coll_list.objects.push_back(collison_item);
                        }
                        else
                        {
                                mtext.text = createTextMessage("radar", itr_distance, itr);
                                mtext_all.text = createTextMessage("radar_all_data", itr_distance, itr);
                                mtext_video_speed.text = createTextMessage("video_speed", itr_distance, itr);

                        }
                        // assign colors to each object ID (deterministic pseudo-random colors)
                        int i = int(itr->first);
                        int j = 100 - int(itr->first);
                        int k = itr->first + 5;
                        double r = double(i % 3) / 2.1;
                        double g = double(j % 21) / 21;
                        double b = double(k % 30) / 30;
                        mobject.color.r = r;
                        mobject.color.g = g;
                        mobject.color.b = b;
                        mobject.color.a = 0.6;
                        mobject.lifetime = ros::Duration(1);
                        mobject.frame_locked = false;

                        if ( (closest_distance > itr_distance) || 
                                (closest_itr == object_map_.begin()) /*|| //akkor kell, ha nem vizsgálunk pontokat (csak jármű kell) 
                                (closest_itr->second.object_extended.obj_class.data == 0)*/) {
                                closest_itr = itr;
                                closest_obj = mobject;

                                closest_text.text = createTextMessage("radar_all_data", itr_distance, itr);
                                mtext_video_distance.text = createTextMessage("video_closest_distance", itr_distance, itr);

                                closest_obj_str.data = std::to_string(itr_distance);
                        }

                        object_list_.objects.push_back(itr->second);
                        marker_array.markers.push_back(mobject);
                        marker_array.markers.push_back(mtext);

                        mobject.header.frame_id = "/radar_all_data";
                        marker_array_all_data.markers.push_back(mobject);
                        marker_array_all_data.markers.push_back(mtext_all);
                        
                        mobject.header.frame_id = "/radar_video_speed";
                        marker_video_obj_speed.markers.push_back(mobject);
                        marker_video_obj_speed.markers.push_back(mtext_video_speed);
                }
        }
        
        //********************************************************************
        //******************************        ******************************
        //*******************  C L O S E S T  O B J E C T  *******************
        //******************************        ******************************
        //********************************************************************
        if (sqrt(pow(closest_itr->second.object_general.obj_distlong.data, 2) + pow(closest_itr->second.object_general.obj_distlat.data, 2)) != 0
                //&& closest_itr->second.object_extended.obj_class.data != 0 //akkor kell, ha nem vizsgálunk pontokat (csak jármű kell)
                ){

                if (closest_text.header.frame_id == "/radar_closest_object") {
                        closest_obj.header.frame_id = "/radar_closest_object";
                        marker_array_closest_object.markers.push_back(closest_obj);
                        marker_array_closest_object.markers.push_back(closest_text);
                        pub_closest_marker.publish(marker_array_closest_object);
                        pub_closest_str.publish(closest_obj_str);
                }
                
                if (mtext_video_distance.header.frame_id == "/radar_video_distance") {
                        closest_obj.header.frame_id = "/radar_video_distance";
                        marker_video_obj_distance.markers.push_back(closest_obj);
                        marker_video_obj_distance.markers.push_back(mtext_video_distance);
                        pub_video_obj_distance.publish(marker_video_obj_distance);
                }
        }

        //********************************************************************
        //******************************        ******************************
        //*************************  G P S  T E X T  *************************
        //******************************        ******************************
        //********************************************************************
        myQuaternion.setRPY(0, 0, 0);
        visualization_msgs::Marker gps_text = createMarker(closest_itr, "/gps_data", myQuaternion, "text", 998, -5, -4, -10);;

        std::stringstream ss2;
        ss2.precision(2);
        ss2 << std::fixed << "GPS DATA:\n"
        << " GPS speed: " << gps_speed << " Km/h\n"
        << " GPS altitude: " << gps_altitude << " m\n"
        << " GPS true course: " << gps_true_course << "°\n"
        << " GPS satelites: " << gps_satelites << "\n"
        << " GPS valid: " << gps_valid << "\n"
        << " GPS yaw x: " << gps_yaw_x << "\n"
        << " GPS yaw y: " << gps_yaw_y << "\n"
        << " GPS yaw z: " << gps_yaw_z << "\n";
        gps_text.text = ss2.str();


        //********************************************************************
        //******************************        ******************************
        //*****************  P U B L I S H   T O   R V I Z  ******************
        //******************************        ******************************
        //********************************************************************
        pub_marker.publish(marker_array);
        pub_marker_with_all_data.publish(marker_array_all_data);
        pub_gps_data.publish(gps_text);
        pub_video_obj_speed.publish(marker_video_obj_speed);
}

void Radar_Conti::publish_cluster_map()
{
        std::map<int, radar_conti::Cluster>::iterator itr;

        
        visualization_msgs::MarkerArray marker_array;

        //marker for ego car
        visualization_msgs::Marker mEgoCar;

        mEgoCar.header.stamp = ros::Time::now();
        mEgoCar.header.frame_id = frame_id_;
        mEgoCar.ns = "ego";
        mEgoCar.id = 999;

        //if you want to use a cube comment out the next 2 line
        mEgoCar.type = 1; // cube
        mEgoCar.action = 0; // add/modify
        mEgoCar.pose.position.x = 0.0;
        mEgoCar.pose.position.y = 0.0;
        mEgoCar.pose.position.z = 0.0;

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, M_PI/2);

        mEgoCar.pose.orientation.w = myQuaternion.getW();
        mEgoCar.pose.orientation.x = myQuaternion.getX();
        mEgoCar.pose.orientation.y = myQuaternion.getY();
        mEgoCar.pose.orientation.z = myQuaternion.getZ();
        mEgoCar.scale.x = 0.2;
        mEgoCar.scale.y = 0.2;
        mEgoCar.scale.z = 0.2;
        mEgoCar.color.r = 0.0;
        mEgoCar.color.g = 0.0;
        mEgoCar.color.b = 1.0;
        mEgoCar.color.a = 1.0;
        mEgoCar.lifetime = ros::Duration(1);
        mEgoCar.frame_locked = false;

        marker_array.markers.push_back(mEgoCar);

        for (itr = cluster_map_.begin(); itr != cluster_map_.end(); ++itr) {

                visualization_msgs::Marker mobject;
                visualization_msgs::Marker mtext;

                mtext.header.stamp = ros::Time::now();
                mtext.header.frame_id = frame_id_;
                mtext.ns = "text";
                mtext.id = (itr->first+100);
                mtext.type = 1; //Cube
                mtext.action = 0; // add/modify
                mtext.pose.position.x = itr->second.cluster_general.cluster_distlong.data;
                mtext.pose.position.y = itr->second.cluster_general.cluster_distlat.data;
                mtext.pose.position.z = 4.0;

        
                //myQuaternion.setRPY(M_PI / 2, 0, 0);
                myQuaternion.setRPY(0, 0, 0);

                mtext.pose.orientation.w = myQuaternion.getW();
                mtext.pose.orientation.x = myQuaternion.getX();
                mtext.pose.orientation.y = myQuaternion.getY();
                mtext.pose.orientation.z = myQuaternion.getZ();
                // mtext.scale.x = 1.0;
                // mtext.scale.y = 1.0;
                mtext.scale.z = 2.0;
                mtext.color.r = 1.0;
                mtext.color.g = 1.0;
                mtext.color.b = 1.0;
                mtext.color.a = 1.0;
                mtext.lifetime = ros::Duration(1);
                mtext.frame_locked = false;
                mtext.type=9;
                mtext.text= "Cluster" + std::to_string(itr->first) + ": \n" 
                + " RCS: " + std::to_string(itr->second.cluster_general.cluster_rcs.data) + "dBm^2"; // + " \n" 
                //+ " V_long: " +   std::to_string(itr->second.cluster_general.cluster_vrellong.data) + "m/s" + " \n" 
                //+ " V_lat: " + std::to_string(itr->second.cluster_general.cluster_vrellat.data) + "m/s" + " \n";
                //+ " Orientation: " + std::to_string(itr->second.cluster_general.cluster_vrellon);


                marker_array.markers.push_back(mtext);



                mobject.header.stamp = ros::Time::now();
                mobject.header.frame_id = frame_id_;
                mobject.ns = "objects";
                mobject.id = itr->first;
                mobject.type = 1; //Cube
                mobject.action = 0; // add/modify
                mobject.pose.position.x = itr->second.cluster_general.cluster_distlong.data;
                mobject.pose.position.y = itr->second.cluster_general.cluster_distlat.data;
                mobject.pose.position.z = 1.0;

                myQuaternion.setRPY(0, 0, 0);

                mobject.pose.orientation.w = myQuaternion.getW();
                mobject.pose.orientation.x = myQuaternion.getX();
                mobject.pose.orientation.y = myQuaternion.getY();
                mobject.pose.orientation.z = myQuaternion.getZ();
                mobject.scale.x = 2.0;
                mobject.scale.y = 2.0;
                mobject.scale.z = 1.0;
                mobject.color.r = 0.0;
                mobject.color.g = 1.0;
                mobject.color.b = 0.0;
                mobject.color.a = 1.0;
                mobject.lifetime = ros::Duration(1);
                mobject.frame_locked = false;

                cluster_list.clusters.push_back(itr->second);

                marker_array.markers.push_back(mobject);


        }
        pub_cluster_list.publish(cluster_list);
        pub_cluster.publish(marker_array);

}

