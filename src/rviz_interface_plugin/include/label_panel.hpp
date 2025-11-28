#pragma once
#include <pcl_conversions/pcl_conversions.h>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <string>

#include "QCheckBox"
#include "QFileDialog"
#include "QLabel"
#include "QLineEdit"
#include "QPushButton"
#include "QSettings"
#include "QSpinBox"
#include "geometry_msgs/msg/point.hpp"
#include "labeling_msgs/srv/frame_request.hpp"
#include "labeling_msgs/srv/labeling_assist_request.hpp"
#include "labeling_msgs/srv/rosbag_info_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "std_msgs/msg/string.hpp"

namespace rviz_interface_plugin {
    class LabelPanel : public rviz_common::Panel {
        Q_OBJECT

       public:
        explicit LabelPanel(QWidget *parent = nullptr);
        rclcpp::Node::SharedPtr raw_node;
        std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
        void onInitialize() override;
        void retrieveClusters(const sensor_msgs::msg::PointCloud2 pointcloud);
        bool isSelectedRosbagFileLoaded();
        void displaySelectedRosbagFile();
        void displayRosbagFrame();
        void mapClusterNames(std::vector<labeling_msgs::msg::Cluster> clusters);
        void displaySuggestedClusters();
        visualization_msgs::msg::InteractiveMarker createMarker(std::string name, float x, float y, float z);
        void iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
        void initMenu();

        void makeMenuMarker(std::string name, float x, float y, float z);
        visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker &msg);
        void addCluster(labeling_msgs::msg::Cluster cluster);
        void approximateNewClusterFromPoint(geometry_msgs::msg::Point point);

       public Q_SLOTS:
        void openRosbagFileSelect();
        void loadSelectedRosbagFile();
        void loadRosbagFrame();
        void loadSuggestedClusters();
        void saveFrame();
        void toggleFilteredView();

       private:
        void removeCluster(std::string name);
        void clearClusters();
        void setMarkerColor(std::string int_marker_name, float r, float g, float b);
        std::string generateClusterName(uint32_t index);
        void updateSuggestedClustersRemaining();
        std::vector<pcl::PointXYZI> getPointCloud2AssociatedWithCluster(const std::string& name);

        std::string selected_rosbag_file_name_;
        uint64_t number_of_frames_;
        uint64_t selected_frame_;
        sensor_msgs::msg::PointCloud2 pointcloud_;
        sensor_msgs::msg::PointCloud2 filtered_pointcloud_;
        std::vector<labeling_msgs::msg::Cluster> clusters_;
        std::map<std::string, labeling_msgs::msg::Cluster> cluster_map_;
        interactive_markers::MenuHandler menu_handler_;
        interactive_markers::MenuHandler::EntryHandle entry_handle_;
        int cluster_index_;


       protected:
        rclcpp::Client<labeling_msgs::srv::RosbagInfoRequest>::SharedPtr rosbag_client_;
        rclcpp::Client<labeling_msgs::srv::FrameRequest>::SharedPtr frame_client_;
        rclcpp::Client<labeling_msgs::srv::LabelingAssistRequest>::SharedPtr cluster_client_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr add_cluster_near_point_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        QLabel *rosbag_file_label_;
        QFileDialog *rosbag_file_select_;
        QPushButton *open_rosbag_file_select_;

        QLabel *frame_number_label_;
        QSpinBox *frame_number_select_;

        QLabel *max_range_label_;
        QSpinBox *max_range_select_;

        QLabel *proposals_remaining_label_;
        QPushButton *save_button_;

        QCheckBox *toggle_filtered_checkbox_;
    };

    class LabelTool : public rviz_common::Tool {
       public:
        explicit LabelTool();
        int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;
        rclcpp::Node::SharedPtr raw_node;
        void onInitialize() override;
        void activate() override;
        void deactivate() override;

       protected:
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
        std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
    };

}  // end namespace rviz_interface_plugin
