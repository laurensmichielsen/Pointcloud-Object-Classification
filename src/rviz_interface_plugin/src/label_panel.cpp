#include "label_panel.hpp"

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <json/json.h>

#include <pluginlib/class_list_macros.hpp>

#include "QHBoxLayout"
#include "custom_msgs/msg/cone.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/render_window.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace rviz_interface_plugin {
    LabelPanel::LabelPanel(QWidget* parent) : rviz_common::Panel(parent) {
        // The constructor defines the UI of the rviz_common::Panel plugin
        // .DB3 File selection
        rosbag_file_label_ = new QLabel("No file selected");
        rosbag_file_label_->setAlignment(Qt::AlignLeft);

        rosbag_file_select_ = new QFileDialog();
        rosbag_file_select_->setFileMode(QFileDialog::ExistingFile);
        rosbag_file_select_->setNameFilter("All db3 files (*.db3)");
        connect(rosbag_file_select_, SIGNAL(fileSelected(QString)), SLOT(loadSelectedRosbagFile()));

        open_rosbag_file_select_ = new QPushButton("Select");
        connect(open_rosbag_file_select_, SIGNAL(clicked(bool)), SLOT(openRosbagFileSelect()));

        auto* rosbag_file_select_layout = new QHBoxLayout;
        rosbag_file_select_layout->addWidget(rosbag_file_label_);
        rosbag_file_select_layout->addWidget(rosbag_file_select_);
        rosbag_file_select_layout->addWidget(open_rosbag_file_select_);

        // Frame number selection
        frame_number_label_ = new QLabel("Select frame number:");
        frame_number_label_->setAlignment(Qt::AlignLeft);

        frame_number_select_ = new QSpinBox();
        frame_number_select_->setEnabled(false);
        frame_number_select_->setValue(0);
        connect(frame_number_select_, SIGNAL(valueChanged(int)), SLOT(loadRosbagFrame()));

        auto* frame_number_select_layout = new QHBoxLayout;
        frame_number_select_layout->addWidget(frame_number_label_);
        frame_number_select_layout->addWidget(frame_number_select_);

        // Max range selection
        max_range_label_ = new QLabel("Max range (meters):");
        max_range_label_->setAlignment(Qt::AlignLeft);

        max_range_select_ = new QSpinBox();
        max_range_select_->setValue(30);
        connect(max_range_select_, SIGNAL(valueChanged(int)), SLOT(loadRosbagFrame()));

        auto* max_range_select_layout = new QHBoxLayout;
        max_range_select_layout->addWidget(max_range_label_);
        max_range_select_layout->addWidget(max_range_select_);

        // Remaining clusters to label and save frame button
        proposals_remaining_label_ = new QLabel("");
        proposals_remaining_label_->setAlignment(Qt::AlignLeft);

        save_button_ = new QPushButton("Save frame");
        save_button_->setEnabled(false);
        connect(save_button_, SIGNAL(clicked()), SLOT(saveFrame()));

        auto* save_layout = new QHBoxLayout;
        save_layout->addWidget(proposals_remaining_label_);
        save_layout->addWidget(save_button_);

        auto toggle_filtered_label = new QLabel("Toggle filtered view");
        toggle_filtered_checkbox_ = new QCheckBox();
        connect(toggle_filtered_checkbox_, SIGNAL(stateChanged(int)), SLOT(toggleFilteredView()));

        auto* toggle_filtered_layout = new QHBoxLayout;
        toggle_filtered_layout->addWidget(toggle_filtered_label);
        toggle_filtered_layout->addWidget(toggle_filtered_checkbox_);

        auto* add_button_label = new QLabel("To add new cluster select the LabelTool");

        auto* add_layout = new QHBoxLayout;
        add_layout->addWidget(add_button_label);

        // Add all widgets to the final vertical box layout
        auto* v_layout = new QVBoxLayout;
        v_layout->addLayout(rosbag_file_select_layout);
        v_layout->addLayout(frame_number_select_layout);
        v_layout->addLayout(max_range_select_layout);
        v_layout->addLayout(save_layout);
        v_layout->addLayout(toggle_filtered_layout);
        v_layout->addLayout(add_layout);

        setLayout(v_layout);
    }

    // Override from rviz_common:Panel
    // Initialize all other components of the plugin
    void LabelPanel::onInitialize() {
        rosbag_file_select_->close();

        // Get the rviz node context during initialization, since it is not available during constructor time
        raw_node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Initialize all service clients to communicate with `rosbag_frame_loader` and `labeling_assist`
        rosbag_client_ = raw_node->create_client<labeling_msgs::srv::RosbagInfoRequest>("rosbag_info_request");
        frame_client_ = raw_node->create_client<labeling_msgs::srv::FrameRequest>("frame_request");
        cluster_client_ = raw_node->create_client<labeling_msgs::srv::LabelingAssistRequest>("labeling_assist_request");

        // Initialize listener for external cluster additions
        add_cluster_near_point_subscriber_ = raw_node->create_subscription<geometry_msgs::msg::Point>(
            "add_cluster_near_point", 1,
            std::bind(&LabelPanel::approximateNewClusterFromPoint, this, std::placeholders::_1));

        // Initialize the interactive marker server called `labeling_tool`
        server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("labeling_tool", raw_node);

        // Should probably need a mapping structure to map from menu label to id
        menu_handler_ = interactive_markers::MenuHandler();
        entry_handle_ =
            menu_handler_.insert("Blue cone", std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));
        entry_handle_ =
            menu_handler_.insert("Yellow cone", std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));
        entry_handle_ =
            menu_handler_.insert("Orange cone", std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));
        entry_handle_ =
            menu_handler_.insert("Unknown cone", std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));
        entry_handle_ =
            menu_handler_.insert("False positive", std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));
        entry_handle_ =
            menu_handler_.insert("Delete", std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));

        // Apply changes to the interactive marker server
        server_->applyChanges();

        // Create the lidar data publisher for visualization in rviz
        publisher_ = raw_node->create_publisher<sensor_msgs::msg::PointCloud2>("/labeling/pointcloud", 10);
    }

    void LabelPanel::openRosbagFileSelect() { 
        RCLCPP_INFO(raw_node->get_logger(), "Opening the rosbag select");
        rosbag_file_select_->open();
        RCLCPP_INFO(raw_node->get_logger(), "Opened the file select");
    }

    void LabelPanel::loadSelectedRosbagFile() {
        RCLCPP_INFO(raw_node->get_logger(), "Loading the selected file");
        selected_rosbag_file_name_ = rosbag_file_select_->selectedFiles()[0].toStdString();
        auto request = std::make_shared<labeling_msgs::srv::RosbagInfoRequest::Request>();

        // Prepare the selected file name std_msgs::msg::String message
        std_msgs::msg::String file_name;
        file_name.data = selected_rosbag_file_name_.c_str();
        request->file = file_name;
        
        RCLCPP_INFO(raw_node->get_logger(), "Waiting for the service");

        // Wait for the service to be available
        while (!frame_client_->wait_for_service(1s)) {
        }
        RCLCPP_INFO(raw_node->get_logger(), "service is available");
        // Setup the service callback to request information about the selected rosbag file
        using ServiceResponseFuture = rclcpp::Client<labeling_msgs::srv::RosbagInfoRequest>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            // TODO: handle status code
            RCLCPP_INFO(raw_node->get_logger(), "Loading the selected file callback");
            // Retrieve the number of frames returned from the response
            auto result = future.get();
            number_of_frames_ = result->number_of_frames;
            RCLCPP_INFO(raw_node->get_logger(), "Got the number of frames");
            // Display the information about the selected rosbag
            displaySelectedRosbagFile();
        };

        // Call the RosbagInfoRequest service
        auto future_result = rosbag_client_->async_send_request(request, response_received_callback);
    }

    // TODO: rewrite
    bool LabelPanel::isSelectedRosbagFileLoaded() { return selected_rosbag_file_name_.length() > 0; }

    // Display the number of frames available in a rosbag file and update the minimum and maximum of the frame number
    // select input
    void LabelPanel::displaySelectedRosbagFile() {
        rosbag_file_label_->setText(QString(selected_rosbag_file_name_.c_str()));
        frame_number_label_->setText(QString(("Frame number (1-" + std::to_string(number_of_frames_) + ")").c_str()));
        frame_number_select_->setMinimum(1);
        frame_number_select_->setMaximum(number_of_frames_);
        frame_number_select_->setEnabled(true);
        RCLCPP_INFO(raw_node->get_logger(), "Displayed the number of frames");
    }

    void LabelPanel::loadRosbagFrame() {
        RCLCPP_INFO(raw_node->get_logger(), "Loading the frame");
        if (!isSelectedRosbagFileLoaded()) {
            return;
        }
        RCLCPP_INFO(raw_node->get_logger(), "File is loaded while loading the frame");
        auto request = std::make_shared<labeling_msgs::srv::FrameRequest::Request>();

        // Prepare the selected file name (std_msgs::msg::String) and frame number (int)
        std_msgs::msg::String file_name;
        file_name.data = selected_rosbag_file_name_;
        request->file = file_name;
        request->frame = frame_number_select_->value();

        RCLCPP_INFO(raw_node->get_logger(), "Waiting for the service to be available");
        // Wait for the service to be available
        while (!rosbag_client_->wait_for_service(1s)) {
        }
        RCLCPP_INFO(raw_node->get_logger(), "Service is available");
        // Setup the service callback to request the PointCloud2 contents of the selected frame of the selected rosbag
        // file
        using ServiceResponseFuture = rclcpp::Client<labeling_msgs::srv::FrameRequest>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            // TODO: handle status code
            RCLCPP_INFO(raw_node->get_logger(), "In the frame load callback");
            // Retrieve the pointcloud from the response
            auto result = future.get();
            pointcloud_ = result->pointcloud;
            filtered_pointcloud_ = sensor_msgs::msg::PointCloud2{};
            selected_frame_ = frame_number_select_->value();
            
            RCLCPP_INFO(raw_node->get_logger(), "Got the result in the frame load callback");
            // Clear the current cluster map and publish the pointcloud.
            // Finally call the labeling assist to populate the cluster map again
            clearClusters();
            RCLCPP_INFO(raw_node->get_logger(), "Cleared the clusters");
            displayRosbagFrame();
            RCLCPP_INFO(raw_node->get_logger(), "Displayed the rosbag frame");
            loadSuggestedClusters();
        };

        RCLCPP_INFO(raw_node->get_logger(), "Calling the frame client");
        // Call the FrameRequest service
        auto future_result = frame_client_->async_send_request(request, response_received_callback);
    }

    void LabelPanel::displayRosbagFrame() {
        // Set the correct header frame_id

        if (toggle_filtered_checkbox_->isChecked()) {
            filtered_pointcloud_.header.frame_id = "labeling_pointcloud";
            publisher_->publish(filtered_pointcloud_);
        } else {
            // double x_cloud; double y_cloud; double z_cloud;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(pointcloud_, *cloud);
            pcl::PointCloud<pcl::PointXYZI>::Ptr msg(new pcl::PointCloud<pcl::PointXYZI>);

            for (size_t i = 0; i < cloud->size(); i++) {
                pcl::PointXYZI point(cloud->points[i].intensity);
                point.x = cloud->points[i].x;
                point.y = cloud->points[i].y;
                point.z = cloud->points[i].z;
                auto max_range = max_range_select_->value();
                if (point.x * point.x + point.y * point.y < max_range * max_range) {
                    msg->points.push_back(point);
                }
            }

            sensor_msgs::msg::PointCloud2 output_cloud_msg;
            pcl::toROSMsg(*msg, output_cloud_msg);

            output_cloud_msg.header.frame_id = "labeling_pointcloud";
            publisher_->publish(output_cloud_msg);
        }
    }

    void LabelPanel::loadSuggestedClusters() {
        auto request = std::make_shared<labeling_msgs::srv::LabelingAssistRequest::Request>();
        RCLCPP_INFO(raw_node->get_logger(), "In the load suggested clusters function");
        // Prepare the loaded pointcloud
        request->pointcloud = pointcloud_;
        request->max_range = max_range_select_->value();
        RCLCPP_INFO(raw_node->get_logger(), "Waiting for the service in the load suggested cluster function");
        // Wait for the service to be available
        while (!cluster_client_->wait_for_service(1s)) {
        }
        RCLCPP_INFO(raw_node->get_logger(), "Done Waiting for the service in the load suggested cluster function");
        // Setup the service callback to request the clusters of the provided pointcloud
        using ServiceResponseFuture = rclcpp::Client<labeling_msgs::srv::LabelingAssistRequest>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            RCLCPP_INFO(raw_node->get_logger(), "Callback");
            auto result = future.get();
            auto clusters = result->clusters;
            filtered_pointcloud_ = result->filtered_pointcloud;
            clearClusters();
            mapClusterNames(clusters);
            displaySuggestedClusters();
        };

        auto future_result = cluster_client_->async_send_request(request, response_received_callback);
    }

    // Map a generated name to each cluster. Interactive markers will be assigned the same name
    // so we can access the corresponding cluster information
    void LabelPanel::mapClusterNames(std::vector<labeling_msgs::msg::Cluster> clusters) {
        for (std::vector<int>::size_type i = 0; i != clusters.size(); i++) {
            addCluster(clusters[i]);
        }
    }

    // Create a menu marker for each name, cluster pair in the cluster map
    void LabelPanel::displaySuggestedClusters() {
        for (auto const& entry : cluster_map_) {
            auto cluster_name = entry.first;
            auto cluster = entry.second;
            makeMenuMarker(cluster_name, cluster.centroid.x, cluster.centroid.y, cluster.centroid.z);
        }
        server_->applyChanges();
        updateSuggestedClustersRemaining();
    }

    // Update the label to display the number of unlabeled clusters
    // Enable the save button if there are no unlabeled clusters left
    void LabelPanel::updateSuggestedClustersRemaining() {
        uint8_t remaining = 0;
        for (auto const& entry : cluster_map_) {
            if (entry.second.type == 0) {
                remaining++;
            }
        }
        proposals_remaining_label_->setText(
            QString((std::to_string(remaining) + "/" + std::to_string(cluster_map_.size()) + " remaining").c_str()));
        save_button_->setEnabled(remaining == 0);
    }

    void LabelPanel::clearClusters() {
        server_->clear();
        cluster_index_ = 0;
        cluster_map_.clear();
        server_->applyChanges();
    }

    std::string LabelPanel::generateClusterName(uint32_t index) {
        auto name = "cluster_" + std::to_string(index);
        cluster_index_++;
        return name;
    }

    void LabelPanel::addCluster(labeling_msgs::msg::Cluster cluster) {
        cluster_map_[generateClusterName(cluster.id)] = cluster;
        //displaySuggestedClusters();
    }

    void LabelPanel::removeCluster(std::string name) {
        server_->erase(name);
        server_->applyChanges();
    }

    void LabelPanel::approximateNewClusterFromPoint(geometry_msgs::msg::Point point) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(pointcloud_, *cloud);

        float closest_z;
        float closest_distance = 1000.0f;
        for (size_t i = 0; i < cloud->size(); i++) {
            float x = cloud->points[i].x;
            float y = cloud->points[i].y;
            float distance = (x - point.x) * (x - point.x) + (y - point.y) * (y - point.y);
            if (distance < closest_distance) {
                closest_z = cloud->points[i].z;
                closest_distance = (x - point.x) * (x - point.x) + (y - point.y) * (y - point.y);
            }
        }

        labeling_msgs::msg::Cluster cluster{};
        cluster.centroid = point;
        cluster.centroid.z = closest_z + 0.1f;
        addCluster(cluster);
    }

    void LabelPanel::toggleFilteredView() { displayRosbagFrame(); }

    // Saves the current cluster map data to a json file named after the original
    void LabelPanel::saveFrame() {
        float max_range = 0.25f;
        Json::Value frame_data;
        Json::Value clusters(Json::arrayValue);

        // Create frame data
        for (auto const& entry : cluster_map_) {
            Json::Value mean;
            auto centroid = entry.second.centroid;

            Json::Value points_json(Json::arrayValue);
            auto points = getPointCloud2AssociatedWithCluster(entry.first);

            for (auto const& i : points) {
                Json::Value point;
                point["x"] = i.x;
                point["y"] = i.y;
                point["z"] = i.z;
                point["i"] = i.intensity;
                points_json.append(point);
            }

            mean["points"] = points_json;
            mean["points_length"] = points.size();
            mean["mean"]["x"] = centroid.x;
            mean["mean"]["y"] = centroid.y;
            mean["mean"]["z"] = centroid.z;
            mean["label"] = entry.second.type;
            clusters.append(mean);
        }

        frame_data["clusters"] = clusters;
        frame_data["max_cluster_size"] = max_range;
        frame_data["max_distance"] = max_range_select_->value();

        // Get base filename without path and extension
        std::string base_filename = selected_rosbag_file_name_.substr(selected_rosbag_file_name_.find_last_of("/\\") + 1);
        std::string::size_type const p(base_filename.find_last_of('.'));
        std::string file_without_extension = base_filename.substr(0, p);

        // Create folder if it doesn't exist
        std::string folderstr = file_without_extension;
        char* folder = folderstr.data();
        mkdir(folder, 0777);

        // Construct json file path
        std::string json_path = folderstr + "/" + file_without_extension + "_labels.json";

        // Load existing JSON file if it exists
        Json::Value root;
        std::ifstream input_file(json_path);
        if (input_file.good()) {
            input_file >> root;
            input_file.close();
        }

        // Add or update frame data
        std::string frame_key = "frame_" + std::to_string(selected_frame_);
        root[frame_key] = frame_data;

        // Write updated JSON to file
        std::ofstream output_file(json_path);
        Json::StyledWriter writer;
        output_file << writer.write(root);
        output_file.close();
    }

    // Return all the points which are max_radius away in the x and y from the given coordinate
    std::vector<pcl::PointXYZI> LabelPanel::getPointCloud2AssociatedWithCluster(const std::string& name) {
        // Retrieve cluster message
        const auto& cluster = cluster_map_.at(name);
        // Convert PointCloud2 → PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(cluster.pointcloud, pcl_cloud);

        // Convert to std::vector<pcl::PointXYZI>
        std::vector<pcl::PointXYZI> points;
        points.reserve(pcl_cloud.points.size());

        for (const auto& p : pcl_cloud.points) {
            points.push_back(p);
        }

        return points;
    }


    // Create an interactive marker by name with a box visualizer, with 3d controls and with the menu defined in the
    // menu handler
    void LabelPanel::makeMenuMarker(std::string name, float x, float y, float z) {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "labeling_pointcloud";
        int_marker.scale = 1;

        int_marker.pose.position.x = x;
        int_marker.pose.position.y = y;
        int_marker.pose.position.z = z;

        int_marker.name = name;
        int_marker.description = name;

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
        control.name = "menu_only_control";

        visualization_msgs::msg::Marker marker = makeBox(int_marker);
        control.markers.push_back(marker);
        control.always_visible = true;
        int_marker.controls.push_back(control);

        // Add 3 degrees of freedom to the interactive marker control
        visualization_msgs::msg::InteractiveMarkerControl move_x;
        move_x.name = "move_x";
        move_x.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        move_x.orientation.x = 1;

        visualization_msgs::msg::InteractiveMarkerControl move_y;
        move_y.name = "move_x";
        move_y.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        move_y.orientation.y = 1;

        visualization_msgs::msg::InteractiveMarkerControl move_z;
        move_z.name = "move_x";
        move_z.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        move_z.orientation.z = 1;

        int_marker.controls.push_back(move_x);
        int_marker.controls.push_back(move_y);
        int_marker.controls.push_back(move_z);

        // Insert the marker in the interactive marker server and in the menu handler and apply the menu callback
        // function
        server_->insert(int_marker);
        server_->setCallback(int_marker.name, std::bind(&LabelPanel::iMenuCallback, this, std::placeholders::_1));
        menu_handler_.apply(*server_, int_marker.name);
    }

    visualization_msgs::msg::Marker LabelPanel::makeBox(visualization_msgs::msg::InteractiveMarker& msg) {
        visualization_msgs::msg::Marker marker;

        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = msg.scale * 0.45;
        marker.scale.y = msg.scale * 0.45;
        marker.scale.z = msg.scale * 0.45;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.8;

        return marker;
    }

    void LabelPanel::setMarkerColor(std::string int_marker_name, float r, float g, float b) {
        visualization_msgs::msg::InteractiveMarker int_marker;
        server_->get(int_marker_name, int_marker);

        int_marker.controls[0].markers[0].color.r = r;
        int_marker.controls[0].markers[0].color.g = g;
        int_marker.controls[0].markers[0].color.b = b;

        server_->insert(int_marker);
        server_->applyChanges();
    }

    void LabelPanel::iMenuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
        // If the marker was moved by the user, then update its centroid information in the cluster map
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
            auto c = cluster_map_[feedback->marker_name];
            visualization_msgs::msg::InteractiveMarker int_marker;
            server_->get(feedback->marker_name, int_marker);
            c.centroid.x = int_marker.pose.position.x;
            c.centroid.y = int_marker.pose.position.y;
            c.centroid.z = int_marker.pose.position.z;

            cluster_map_[feedback->marker_name] = c;
        }

        // If the user interacted with the menu on the interactive marker check for the menu entry and handle
        // accordingly
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT) {
            // Blue cone
            if (feedback->menu_entry_id == 1) {
                auto c = cluster_map_[feedback->marker_name];
                c.type = custom_msgs::msg::Cone::BLUE;
                cluster_map_[feedback->marker_name] = c;
                setMarkerColor(feedback->marker_name, 0.0, 0.4, 1.0);
            }

            // Yellow cone
            else if (feedback->menu_entry_id == 2) {
                auto c = cluster_map_[feedback->marker_name];
                c.type = custom_msgs::msg::Cone::YELLOW;
                cluster_map_[feedback->marker_name] = c;
                setMarkerColor(feedback->marker_name, 1.0, 1.0, 0.0);
            }

            // Orange cone
            else if (feedback->menu_entry_id == 3) {
                auto c = cluster_map_[feedback->marker_name];
                c.type = custom_msgs::msg::Cone::ORANGE;
                cluster_map_[feedback->marker_name] = c;
                setMarkerColor(feedback->marker_name, 1.0, 0.4, 0.0);
            }

            // Unknown cone
            else if (feedback->menu_entry_id == 4) {
                auto c = cluster_map_[feedback->marker_name];
                c.type = custom_msgs::msg::Cone::UNKNOWN;
                cluster_map_[feedback->marker_name] = c;
                setMarkerColor(feedback->marker_name, 1.0, 1.0, 1.0);
            }

            // False positive
            else if (feedback->menu_entry_id == 5) {
                auto c = cluster_map_[feedback->marker_name];
                c.type = -1;
                cluster_map_[feedback->marker_name] = c;
                setMarkerColor(feedback->marker_name, 1.0, 0.0, 0.0);
            }

            // Delete cone
            else if (feedback->menu_entry_id == 6) {
                removeCluster(feedback->marker_name);
                cluster_map_.erase(feedback->marker_name);
            }

            // Update the UI
            updateSuggestedClustersRemaining();
        }
    }

    LabelTool::LabelTool() : rviz_common::Tool() {
        projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    void LabelTool::onInitialize() {
        raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
        publisher_ = raw_node->create_publisher<geometry_msgs::msg::Point>("add_cluster_near_point", 10);
        RCLCPP_INFO(raw_node->get_logger(), "Init");
    }

    int LabelTool::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
        if (event.leftUp()) {
            auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
                event.panel->getRenderWindow(), event.x, event.y);
            RCLCPP_INFO(raw_node->get_logger(), std::to_string(point_projection_on_xy_plane.second.x).c_str());
            geometry_msgs::msg::Point point{};
            point.x = point_projection_on_xy_plane.second.x;
            point.y = point_projection_on_xy_plane.second.y;
            publisher_->publish(point);
        }
        return 0;
    }

    void LabelTool::activate() { setStatus("Click and drag mouse to add a new label"); }

    void LabelTool::deactivate() {}

}  // namespace rviz_interface_plugin

PLUGINLIB_EXPORT_CLASS(rviz_interface_plugin::LabelPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(rviz_interface_plugin::LabelTool, rviz_common::Tool)
