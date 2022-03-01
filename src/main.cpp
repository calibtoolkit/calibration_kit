#include <functional>  // for function
#include <memory>      // for allocator, __shared_ptr_access
#include <string>      // for string, basic_string, operator+, to_string
#include <vector>      // for vector
#include <filesystem>

#include "ftxui/component/captured_mouse.hpp"  // for ftxui
#include "ftxui/component/component.hpp"       // for Menu, Horizontal, Renderer
#include "ftxui/component/component_base.hpp"  // for ComponentBase
#include "ftxui/component/component_options.hpp"  // for MenuOption
#include "ftxui/component/screen_interactive.hpp"  // for Component, ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for text, separator, bold, hcenter, vbox, hbox, gauge, Element, operator|, border

#include <lidar_camera_calib.hpp>

struct BaseTUI
{
    BaseTUI(const std::string& quit_button_name = "Quit")
            :screen{ftxui::ScreenInteractive::TerminalOutput()}
            ,quit{ftxui::Button(quit_button_name, screen.ExitLoopClosure())}
    {}

    ftxui::ScreenInteractive screen;
    ftxui::Component quit;
};

struct ToDoTUI : public BaseTUI
{
    ToDoTUI()
        :BaseTUI("Back")
        ,layout{ftxui::Container::Horizontal({quit})}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ftxui::text("TODO"), quit->Render()}) | ftxui::border;
        }));
    }

    ftxui::Component layout;
};

struct LidarToCamera : public BaseTUI
{
    LidarToCamera()
            :BaseTUI("Back")
            ,have_timestamps(true)
            ,input_pcd_file_name{ftxui::Input(&pcd_file_name, "")}
            ,input_img_file_name{ftxui::Input(&img_file_name, "")}
            ,input_cam_intrinsic_file_name{ftxui::Input(&cam_intrinsic_file_name, "")}
            ,input_cam_lidar_extrinsic_file_name{ftxui::Input(&cam_lidar_extrinsic_file_name, "")}
            ,check_with_timestamps{ftxui::Checkbox(" pcd with timestamps", &have_timestamps)}
            ,start{ftxui::Button("Start", [this]{calibration();})}
            ,layout{ftxui::Container::Vertical({
                input_pcd_file_name,
                input_img_file_name,
                input_cam_intrinsic_file_name,
                input_cam_lidar_extrinsic_file_name,
                check_with_timestamps,
                ftxui::Container::Horizontal({start, quit})
            })}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            bool check_flag = !(pcd_file_name.empty() || img_file_name.empty() || cam_intrinsic_file_name.empty() || cam_lidar_extrinsic_file_name.empty());
            if (!(pcd_file_name.empty() || std::filesystem::exists(pcd_file_name)))
                pcd_file_notice_text = ftxui::text("File does not exist"), check_flag=false;
            else
                pcd_file_notice_text = ftxui::text("");
            if (!(img_file_name.empty() || std::filesystem::exists(img_file_name)))
                img_file_notice_text = ftxui::text("File does not exist"), check_flag=false;
            else
                img_file_notice_text = ftxui::text("");
            if (!(cam_intrinsic_file_name.empty() || std::filesystem::exists(cam_intrinsic_file_name)))
                intrinsic_file_notice_text = ftxui::text("File does not exist"), check_flag=false;
            else
                intrinsic_file_notice_text = ftxui::text("");
            if (!(cam_lidar_extrinsic_file_name.empty() || std::filesystem::exists(cam_lidar_extrinsic_file_name)))
                extrinsic_file_notice_text = ftxui::text("File does not exist"), check_flag=false;
            else
                extrinsic_file_notice_text = ftxui::text("");

            ftxui::Element start_agent;
            if(check_flag)
                start_agent = start->Render();
            else
                start_agent = ftxui::text("Start") | ftxui::border | ftxui::dim;
            start_agent = start->Render();

            return ftxui::vbox({
                ftxui::hbox({ftxui::text(" pcd file name                 : "), input_pcd_file_name->Render() | ftxui::flex, pcd_file_notice_text | ftxui::color(ftxui::Color::Red)}),
                ftxui::hbox({ftxui::text(" img file name                 : "), input_img_file_name->Render() | ftxui::flex, img_file_notice_text | ftxui::color(ftxui::Color::Red)}),
                ftxui::hbox({ftxui::text(" cam intrinsic file name       : "), input_cam_intrinsic_file_name->Render() | ftxui::flex, intrinsic_file_notice_text | ftxui::color(ftxui::Color::Red)}),
                ftxui::hbox({ftxui::text(" cam lidar extrinsic file name : "), input_cam_lidar_extrinsic_file_name->Render() | ftxui::flex, extrinsic_file_notice_text | ftxui::color(ftxui::Color::Red)}),
                check_with_timestamps->Render(),
                ftxui::separator(),
                ftxui::hbox({start_agent, ftxui::filler() | ftxui::flex, quit->Render()}),
            }) | ftxui::border;
        }));
    }

    void calibration()
    {
        pcd_file_name="/home/incloon/Develop/lidar_camera_calib/test/1.pcd";
        img_file_name="/home/incloon/Develop/lidar_camera_calib/test/1.jpeg";
        cam_intrinsic_file_name="/home/incloon/Develop/lidar_camera_calib/test/front_6mm_intrinsics.yaml";
        cam_lidar_extrinsic_file_name="/home/incloon/Develop/lidar_camera_calib/test/front_6mm_extrinsics.yaml";
        cv::Mat image = cv::imread(img_file_name);
        if (have_timestamps)
        {
            pcl::PointCloud<kit::tools::PointXYZIT>::Ptr cloud(new pcl::PointCloud<kit::tools::PointXYZIT>());
            if (pcl::io::loadPCDFile(pcd_file_name, *cloud) == -1)
            {
                pcd_file_notice_text = ftxui::text("Couldn't read this file");
                return;
            }
            kit::tools::LidarCameraCalib{cam_intrinsic_file_name, cam_lidar_extrinsic_file_name}.StartCalib(image, cloud);
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            if (pcl::io::loadPCDFile(pcd_file_name, *cloud) == -1)
            {
                pcd_file_notice_text = ftxui::text("Couldn't read this file");
                return;
            }
            kit::tools::LidarCameraCalib{cam_intrinsic_file_name, cam_lidar_extrinsic_file_name}.StartCalib(image, cloud);
        }
        std::cout<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
    }

    bool have_timestamps;
    std::string pcd_file_name;
    std::string img_file_name;
    std::string cam_intrinsic_file_name;
    std::string cam_lidar_extrinsic_file_name;
    ftxui::Component input_pcd_file_name;
    ftxui::Component input_img_file_name;
    ftxui::Component input_cam_intrinsic_file_name;
    ftxui::Component input_cam_lidar_extrinsic_file_name;
    ftxui::Component check_with_timestamps;
    ftxui::Component start;
    ftxui::Component layout;
    ftxui::Element pcd_file_notice_text;
    ftxui::Element img_file_notice_text;
    ftxui::Element intrinsic_file_notice_text;
    ftxui::Element extrinsic_file_notice_text;
};

struct CalibrationTUI : public BaseTUI
{
    CalibrationTUI()
        :BaseTUI()
        ,camera{ftxui::Button("Camera", []{ToDoTUI{}.show();})}
        ,camera2camera{ftxui::Button("Camera To Camera", []{ToDoTUI{}.show();})}
        ,lidar2camera{ftxui::Button("Lidar To Camera", []{LidarToCamera{}.show();})}
        ,lidar2lidar{ftxui::Button("Lidar To Lidar", []{ToDoTUI{}.show();})}
        ,lidar2imu{ftxui::Button("Lidar To IMU", []{ToDoTUI{}.show();})}
        ,layout{ftxui::Container::Horizontal({camera,camera2camera,lidar2camera,lidar2lidar,lidar2imu,quit})}
    {

    }

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::hbox({
                                    camera->Render(),
                                    camera2camera->Render(),
                                    lidar2camera->Render(),
                                    lidar2lidar->Render(),
                                    lidar2imu->Render(),
                                    quit->Render(),
                               }) | ftxui::border;
        }));
    }

    ftxui::Component camera;
    ftxui::Component camera2camera;
    ftxui::Component lidar2camera;
    ftxui::Component lidar2lidar;
    ftxui::Component lidar2imu;
    ftxui::Component layout;
};

int main()
{
    CalibrationTUI{}.show();
}
