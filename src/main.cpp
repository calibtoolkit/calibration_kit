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

#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <camodocal/chessboard/Chessboard.h>
#include <camodocal/calib/CameraCalibration.h>
#include <camodocal/gpl/gpl.h>

#include <camodocal/calib/StereoCameraCalibration.h>

#include "tui_tools.h"

struct LidarToCamera : public BaseTUI
{
    LidarToCamera()
        : BaseTUI("Back")
        , have_timestamps{ true }
        , check_with_timestamps{ ftxui::Checkbox(" pcd with timestamps", &have_timestamps) }
        , start{ ftxui::Button("Start", [this]{ calibration(); }) }
        , layout{ ftxui::Container::Vertical({
            pcd_file,
            img_file,
            cam_intrinsic_file,
            cam_lidar_extrinsic_file,
            check_with_timestamps,
            ftxui::Container::Horizontal({start, quit})
        }) }
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({
                ftxui::hbox({ftxui::text(" pcd file name                 : "), pcd_file.Render() | ftxui::flex}),
                ftxui::hbox({ftxui::text(" img file name                 : "), img_file.Render() | ftxui::flex}),
                ftxui::hbox({ftxui::text(" cam intrinsic file name       : "), cam_intrinsic_file.Render() | ftxui::flex}),
                ftxui::hbox({ftxui::text(" cam lidar extrinsic file name : "), cam_lidar_extrinsic_file.Render() | ftxui::flex}),
                check_with_timestamps->Render(),
                ftxui::separator(),
                ftxui::hbox({start->Render(), ftxui::filler() | ftxui::flex, quit->Render()}),
            }) | ftxui::border;
        }));
    }

    void calibration()
    {
        if(!(pcd_file && img_file && cam_intrinsic_file && cam_lidar_extrinsic_file))
            return;

        if (have_timestamps)
        {
            pcl::PointCloud<kit::tools::PointXYZIT>::Ptr cloud(new pcl::PointCloud<kit::tools::PointXYZIT>());
            if (pcl::io::loadPCDFile(pcd_file.raw_input, *cloud) == -1)
            {
                pcd_file.notice = "Couldn't read this file";
                return;
            }
            std::cout<<"\n\n";
            kit::tools::LidarCameraCalib{cam_intrinsic_file.raw_input, cam_lidar_extrinsic_file.raw_input}.StartCalib(cv::imread(img_file.raw_input), cloud);
        }
        else
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            if (pcl::io::loadPCDFile(pcd_file.raw_input, *cloud) == -1)
            {
                pcd_file.notice = "Couldn't read this file";
                return;
            }
            std::cout<<"\n\n";
            kit::tools::LidarCameraCalib{cam_intrinsic_file.raw_input, cam_lidar_extrinsic_file.raw_input}.StartCalib(cv::imread(img_file.raw_input), cloud);
        }
        std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    bool have_timestamps;
    InputFile pcd_file;
    InputFile img_file;
    InputFile cam_intrinsic_file;
    InputFile cam_lidar_extrinsic_file;
    ftxui::Component check_with_timestamps;
    ftxui::Component start;
    ftxui::Component layout;
};

struct BaseCamera : public BaseTUI
{
    BaseCamera()
        : BaseTUI("Back")
        , width{9}
        , height{6}
        , square_size{70}
        , camera_model_value{1}
        , useOpenCV{true}
        , viewResults{false}
        , verbose{true}
        , check_use_openCV{ftxui::Checkbox("Use OpenCV  ", &useOpenCV)}
        , check_view_results{ftxui::Checkbox("View Results  ", &viewResults)}
        , check_verbose{ftxui::Checkbox("verbose  ", &verbose)}
        , camera_model_list{" kannala-brandt ", " mei ", " pinhole ", " scaramuzza "}
        , toggle_camera_model{ftxui::Toggle(&camera_model_list, &camera_model_value)}
    {}

    bool checkInput()
    { return input_dir && width && height && square_size; }

    InputHelp<int> width;
    InputHelp<int> height;
    InputHelp<float> square_size;
    InputDir input_dir;
    InputHelp<std::string> file_extension;
    std::vector<std::string> camera_model_list;
    int camera_model_value;
    bool useOpenCV;
    bool viewResults;
    bool verbose;

    ftxui::Component check_use_openCV;
    ftxui::Component check_view_results;
    ftxui::Component check_verbose;
    ftxui::Component toggle_camera_model;
};

struct Camera : public BaseCamera
{
    Camera()
        : BaseCamera()
        , start{ftxui::Button("Start", [this]{calibration();})}
        , layout{ftxui::Container::Vertical({
                input_dir,
                ftxui::Container::Horizontal({file_extension, prefix, camera_name}),
                ftxui::Container::Horizontal({width, height, square_size}),
                toggle_camera_model,
                ftxui::Container::Horizontal({check_use_openCV, check_view_results, check_verbose}),
                ftxui::Container::Horizontal({start, quit})
            })}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({
                ftxui::hbox({ftxui::text(" Input directory : "), input_dir.Render() | ftxui::flex}),
                ftxui::hbox({ftxui::text(" file-extension : "), file_extension.Render() | ftxui::flex, ftxui::text(" prefix : "), prefix.Render() | ftxui::flex, ftxui::text(" camera_name : "), camera_name.Render() | ftxui::flex,}),
                ftxui::hbox({ftxui::text(" width : "), width.Render() | ftxui::flex, ftxui::text(" height : "), height.Render() | ftxui::flex, ftxui::text(" square_size : "), square_size.Render() | ftxui::flex,}),
                ftxui::hbox({toggle_camera_model->Render(), ftxui::text("  "), check_use_openCV->Render(), check_view_results->Render(), check_verbose->Render()}),
                ftxui::separator(),
                ftxui::hbox({start->Render(), ftxui::filler() | ftxui::flex, quit->Render()}),
            }) | ftxui::border;
        }));
    }

    void calibration()
    {
        if(!checkInput())
            return;

        camodocal::Camera::ModelType modelType;
        switch (camera_model_value)
        {
        case 0:
            modelType = camodocal::Camera::KANNALA_BRANDT;
            break;
        case 1:
            modelType = camodocal::Camera::MEI;
            break;
        case 2:
            modelType = camodocal::Camera::PINHOLE;
            break;
        case 3:
            modelType = camodocal::Camera::SCARAMUZZA;
            break;
        default:
            std::cerr << "# ERROR: Unknown camera model" << std::endl;
            return;
        }

        // look for images in input directory
        std::vector<std::string> imageFilenames;
        for (std::filesystem::directory_iterator itr(input_dir.raw_input); itr != std::filesystem::directory_iterator(); ++itr)
        {
            if (!std::filesystem::is_regular_file(itr->status()))
            {
                continue;
            }

            std::string filename = itr->path().filename().string();

            std::cout<<filename<<std::endl;

            // check if prefix matches
            if (!prefix.raw_input.empty())
            {
                if (filename.compare(0, prefix.raw_input.length(), prefix.raw_input) != 0)
                {
                    continue;
                }
            }

            // check if file extension matches
            if (filename.compare(filename.length() - file_extension.raw_input.length(), file_extension.raw_input.length(), file_extension.raw_input) != 0)
            {
                continue;
            }

            imageFilenames.push_back(itr->path().string());

            if (verbose)
            {
                std::cerr << "# INFO: Adding " << imageFilenames.back() << std::endl;
            }
        }

        if (imageFilenames.empty())
        {
            std::cerr << "# ERROR: No chessboard images found." << std::endl;
            return;
        }

        if (verbose)
        {
            std::cerr << "# INFO: # images: " << imageFilenames.size() << std::endl;
        }

        cv::Size boardSize;
        boardSize.width = width.value;
        boardSize.height = height.value;

        std::sort(imageFilenames.begin(), imageFilenames.end());

        cv::Mat image = cv::imread(imageFilenames.front(), -1);
        const cv::Size frameSize = image.size();

        camodocal::CameraCalibration calibration(modelType, camera_name.raw_input, frameSize, boardSize, square_size.value);
        calibration.setVerbose(verbose);

        std::vector<bool> chessboardFound(imageFilenames.size(), false);
        for (size_t i = 0; i < imageFilenames.size(); ++i)
        {
            image = cv::imread(imageFilenames.at(i), -1);

            camodocal::Chessboard chessboard(boardSize, image);

            chessboard.findCorners(useOpenCV);
            if (chessboard.cornersFound())
            {
                if (verbose)
                {
                    std::cerr << "# INFO: Detected chessboard in image " << i + 1 << ", " << imageFilenames.at(i) << std::endl;
                }

                calibration.addChessboardData(chessboard.getCorners());

                cv::Mat sketch;
                chessboard.getSketch().copyTo(sketch);

                cv::imshow("Image", sketch);
                cv::waitKey(50);
            }
            else if (verbose)
            {
                std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
            }
            chessboardFound.at(i) = chessboard.cornersFound();
        }
        cv::destroyWindow("Image");

        if (calibration.sampleCount() < 10)
        {
            std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
            return;
        }

        if (verbose)
        {
            std::cerr << "# INFO: Calibrating..." << std::endl;
        }

        double startTime = camodocal::timeInSeconds();

        calibration.calibrate();
        calibration.writeParams(camera_name.raw_input + "_camera_calib.yaml");
        calibration.writeChessboardData(camera_name.raw_input + "_chessboard_data.dat");

        if (verbose)
        {
            std::cout << "# INFO: Calibration took a total time of "
                      << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                      << " sec.\n";
        }

        if (verbose)
        {
            std::cerr << "# INFO: Wrote calibration file to " << camera_name.raw_input + "_camera_calib.yaml" << std::endl;
        }

        if (viewResults)
        {
            std::vector<cv::Mat> cbImages;
            std::vector<std::string> cbImageFilenames;

            for (size_t i = 0; i < imageFilenames.size(); ++i)
            {
                if (!chessboardFound.at(i))
                {
                    continue;
                }

                cbImages.push_back(cv::imread(imageFilenames.at(i), -1));
                cbImageFilenames.push_back(imageFilenames.at(i));
            }

            // visualize observed and reprojected points
            calibration.drawResults(cbImages);

            for (size_t i = 0; i < cbImages.size(); ++i)
            {
                cv::putText(cbImages.at(i), cbImageFilenames.at(i), cv::Point(10,20),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                            1, CV_AA);
                cv::imshow("Image", cbImages.at(i));
                cv::waitKey(0);
            }
            cv::destroyWindow("Image");
            std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        }
    }

    InputHelp<std::string> prefix;
    InputHelp<std::string> camera_name;
    ftxui::Component start;
    ftxui::Component layout;
};

struct CameraToCamera : public BaseCamera
{
    CameraToCamera()
        : BaseCamera()
        , start{ftxui::Button("Start", [this]{calibration();})}
        , layout{ftxui::Container::Vertical({
                input_dir,
                ftxui::Container::Horizontal({file_extension, output_dir}),
                ftxui::Container::Horizontal({prefix_left, camera_name_left}),
                ftxui::Container::Horizontal({prefix_right, camera_name_right}),
                ftxui::Container::Horizontal({width, height, square_size}),
                toggle_camera_model,
                ftxui::Container::Horizontal({check_use_openCV, check_view_results, check_verbose}),
                ftxui::Container::Horizontal({start, quit})
            })}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({
                ftxui::hbox({ftxui::text(" Input directory : "), input_dir.Render() | ftxui::flex}),
                ftxui::hbox({ftxui::text(" file-extension : "), file_extension.Render() | ftxui::flex, ftxui::text(" Output directory : "), output_dir.Render() | ftxui::flex,}),
                ftxui::hbox({ftxui::text(" prefix_left : "), prefix_left.Render() | ftxui::flex, ftxui::text(" camera_name_left : "), camera_name_left.Render() | ftxui::flex, }),
                ftxui::hbox({ftxui::text(" prefix_right : "), prefix_right.Render() | ftxui::flex, ftxui::text(" camera_name_right : "), camera_name_right.Render() | ftxui::flex, }),
                ftxui::hbox({ftxui::text(" width : "), width.Render() | ftxui::flex, ftxui::text("height : "), height.Render() | ftxui::flex, ftxui::text("square_size : "), square_size.Render() | ftxui::flex,}),
                ftxui::hbox({toggle_camera_model->Render(), ftxui::text("  "), check_use_openCV->Render(), check_view_results->Render(), check_verbose->Render()}),
                ftxui::separator(),
                ftxui::hbox({start->Render(), ftxui::filler() | ftxui::flex, quit->Render()}),
            }) | ftxui::border;
        }));
    }

    void calibration()
    {
        if(!checkInput())
            return;

        camodocal::Camera::ModelType modelType;
        switch (camera_model_value)
        {
            case 0:
                modelType = camodocal::Camera::KANNALA_BRANDT;
                break;
            case 1:
                modelType = camodocal::Camera::MEI;
                break;
            case 2:
                modelType = camodocal::Camera::PINHOLE;
                break;
            case 3:
                modelType = camodocal::Camera::SCARAMUZZA;
                break;
            default:
                std::cerr << "# ERROR: Unknown camera model" << std::endl;
                return;
        }

        // look for images in input directory
        std::vector<std::string> imageFilenamesL, imageFilenamesR;
        for (boost::filesystem::directory_iterator itr(input_dir.raw_input); itr != boost::filesystem::directory_iterator(); ++itr)
        {
            if (!boost::filesystem::is_regular_file(itr->status()))
            {
                continue;
            }

            std::string filename = itr->path().filename().string();

            // check if file extension matches
            if (filename.compare(filename.length() - file_extension.raw_input.length(), file_extension.raw_input.length(), file_extension.raw_input) != 0)
            {
                continue;
            }

            // check if prefix matches
            if (prefix_left.raw_input.empty() || (!prefix_left.raw_input.empty() && (filename.compare(0, prefix_left.raw_input.length(), prefix_left.raw_input) == 0)))
            {
                imageFilenamesL.push_back(itr->path().string());

                if (verbose)
                {
                    std::cerr << "# INFO: Adding " << imageFilenamesL.back() << std::endl;
                }
            }
            if (prefix_right.raw_input.empty() || (!prefix_right.raw_input.empty() && (filename.compare(0, prefix_right.raw_input.length(), prefix_right.raw_input) == 0)))
            {
                imageFilenamesR.push_back(itr->path().string());

                if (verbose)
                {
                    std::cerr << "# INFO: Adding " << imageFilenamesR.back() << std::endl;
                }
            }
        }

        if (imageFilenamesL.empty() || imageFilenamesR.empty())
        {
            std::cerr << "# ERROR: No chessboard images found." << std::endl;
            return;
        }

        if (imageFilenamesL.size() != imageFilenamesR.size())
        {
            std::cerr << "# ERROR: # chessboard images from left and right cameras do not match." << std::endl;
            return;
        }

        cv::Size boardSize;
        boardSize.width = width.value;
        boardSize.height = height.value;

        bool matchImages = true;
        std::sort(imageFilenamesL.begin(), imageFilenamesL.end());
        std::sort(imageFilenamesR.begin(), imageFilenamesR.end());

        for (size_t i = 0; i < imageFilenamesL.size(); ++i)
        {
            std::string filenameL = boost::filesystem::path(imageFilenamesL.at(i)).filename().string();
            std::string filenameR = boost::filesystem::path(imageFilenamesR.at(i)).filename().string();

            if (filenameL.compare(prefix_left.raw_input.length(),
                                  filenameL.size() - prefix_left.raw_input.length(),
                                  filenameR,
                                  prefix_right.raw_input.length(),
                                  filenameR.size() - prefix_right.raw_input.length()) != 0)
            {
                matchImages = false;

                if (verbose)
                {
                    std::cerr << "# ERROR: Filenames do not match: "
                              << imageFilenamesL.at(i) << " " << imageFilenamesR.at(i) << std::endl;
                }
            }
        }

        if (!matchImages)
        {
            return;
        }

        if (verbose)
        {
            std::cerr << "# INFO: # images: " << imageFilenamesL.size() << std::endl;
        }

        cv::Mat imageL = cv::imread(imageFilenamesL.front(), -1);
        cv::Mat imageR;
        const cv::Size frameSize = imageL.size();

        camodocal::StereoCameraCalibration calibration(modelType, camera_name_left.raw_input, camera_name_right.raw_input, frameSize, boardSize, square_size.value);
        calibration.setVerbose(verbose);

        std::vector<bool> chessboardFoundL(imageFilenamesL.size(), false);
        std::vector<bool> chessboardFoundR(imageFilenamesR.size(), false);
        for (size_t i = 0; i < imageFilenamesL.size(); ++i)
        {
            imageL = cv::imread(imageFilenamesL.at(i), -1);
            imageR = cv::imread(imageFilenamesR.at(i), -1);

            camodocal::Chessboard chessboardL(boardSize, imageL);
            camodocal::Chessboard chessboardR(boardSize, imageR);

            chessboardL.findCorners(useOpenCV);
            chessboardR.findCorners(useOpenCV);
            if (chessboardL.cornersFound() && chessboardR.cornersFound())
            {
                if (verbose)
                {
                    std::cerr << "# INFO: Detected chessboard in image " << i + 1 << std::endl;
                }

                calibration.addChessboardData(chessboardL.getCorners(),
                                              chessboardR.getCorners());

                cv::Mat sketch;
                chessboardL.getSketch().copyTo(sketch);

                cv::imshow("Image - Left", sketch);

                chessboardR.getSketch().copyTo(sketch);

                cv::imshow("Image - Right", sketch);

                cv::waitKey(50);
            }
            else if (verbose)
            {
                std::cerr << "# INFO: Did not detect chessboard in image " << i + 1 << std::endl;
            }
            chessboardFoundL.at(i) = chessboardL.cornersFound();
            chessboardFoundR.at(i) = chessboardR.cornersFound();
        }
        cv::destroyWindow("Image - Left");
        cv::destroyWindow("Image - Right");

        if (calibration.sampleCount() < 10)
        {
            std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
            return;
        }

        if (verbose)
        {
            std::cerr << "# INFO: Calibrating..." << std::endl;
        }

        double startTime = camodocal::timeInSeconds();

        calibration.calibrate();
        calibration.writeParams(output_dir.raw_input);

        if (verbose)
        {
            std::cout << "# INFO: Calibration took a total time of "
                      << std::fixed << std::setprecision(3) << camodocal::timeInSeconds() - startTime
                      << " sec.\n";
        }

        if (verbose)
        {
            std::cerr << "# INFO: Wrote calibration files to " << boost::filesystem::absolute(output_dir.raw_input).string() << std::endl;
        }

        if (viewResults)
        {
            std::vector<cv::Mat> cbImagesL, cbImagesR;
            std::vector<std::string> cbImageFilenamesL;
            std::vector<std::string> cbImageFilenamesR;

            for (size_t i = 0; i < imageFilenamesL.size(); ++i)
            {
                if (!chessboardFoundL.at(i) || !chessboardFoundR.at(i))
                {
                    continue;
                }

                cbImagesL.push_back(cv::imread(imageFilenamesL.at(i), -1));
                cbImageFilenamesL.push_back(imageFilenamesL.at(i));

                cbImagesR.push_back(cv::imread(imageFilenamesR.at(i), -1));
                cbImageFilenamesR.push_back(imageFilenamesR.at(i));
            }

            // visualize observed and reprojected points
            calibration.drawResults(cbImagesL, cbImagesR);

            for (size_t i = 0; i < cbImagesL.size(); ++i)
            {
                cv::putText(cbImagesL.at(i), cbImageFilenamesL.at(i), cv::Point(10,20),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                            1, CV_AA);
                cv::imshow("Image - Left", cbImagesL.at(i));
                cv::putText(cbImagesR.at(i), cbImageFilenamesR.at(i), cv::Point(10,20),
                            cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255),
                            1, CV_AA);
                cv::imshow("Image - Right", cbImagesR.at(i));
                cv::waitKey(0);
            }
            cv::destroyWindow("Image - Left");
            cv::destroyWindow("Image - Right");
            std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        }
    }

    InputHelp<std::string> prefix_left;
    InputHelp<std::string> prefix_right;
    InputHelp<std::string> camera_name_left;
    InputHelp<std::string> camera_name_right;
    InputDir output_dir;
    ftxui::Component start;
    ftxui::Component layout;
};

void lidarToIMUCalibration(const std::string& conf_file);

struct LidarToIMU : public BaseTUI
{
    LidarToIMU()
            :BaseTUI("Back")
            ,start{ftxui::Button("Start", [this]{calibration();})}
            ,layout{ftxui::Container::Vertical({
                                                       config_file,
                                                       ftxui::Container::Horizontal({start, quit})
                                               })}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ ftxui::hbox({ftxui::text(" config file name : "), config_file.Render() | ftxui::flex}),
                                 ftxui::separator(),
                                 ftxui::hbox({start->Render(), ftxui::filler() | ftxui::flex, quit->Render()}),
                               }) | ftxui::border;
        }));
    }

    void calibration()
    {
        std::cout<<"\n\n";
        lidarToIMUCalibration(config_file.raw_input);
        std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    InputFile config_file;
    ftxui::Component start;
    ftxui::Component layout;
};

void lidar2lidarCalibration(const std::string& path1, const std::string& path2);

struct LidarToLidar : public BaseTUI
{
    LidarToLidar()
            :BaseTUI("Back")
            ,start{ftxui::Button("Start", [this]{calibration();})}
            ,layout{ftxui::Container::Vertical({
                                                       source_pcd_file,
                                                       target_pcd_file,
                                                       ftxui::Container::Horizontal({start, quit})
                                               })}
    {}

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ ftxui::hbox({ftxui::text(" source pcd file name : "), source_pcd_file.Render() | ftxui::flex}),
                                 ftxui::hbox({ftxui::text(" target pcd file name : "), target_pcd_file.Render() | ftxui::flex}),
                                 ftxui::separator(),
                                 ftxui::hbox({start->Render(), ftxui::filler() | ftxui::flex, quit->Render()}),
                               }) | ftxui::border;
        }));
    }

    void calibration()
    {
        std::cout<<"\n\n";
        lidar2lidarCalibration(source_pcd_file.raw_input, target_pcd_file.raw_input);
        std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    InputFile source_pcd_file;
    InputFile target_pcd_file;
    ftxui::Component start;
    ftxui::Component layout;
};

void nonlinear_opt_demo();

struct NonlinearOptDemoTUI : public BaseTUI
{
    NonlinearOptDemoTUI()
            :BaseTUI("Back")
            ,layout{ftxui::Container::Horizontal({quit})}
    {
        std::cout<<"\n\n";
        nonlinear_opt_demo();
        std::cout<<"\n\n\n\n\n\n\n\n\n\n\n\n";
    }

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return quit->Render() | ftxui::border | ftxui::center;
        }));
    }

    ftxui::Component layout;
};



struct CalibrationTUI : public BaseTUI
{
    CalibrationTUI()
        :BaseTUI()
        ,camera{ftxui::Button(" Camera ", []{Camera{}.show();})}
        ,camera2camera{ftxui::Button(" Camera To Camera ", []{CameraToCamera{}.show();})}
        ,lidar2camera{ftxui::Button(" Lidar To Camera ", []{LidarToCamera{}.show();})}
        ,lidar2lidar{ftxui::Button(" Lidar To Lidar ", []{LidarToLidar{}.show();})}
        ,lidar2imu{ftxui::Button(" Lidar To IMU ", []{LidarToIMU{}.show();})}
        ,nonlinear{ftxui::Button(" Nonlinear Optimization Demo ", []{NonlinearOptDemoTUI{}.show();})}
        ,layout{ftxui::Container::Horizontal({camera, camera2camera, lidar2camera, lidar2lidar, lidar2imu, nonlinear, quit})}
    { }

    void show()
    {
        screen.Loop(ftxui::Renderer(layout,[this]
        {
            return ftxui::vbox({ftxui::text("Calibration Kit") | ftxui::center,
                ftxui::separator(),
                ftxui::hbox({camera->Render(), camera2camera->Render(), lidar2camera->Render(), lidar2lidar->Render(), lidar2imu->Render(), nonlinear->Render()}),
                ftxui::separator(),
                quit->Render() | ftxui::center,
            }) | ftxui::border | ftxui::center;
        }));
    }

    std::string a;
    ftxui::Component camera;
    ftxui::Component camera2camera;
    ftxui::Component lidar2camera;
    ftxui::Component lidar2lidar;
    ftxui::Component lidar2imu;
    ftxui::Component nonlinear;
    ftxui::Component layout;
};

int main()
{
    CalibrationTUI{}.show();
}
