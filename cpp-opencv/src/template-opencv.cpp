/*
 * Copyright (C) 2020 Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"  // For CLUON messaging framework
#include <algorithm>  // For std::sort
#include "opendlv-standard-message-set.hpp"
#include <chrono>  // For time-related operations
#include <iomanip>  // For formatting output
#include <opencv2/core.hpp>  // Core functionality
#include <opencv2/imgproc/imgproc.hpp>  // Image processing
#include <opencv2/highgui/highgui.hpp>  // GUI
#include <experimental/filesystem>
#include <sstream>

namespace fs = std::experimental::filesystem;

void createSliderWindowBlueMask();
void createSliderWindowYellowMask();
int getGaussianKernelSizeDimension();
void createMask(cv::Mat& mask, cv::Scalar minHSV, cv::Scalar maxHSV);
void putFrameInfo(cv::Mat& img, std::string color, int minH, int minS, int minV, int maxH, int maxS, int maxV, std::string timeStamp);
bool writeImageToFolder(cv::Mat& image, std::string& folderPath, std::string& filename);
void processContour(const std::vector<cv::Point>& contour, cv::Mat& image, const cv::Scalar& color, int detection_threshold);
void sortAndProcessContours(std::vector<std::vector<cv::Point>>& contours, cv::Mat& image, const cv::Scalar& color, int detection_threshold);

cv::Mat img, croppedImg, blurredCroppedImg, hsvImage, blueMask, yellowMask;
int blueMinH, blueMaxH, blueMinS, blueMaxS, blueMinV, blueMaxV;
int yellowMinH, yellowMaxH, yellowMinS, yellowMaxS, yellowMinV, yellowMaxV;
int gaussianKernelSize;
int gaussianKernelSizeSlider = 0, gaussianStandardDeviationX = 0, gaussianStandardDeviationY = 0;
std::vector<std::vector<cv::Point>> blueContours, yellowContours;
std::vector<cv::Vec4i> blueHierchy, yellowHierchy;

int main(int argc, char **argv) {
    int retCode = 1;
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (commandlineArguments.count("cid") == 0 || commandlineArguments.count("name") == 0 ||
        commandlineArguments.count("width") == 0 || commandlineArguments.count("height") == 0) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    } else {
        const std::string NAME = commandlineArguments["name"];
        const uint32_t WIDTH = static_cast<uint32_t>(std::stoi(commandlineArguments["width"]));
        const uint32_t HEIGHT = static_cast<uint32_t>(std::stoi(commandlineArguments["height"]));
        const bool VERBOSE = commandlineArguments.count("verbose") != 0;

        std::unique_ptr<cluon::SharedMemory> sharedMemory(new cluon::SharedMemory{NAME});
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;
            cluon::OD4Session od4(static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])));

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            std::string timeStamp;

            auto onGroundSteeringRequest = [&gsr, &gsrMutex, &timeStamp](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                timeStamp = std::to_string(env.sampleTimeStamp().seconds()) + std::to_string(env.sampleTimeStamp().microseconds());
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
            char writeChoice, colorChoice, pauseOnFrameMode;
            std::string folderPath = "output";
            int approach = 0;

            while (!(writeChoice == 'y' || writeChoice == 'n')) {
                std::cout << "Write frames to a folder (y | n)?" << std::endl;
                std::cin >> writeChoice;
            }

            if (writeChoice == 'y') {
                while (!(colorChoice == 'b' || colorChoice == 'y')) {
                    std::cout << "Color to process: Blue (b) | Yellow (y)?" << std::endl;
                    std::cin >> colorChoice;
                }

                if (colorChoice == 'b') {
                    blueMinH = 100; blueMaxH = 120; blueMinS = 50; blueMaxS = 255; blueMinV = 30; blueMaxV = 255;
                    createSliderWindowBlueMask();
                } else {
                    yellowMinH = 20; yellowMaxH = 40; yellowMinS = 60; yellowMaxS = 200; yellowMinV = 70; yellowMaxV = 200;
                    createSliderWindowYellowMask();
                }
            } else {
                blueMinH = 100; blueMaxH = 120; blueMinS = 50; blueMaxS = 255; blueMinV = 30; blueMaxV = 255;
                yellowMinH = 20; yellowMaxH = 40; yellowMinS = 60; yellowMaxS = 200; yellowMinV = 70; yellowMaxV = 200;
                createSliderWindowBlueMask();
                createSliderWindowYellowMask();
            }

            while (!(pauseOnFrameMode == 'y' || pauseOnFrameMode == 'n')) {
                std::cout << "Enable pause-on-frame mode (y | n)?" << std::endl;
                std::cin >> pauseOnFrameMode;
            }

            while (!(approach == 1 || approach == 2)) {
                std::cout << "Choose the contour processing approach:\n";
                std::cout << "1. Direct Iteration\n";
                std::cout << "2. Sorting Method\n";
                std::cin >> approach;
            }

            cv::namedWindow("Blurring Inspector", CV_WINDOW_AUTOSIZE);
            cvCreateTrackbar("Kernel Size Mode", "Blurring Inspector", &gaussianKernelSizeSlider, 30);
            cvCreateTrackbar("Standard Deviation X axis", "Blurring Inspector", &gaussianStandardDeviationX, 9999999);
            cvCreateTrackbar("Standard Deviation Y axis", "Blurring Inspector", &gaussianStandardDeviationY, 9999999);

            while (od4.isRunning()) {
                sharedMemory->wait();
                sharedMemory->lock();
                {
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                auto now = std::chrono::system_clock::now();
                std::time_t now_t = std::chrono::system_clock::to_time_t(now);
                std::tm* now_tm = std::gmtime(&now_t);
                std::ostringstream oss;
                oss << std::put_time(now_tm, "%Y-%m-%dT%H:%M:%SZ");
                std::string utc_time = oss.str();
                std::string imageMessage = "Now: " + utc_time + ";" + " ts: " + timeStamp + ";";

                croppedImg = img(cv::Rect(0, 255, 640, 144));
                gaussianKernelSize = getGaussianKernelSizeDimension();
                cv::GaussianBlur(croppedImg, blurredCroppedImg, cv::Size(gaussianKernelSize, gaussianKernelSize), gaussianStandardDeviationX, gaussianStandardDeviationY);
                blurredCroppedImg.copyTo(hsvImage);
                cv::cvtColor(hsvImage, hsvImage, cv::COLOR_BGR2HSV);

                if (writeChoice == 'y') {
                    if (colorChoice == 'b') {
                        createMask(blueMask, cv::Scalar(blueMinH, blueMinS, blueMinV), cv::Scalar(blueMaxH, blueMaxS, blueMaxV));
                        cv::findContours(blueMask, blueContours, blueHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                        cv::drawContours(croppedImg, blueContours, -1, cv::Scalar(255, 0, 0), 2);
                        putFrameInfo(blueMask, "Blue", blueMinH, blueMinS, blueMinV, blueMaxH, blueMaxS, blueMaxV, timeStamp);
                    } else {
                        createMask(yellowMask, cv::Scalar(yellowMinH, yellowMinS, yellowMinV), cv::Scalar(yellowMaxH, yellowMaxS, yellowMaxV));
                        cv::findContours(yellowMask, yellowContours, yellowHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                        cv::drawContours(croppedImg, yellowContours, -1, cv::Scalar(0, 255, 255), 2);
                        putFrameInfo(yellowMask, "Yellow", yellowMinH, yellowMinS, yellowMinV, yellowMaxH, yellowMaxS, yellowMaxV, timeStamp);
                    }
                } else {
                    createMask(blueMask, cv::Scalar(blueMinH, blueMinS, blueMinV), cv::Scalar(blueMaxH, blueMaxS, blueMaxV));
                    createMask(yellowMask, cv::Scalar(yellowMinH, yellowMinS, yellowMinV), cv::Scalar(yellowMaxH, yellowMaxS, yellowMaxV));
                    cv::findContours(blueMask, blueContours, blueHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                    cv::drawContours(croppedImg, blueContours, -1, cv::Scalar(255, 0, 0), 2);
                    cv::findContours(yellowMask, yellowContours, yellowHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                    cv::drawContours(croppedImg, yellowContours, -1, cv::Scalar(0, 255, 255), 2);
                    putFrameInfo(blueMask, "Blue", blueMinH, blueMinS, blueMinV, blueMaxH, blueMaxS, blueMaxV, timeStamp);
                    putFrameInfo(yellowMask, "Yellow", yellowMinH, yellowMinS, yellowMinV, yellowMaxH, yellowMaxS, yellowMaxV, timeStamp);
                }

                for (int i = 0; i < blueHierchy.size(); i++) {
                    std::cout << "blue hierchy list number: " << i << std::endl;
                    for (int j = 0; j < 4; j++) {
                        std::cout << blueHierchy[i][j] << std::endl;
                    }
                }

                std::cout << std::endl;

                for (int i = 0; i < blueContours.size(); i++) {
                    std::cout << "blue contour : " << i << " points" << std::endl;
                    for (int j = 0; j < blueContours[i].size(); j++) {
                        std::cout << "x: " << blueContours[i][j].x << ", y: " <<  blueContours[i][j].y << std::endl;
                    }
                }

                std::cout << std::endl;

                for (int i = 0; i < blueContours.size(); i++) {
                    std::cout << "blue contour number bounding area: " << i << std::endl;
                    cv::Rect boundingArea = cv::boundingRect(blueContours[i]);
                    std::cout << "area: " << boundingArea.width * boundingArea.height << std::endl;
                }

                std::vector<cv::Moments> muYellow(yellowContours.size());
                std::vector<cv::Moments> muBlue(blueContours.size());
                
                std::vector<cv::Point2f> mcYellow(yellowContours.size());
                std::vector<cv::Point2f> mcBlue(blueContours.size());

                int detection_threshold = 10;

                if (approach == 1) {
                    int max_yellow_contour_area = 0;
                    int index_yellow = -1;
                    for (int i = 0; i < yellowContours.size(); i++) {
                        cv::Rect bounding_rect_yellow = cv::boundingRect(yellowContours[i]);
                        int area = bounding_rect_yellow.width * bounding_rect_yellow.height;
                        if (area > max_yellow_contour_area) {
                            max_yellow_contour_area = area;
                            index_yellow = i;
                        }
                    }

                    int max_blue_contour_area = 0;
                    int index_blue = -1;
                    for (int i = 0; i < blueContours.size(); i++) {
                        cv::Rect bounding_rect_blue = cv::boundingRect(blueContours[i]);
                        int area = bounding_rect_blue.width * bounding_rect_blue.height;
                        if (area > max_blue_contour_area) {
                            max_blue_contour_area = area;
                            index_blue = i;
                        }
                    }

                    if (index_yellow != -1) {
                        processContour(yellowContours[index_yellow], blurredCroppedImg, cv::Scalar(0, 255, 255), detection_threshold);
                    }
                    if (index_blue != -1) {
                        processContour(blueContours[index_blue], blurredCroppedImg, cv::Scalar(255, 0, 0), detection_threshold);
                    }
                } else if (approach == 2) {
                    sortAndProcessContours(yellowContours, blurredCroppedImg, cv::Scalar(0, 255, 255), detection_threshold);
                    sortAndProcessContours(blueContours, blurredCroppedImg, cv::Scalar(255, 0, 0), detection_threshold);
                }

                std::lock_guard<std::mutex> lck(gsrMutex);
                std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;

                if (writeChoice == 'y') {
                    bool isWriteSuccess;

                    if (colorChoice == 'b') {
                        std::string filename = utc_time + "blue" + timeStamp + ".jpg";
                        isWriteSuccess = writeImageToFolder(blueMask, folderPath, filename);
                    } else if (colorChoice == 'y') {
                        std::string filename = utc_time + "yellow" + timeStamp + ".jpg";
                        isWriteSuccess = writeImageToFolder(yellowMask, folderPath, filename);
                    }
                    if (!isWriteSuccess) {
                        return 1;
                    }
                }

                if (VERBOSE) {
                    cv::imshow("CroppedImg", croppedImg);
                    cv::imshow("Cropped Blurred Image", blurredCroppedImg);
                    if (writeChoice == 'y') {
                        (colorChoice == 'b') ? cv::imshow("Blue Mask", blueMask) : cv::imshow("Yellow Mask", yellowMask);
                    } else {
                        cv::imshow("Blue Mask", blueMask);
                        cv::imshow("Yellow Mask", yellowMask);
                    }

                    if (pauseOnFrameMode == 'y') {
                        cv::waitKey(0);
                    } else {
                        cv::waitKey(1);
                    }
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

void processContour(const std::vector<cv::Point>& contour, cv::Mat& image, const cv::Scalar& color, int detection_threshold) {
    cv::Rect bounding_rect = cv::boundingRect(contour);
    int area = bounding_rect.width * bounding_rect.height;
    if (area > detection_threshold) {
        cv::rectangle(image, bounding_rect, color, 1);
        cv::Moments mu = cv::moments(contour);
        if (mu.m00 != 0) {
            cv::Point2f mc = cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
            cv::circle(image, mc, 2, color, -1);
            std::ostringstream coords;
            coords << "x: " << mc.x << ", y: " << mc.y;
            cv::putText(image, coords.str(), cv::Point2f(mc.x + 5, 50), cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
        }
    }
}

void sortAndProcessContours(std::vector<std::vector<cv::Point>>& contours, cv::Mat& image, const cv::Scalar& color, int detection_threshold) {
    if (!contours.empty()) {
        std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
            return cv::contourArea(a) > cv::contourArea(b);
        });
        processContour(contours[0], image, color, detection_threshold);
    }
}

void createSliderWindowBlueMask() {
    cv::namedWindow("Blue Masking Inspector", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Hue (min)", "Blue Masking Inspector", &blueMinH, 179);
    cv::createTrackbar("Hue (max)", "Blue Masking Inspector", &blueMaxH, 179);
    cv::createTrackbar("Sat (min)", "Blue Masking Inspector", &blueMinS, 255);
    cv::createTrackbar("Sat (max)", "Blue Masking Inspector", &blueMaxS, 255);
    cv::createTrackbar("Val (min)", "Blue Masking Inspector", &blueMinV, 255);
    cv::createTrackbar("Val (max)", "Blue Masking Inspector", &blueMaxV, 255);
}

void createSliderWindowYellowMask() {
    cv::namedWindow("Yellow Masking Inspector", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Hue (min)", "Yellow Masking Inspector", &yellowMinH, 179);
    cv::createTrackbar("Hue (max)", "Yellow Masking Inspector", &yellowMaxH, 179);
    cv::createTrackbar("Sat (min)", "Yellow Masking Inspector", &yellowMinS, 255);
    cv::createTrackbar("Sat (max)", "Yellow Masking Inspector", &yellowMaxS, 255);
    cv::createTrackbar("Val (min)", "Yellow Masking Inspector", &yellowMinV, 255);
    cv::createTrackbar("Val (max)", "Yellow Masking Inspector", &yellowMaxV, 255);
}

int getGaussianKernelSizeDimension() {
    return (2 * gaussianKernelSizeSlider) + 1;
}

void createMask(cv::Mat& mask, cv::Scalar minHSV, cv::Scalar maxHSV) {
    cv::inRange(hsvImage, minHSV, maxHSV, mask);
}

void putFrameInfo(cv::Mat& targetImg, std::string color, int minH, int minS, int minV, int maxH, int maxS, int maxV, std::string timeStamp) {
    std::ostringstream oss;
    oss << color << " Timestamp: " << timeStamp << ", Blur: " << gaussianKernelSize << "x" << gaussianKernelSize << ", X-blur: " << gaussianStandardDeviationX << ", Y-blur: " << gaussianStandardDeviationY
        << ", HL: " << minH << ", SL: " << minS << ", VL: " << minV
        << ", HH: " << maxH << ", SH: " << maxS << ", VH: " << maxV;
    cv::putText(targetImg, oss.str(), cv::Point(3, 20), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 255, 255), 1);
}

bool writeImageToFolder(cv::Mat& image, std::string& folderPath, std::string& filename) {
    std::string fullPath = folderPath + "/" + filename;
    if (!fs::exists(folderPath)) {
        if (!fs::create_directories(folderPath)) {
            std::cerr << "Failed to create folder: " << folderPath << std::endl;
            return false;
        }
    }
    if (!cv::imwrite(fullPath, image)) {
        std::cerr << "Failed to write image to file: " << fullPath << std::endl;
        return false;
    }
    return true;
}
