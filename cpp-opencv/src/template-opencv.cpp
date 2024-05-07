/*
 * Copyright (C) 2020  Christian Berger
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

// Include the single-file, header-only middleware libcluon to create high-performance microservices


#include "cluon-complete.hpp"  // For CLUON messaging framework

// Include the OpenDLV Standard Message Set that contains messages for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the standard library headers
#include <chrono>   // For time-related operations
#include <iomanip>  // For formatting output

// Include the image processing and GUI headers from OpenCV
#include <opencv2/core.hpp>         // Core functionality
#include <opencv2/imgproc/imgproc.hpp> // Image processing
#include <opencv2/highgui/highgui.hpp> // GUI

// Include filesystem header for filesystem operations
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

// Define HSV color ranges for detecting yellow, blue, and red cones:
// Each pair of Scalars defines the min and max H, S, and V values.
cv::Scalar yellowMin = cv::Scalar(20, 60, 70);
cv::Scalar yellowMax = cv::Scalar(40, 200, 200);

cv::Scalar blueMin = cv::Scalar(100, 50, 30);
cv::Scalar blueMax = cv::Scalar(120, 255, 253);

cv::Scalar redMin = cv::Scalar(177, 100, 100);
cv::Scalar redMax = cv::Scalar(179, 190, 253);

bool writeImageToFolder(const cv::Mat& image, const std::string& folderPath, const std::string& filename) {
    // Create the full path
    std::string fullPath = folderPath + "/" + filename;

    // Check if the folder exists
    if (!fs::exists(folderPath)) {
        // Create the folder if it doesn't exist
        if (!fs::create_directories(folderPath)) {
            std::cerr << "Failed to create folder: " << folderPath << std::endl;
            return false;
        }
    }

    // Write the image
    if (!cv::imwrite(fullPath, image)) {
        std::cerr << "Failed to write image to file: " << fullPath << std::endl;
        return false;
    }

    std::cout << "Image saved to: " << fullPath << std::endl;
    return true;
}

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            std::string timeStamp;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex, &timeStamp](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                
                //locks twice, to get image, to get data
                //CHANGE HERE
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;

                timeStamp = std::to_string(env.sampleTimeStamp().seconds()) + std::to_string(env.sampleTimeStamp().microseconds());
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            char writeChoice;
            char colorChoice;
            std::string folderPath = "output";

            while (!(writeChoice == 'y' || writeChoice == 'n')){
                std::cout << "Write frames to a folder (y | n)?" << std::endl;
                std::cin >> writeChoice;
            }
            if (writeChoice == 'y') {
                while (!(colorChoice == 'b' || colorChoice == 'y')){
                    std::cout << "Color to process: Blue (b) | Yellow (y)?" << std::endl;
                    std::cin >> colorChoice;
                }
            }

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                // OpenCV data structure to hold an image.
                cv::Mat img, croppedImg, blurredCroppedImg;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
     
                sharedMemory->unlock();


                // Get current time as a time_point object
                auto now = std::chrono::system_clock::now();

                // Convert time_point object to time_t
                std::time_t now_t = std::chrono::system_clock::to_time_t(now);

                // Convert time_t to tm as UTC
                std::tm* now_tm = std::gmtime(&now_t);

                // Prepare output stream
                std::ostringstream oss;

                // Write time into the output stream
                oss << std::put_time(now_tm, "%Y-%m-%dT%H:%M:%SZ");

                // Get string from output stream
                std::string utc_time = oss.str();

                std::string imageMessage = "Now: " + utc_time + ";" + " ts: " + timeStamp + ";";


                //  Cropping
                croppedImg = img(cv::Rect(0, 255, 640, 144));

                //  Blurring
                cv::GaussianBlur(croppedImg, blurredCroppedImg, cv::Size(101, 101), 2.5);

                // Create matrix for storing blurred image copy
                cv::Mat hsvImage;
                // Copy blurred image into new matrix
                blurredCroppedImg.copyTo(hsvImage);
                // Convert the copied image into hsv color space
                cv::cvtColor(hsvImage, hsvImage, cv::COLOR_BGR2HSV);

                // Create masks isolating yellow, blue, and red hues within their respective ranges,
                // and find contours to store outlines of cones of each color.

                cv::Mat yellowMask;
                std::vector<std::vector<cv::Point>> yellowContours;
                std::vector<cv::Vec4i> yellowHierchy;
                cv::inRange(hsvImage, yellowMin, yellowMax, yellowMask);
                cv::findContours(yellowMask, yellowContours, yellowHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                cv::drawContours(croppedImg, yellowContours, -1, cv::Scalar(0, 255, 255), 2);

                cv::Mat blueMask;
                std::vector<std::vector<cv::Point>> blueContours;
                std::vector<cv::Vec4i> blueHierchy;
                cv::inRange(hsvImage, blueMin, blueMax, blueMask);
                cv::findContours(blueMask, blueContours, blueHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                cv::drawContours(croppedImg, blueContours, -1, cv::Scalar(255, 0, 0), 2);

                //  Post Mask Modfications
                //  For blue mask
                std::string blueMaskFrameText = "Blue; " + timeStamp + "; " + utc_time;
                cv::putText(blueMask, blueMaskFrameText, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                //  For yellow mask
                std::string yellowMaskFrameText = "Yellow; " + timeStamp + "; " + utc_time;
                cv::putText(yellowMask, yellowMaskFrameText, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);


                for (int i = 0; i < blueHierchy.size(); i++) {
                        std::cout << "blue hierchy list number: " << i << std::endl;
                    for (int j = 0; j < 4; j++) {
                        std::cout << blueHierchy[i][j] << std::endl;
                    }
                }


                for (int i = 0; i < blueContours.size(); i++) {
                        std::cout << "blue countor number: " << i << std::endl;

                        cv::Rect boundingArea = cv::boundingRect(blueContours[i]);
                        std::cout << "area: " << boundingArea.width*boundingArea.height << std::endl;
                }

                std::cout << "" << std::endl; 

                // cv::Mat redMask;
                // std::vector<std::vector<cv::Point>> redContours;
                // std::vector<cv::Vec4i> redHierchy;
                // cv::inRange(hsvImage, redMin, redMax, redMask);
                // cv::findContours(redMask, redContours, redHierchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                // cv::drawContours(croppedImg, redContours, -1, cv::Scalar(0, 0, 255), 2);


                // Combine images with bitwise_or
                //cv::bitwise_or(yellowMask, blueMask, result);
                //cv::bitwise_or(redMask, result, result);
                
                // std::vector<cv::Moments> muYellow(yellowContours.size());
                // std::vector<cv::Moments> muBlue(blueContours.size());
                // std::vector<cv::Moments> muRed(redContours.size());
                
                // std::vector<cv::Point2f> mcYellow(yellowContours.size());
                // std::vector<cv::Point2f> mcBlue(blueContours.size());
                // std::vector<cv::Point2f> mcRed(redContours.size());

                // Print timestamp
                // std::string messageTimeStamp = + "ts: " + timeStamp + ";";
                // cv::putText(blurredCroppedImg, messageTimeStamp, cv::Point2f(5,10), cv::FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(255, 255, 255), 1);
                
                /*int detection_threshold = 10;
                
                if(yellowContours.size()>0){
                    // Define rectangle bounding box around the objects - take first in hierarchy
                    cv::Rect bounding_rect_yellow = cv::boundingRect(yellowContours[0]);
                    // Calculate size of detected object
                    int rect_yellow_area = bounding_rect_yellow.width*bounding_rect_yellow.height;
                    
                    // Check if detected object exceeds detection threshold
                    if (rect_yellow_area > detection_threshold){
                            // Draw bounding box
                            cv::rectangle(blurredCroppedImg, bounding_rect_yellow, cv::Scalar(0, 255, 255), 1);
                            // Use moments to calculate center of detected object
                            muYellow[0] = cv::moments(yellowContours[0]);
                            if(muYellow[0].m00 !=0){
                                mcYellow[0] = cv::Point2f(static_cast<float>(muYellow[0].m10 / muYellow[0].m00), static_cast<float>(muYellow[0].m01 / muYellow[0].m00));
                                cv::circle(blurredCroppedImg, mcYellow[0], 2, cv::Scalar(0, 255, 255), -1);
                                
                                // Overlay centroid coordinates on bounding box
                                std::string coords = "x: " + std::to_string(mcYellow[0].x) + ", y: " + std::to_string(mcYellow[0].y);
                                cv::putText(blurredCroppedImg, coords, cv::Point2f(mcYellow[0].x+5,50), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 255), 1);
                            }
                    }
                    
                }
                
                if(blueContours.size()>0){
                    // Define rectangle bounding box around the objects - take first in hierarchy
                    cv::Rect bounding_rect_blue = cv::boundingRect(blueContours[0]);
                    // Calculate size of detected object
                    int rect_blue_area = bounding_rect_blue.width*bounding_rect_blue.height;

                    // Check if detected object exceeds detection threshold
                    if (rect_blue_area > detection_threshold){
                        // Draw bounding box
                        cv::rectangle(blurredCroppedImg, bounding_rect_blue, cv::Scalar(255, 0, 0), 1);                            
                        // Use moments to calculate center of detected object
                        muBlue[0] = cv::moments(blueContours[0]);
                        if(muBlue[0].m00 !=0){
                            mcBlue[0] = cv::Point2f(static_cast<float>(muBlue[0].m10 / muBlue[0].m00), static_cast<float>(muBlue[0].m01 / muBlue[0].m00));
                            cv::circle(blurredCroppedImg, mcBlue[0], 2, cv::Scalar(255, 0, 0), -1);
                        
                            // Overlay centroid coordinates on bounding box
                            std::string coords = "x: " + std::to_string(mcBlue[0].x) + ", y: " + std::to_string(mcBlue[0].y);
                            cv::putText(blurredCroppedImg, coords,cv::Point2f(mcBlue[0].x+5,50), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
                        }
                    }
                }
                
                if(redContours.size()>0){
                    // Define rectangle bounding box around the objects - take first in hierarchy
                    cv::Rect bounding_rect_red = cv::boundingRect(redContours[0]);
                    // Calculate size of detected object
                    int rect_red_area = bounding_rect_red.width*bounding_rect_red.height;
                    
                    // Check if detected object exceeds detection threshold
                    if (rect_red_area > detection_threshold){
                        // Draw bounding box
                        cv::rectangle(blurredCroppedImg, bounding_rect_red, cv::Scalar(0, 0, 255), 1);
                        // Use moments to calculate center of detected object
                        muRed[0] = cv::moments(redContours[0]);
                        if(muRed[0].m00 !=0){
                            mcRed[0] = cv::Point2f(static_cast<float>(muRed[0].m10 / muRed[0].m00), static_cast<float>(muRed[0].m01 / muRed[0].m00));
                            cv::circle(blurredCroppedImg, mcRed[0], 2, cv::Scalar(0, 0, 255), -1);
                        
                            // Overlay centroid coordinates on bounding box
                            std::string coords = "x: " + std::to_string(mcRed[0].x) + ", y: " + std::to_string(mcRed[0].y);
                            cv::putText(blurredCroppedImg, coords,cv::Point2f(mcRed[0].x+5,50), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
                        }
                    }
                }*/


                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                //  Frame folder output
                if (colorChoice == 'b') {
                    std::string filename = utc_time + "blue" + timeStamp + ".jpg";
                    writeImageToFolder(blueMask, folderPath, filename);
                } else {
                    std::string filename = utc_time + "yellow" + timeStamp + ".jpg";
                    writeImageToFolder(yellowMask, folderPath, filename);
                }

                // Display image on your screen.
                if (VERBOSE) {
                    //cv::imshow(sharedMemory->name().c_str(), img);
                    //cv::imshow("yellow mask", yellowMask);
                    //cv::imshow("blue mask", blueMask);
                    cv::imshow("croppedImg", croppedImg);

                    if (writeChoice == 'y') {
                        (colorChoice == 'b') ? cv::imshow("blueMask", blueMask) : cv::imshow("yellowMask", yellowMask);
                    } else {
                        cv::imshow("blueMask", blueMask);;
                        cv::imshow("yellowMask", yellowMask);
                    }

                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}