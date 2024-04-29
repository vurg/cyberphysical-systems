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
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <iomanip>

using namespace cv;
using namespace std;

// Define color range for detecting yellow cones in HSV color space
cv::Scalar yellowMin = cv::Scalar(20, 60, 70);
cv::Scalar yellowMax = cv::Scalar(40, 200, 200);

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
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;

                timeStamp = std::to_string(env.sampleTimeStamp().seconds()) + std::to_string(env.sampleTimeStamp().microseconds());
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            int gaussianKernelSize = 0, gaussianStandardDeviation = 0;

            int gaussianKernelSizeOptions[] = {1, 3, 5, 11, 13};

            //  Blurring controls
            cv::namedWindow("Blurring Inspector", CV_WINDOW_AUTOSIZE);
            cvCreateTrackbar("Kernel Size Mode", "Blurring Inspector", &gaussianKernelSize, 4);
            cvCreateTrackbar("Standard Deviation", "Blurring Inspector", &gaussianStandardDeviation, 9999);

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

                /* If needed again in the future ...

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

                cv::putText(img,                     // Image to draw on
                imageMessage,            // Text to draw
                cv::Point(5, 50),       // Position of the text (x, y)
                cv::FONT_HERSHEY_TRIPLEX, // Font type
                0.5,                     // Font scale
                cv::Scalar(255, 255, 255),   // Font color (BGR)
                1,                       // Font thickness
                cv::LINE_AA);            // Anti-aliasing

                // Example: Draw a red rectangle and display image.
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));
                */

                //  Cropping
                croppedImg = img(cv::Rect(0, 255, 640, 155));

                //  Blurring
                GaussianBlur(croppedImg, blurredCroppedImg, Size(gaussianKernelSizeOptions[gaussianKernelSize], gaussianKernelSizeOptions[gaussianKernelSize]), gaussianStandardDeviation, gaussianKernelSize);

                // Create matrix for storing blurred image copy
                cv::Mat hsvImage;
                // Copy blurred image into new matrix
                blurredCroppedImg.copyTo(hsvImage);
                // Convert the copied image into hsv color space
                cv::cvtColor(hsvImage, hsvImage, cv::COLOR_BGR2HSV);

                // Create a mask isolating yellow hues within the specified range.
                cv::Mat mask;
                cv::inRange(hsvImg, yellowMin, yellowMax, mask);
                // Find contours to store outlines of the yellow cones
                std::vector<std::vector<cv::Point>> yellowContours;

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    imshow("cropped image", croppedImg);
                    imshow("blurred image", blurredCroppedImg);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

