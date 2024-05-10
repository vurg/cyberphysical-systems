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

// Library for writing plotting data to a data file (CSV)
#include <fstream>

// (Likely remove for production) Random library for placeholder angle calculation
#include <random>

#include <string>
#include <cmath>  // For std::abs
#include <sstream>  // For std::ostringstream
#include <iomanip>  // For std::setprecision

// Define HSV color ranges for detecting yellow, blue, and red cones:
// Each pair of Scalars defines the min and max H, S, and V values.
cv::Scalar yellowMin = cv::Scalar(20, 60, 70);
cv::Scalar yellowMax = cv::Scalar(40, 200, 200);

cv::Scalar blueMin = cv::Scalar(100, 50, 30);
cv::Scalar blueMax = cv::Scalar(120, 255, 253);

cv::Scalar redMin = cv::Scalar(177, 100, 100);
cv::Scalar redMax = cv::Scalar(179, 190, 253);

// Function declarations
float generateRandomSteeringAngle(); // (Likely remove for production) placeholder for actual calculations
void processContour(const std::vector<cv::Point>& contour, cv::Mat& image, const cv::Scalar& color, int detection_threshold); // Function to process countour
std::string calculatePercentageDifference(const std::string& calculatedStr, const std::string& actualStr);
std::string padMicroseconds(const std::string& timeStamp);
void writeDataEntry(const std::string &filename, const std::string &timeStamp, const std::string &calculatedSteeringAngle, const std::string &actualGroundSteering);

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
                //std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;

                timeStamp = std::to_string(env.sampleTimeStamp().seconds()) + padMicroseconds(std::to_string(env.sampleTimeStamp().microseconds()));
            };

            // Plotting data file setup (to make sure we dont use old data)
            std::string filename = "/tmp/plotting_data.csv";
            std::ofstream outFile;
            outFile.open(filename, std::ios::out | std::ios::trunc); // Opens and resets existing data file

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                // OpenCV data structure to hold an image.
                cv::Mat img, croppedImg, blurredCroppedImg;

                // Placeholder for calculated steering angle
                float calculatedSteeringAngleFloat = generateRandomSteeringAngle();
                std::string calculatedSteeringAngle = std::to_string(calculatedSteeringAngleFloat);

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
                cv::inRange(hsvImage, yellowMin, yellowMax, yellowMask);
                std::vector<std::vector<cv::Point>> yellowContours;
                cv::findContours(yellowMask, yellowContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

                cv::Mat blueMask;
                cv::inRange(hsvImage, blueMin, blueMax, blueMask);
                std::vector<std::vector<cv::Point>> blueContours;
                cv::findContours(blueMask, blueContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

                // Combine images with bitwise_or
                //cv::bitwise_or(yellowMask, blueMask, result);
                
                std::vector<cv::Moments> muYellow(yellowContours.size());
                std::vector<cv::Moments> muBlue(blueContours.size());

                std::vector<cv::Point2f> mcYellow(yellowContours.size());
                std::vector<cv::Point2f> mcBlue(blueContours.size());

                // Print timestamp
                std::string messageTimeStamp = + "ts: " + timeStamp + ";";
                cv::putText(blurredCroppedImg, messageTimeStamp, cv::Point2f(5,10), cv::FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(255, 255, 255), 1);
                
                int detection_threshold = 10;
                
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

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::string actualGroundSteering = std::to_string(gsr.groundSteering());
                    //std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                    std::cout << "group_21;" << timeStamp << ";" << calculatedSteeringAngle << ";" << actualGroundSteering << ";Percentage Difference: " << calculatePercentageDifference(calculatedSteeringAngle,actualGroundSteering) <<  std::endl;
                    writeDataEntry(filename, timeStamp, calculatedSteeringAngle, actualGroundSteering);
                }

                // Display image on your screen.
                if (VERBOSE) {
                    //cv::imshow(sharedMemory->name().c_str(), img);
                    //cv::imshow("yellow mask", yellowMask);
                    //cv::imshow("blue mask", blueMask);
                    cv::imshow("cropped blurred image", blurredCroppedImg);
                    cv::waitKey(1);
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

float generateRandomSteeringAngle() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> distr(-0.22, 0.22); // Random number between -0.22 and +0.22
    return static_cast<float>(distr(gen));
}

// Function below generated by LLM*
std::string calculatePercentageDifference(const std::string& calculatedStr, const std::string& actualStr) {
    double calculated = std::stod(calculatedStr);
    double actual = std::stod(actualStr);

    if (actual == 0) {
        return (calculated == 0) ? "0.0%" : "Undefined";  // Handle division by zero
    }

    double difference = ((calculated - actual) / actual) * 100.0;

    // Format the result to a string with a percentage sign
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << difference << '%';
    return oss.str();
}

std::string padMicroseconds(const std::string& timeStamp) { // Note: this function is specifically for fixing truncation in MICROSECONDS ONLY
    int requiredLength = 6;
    int currentLength = timeStamp.length();
    if (currentLength < requiredLength) { // If the timestamp is shorter than it should be
        int zerosToAdd = requiredLength - currentLength; // Figure out how many zeros need to be added
        std::string padding(zerosToAdd, '0');
        return padding + timeStamp; // Add padding BEFORE the timestamp
    }
    return timeStamp; // If the timestamp is already 6 characters, just return it as normal
}

// Function that is called every frame to write the plotting data
// Note: This function requires that the necessary file creation and cleanup is done at the beginning of main
void writeDataEntry(const std::string &filename, const std::string &timeStamp, const std::string &calculatedSteeringAngle, const std::string &actualGroundSteering) {
    std::ofstream file;
    file.open(filename, std::ios_base::app); // opens data file

    if(file.is_open()) {
        // Writes data to file
        file << timeStamp << "," << calculatedSteeringAngle << "," << actualGroundSteering << "\n";
        file.close(); // Closes file
    } else {
        std::cout << "Failed to open data file: " << filename << std::endl;
    }
}