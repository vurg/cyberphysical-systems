/* Title: Steering Actuator Microservice for Autonomous Car
 * Authors: Nasit Vurgun, Sam Hardingham, Kai Rowley, Daniel van den Heuvel
 * Institution: University of Gothenburg, Sweden
 * Course: DIT638/DIT639 (2024), taught by Prof. Christian Berger
 *
 * Template Code provided by:
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

// Other utility libraries
#include <chrono>   // For timing
#include <iomanip>  // For io
#include <fstream>  // Library for writing plotting data to a data file (CSV)
#include <string>   // For strings
#include <cmath>    // For std::abs, math
#include <iostream> // For std::ostringstream

// GLOBAL VARIABLES:
// OpenCV data structure to hold an image.
cv::Mat croppedImg, blurredCroppedImg, hsvImage;

// Cropping rectangle definition of the shared memory img
cv::Rect roi = cv::Rect(0, 255, 640, 144);

// Define HSV color ranges for detecting yellow, blue, and red cones:
// Each pair of Scalars defines the min and max H, S, and V values.
cv::Scalar yellowMin = cv::Scalar(20, 60, 70);
cv::Scalar yellowMax = cv::Scalar(40, 200, 200);
cv::Scalar blueMin = cv::Scalar(100, 50, 30);
cv::Scalar blueMax = cv::Scalar(120, 255, 253);

// Sensor variables
double distanceUS = 0.0;       // Ultrasound sensor reading
double angularVelocityZ = 0.0; // Angular velocity Z sensor reading

// Object Detection Variables
int yellowCone = 0; // yellow cone found
int blueCone = 0;   // blue cone found

// Steering Wheel Angle Related Variables
double steeringWheelAngle = 0.0; // calculated steering wheel angle
double actual_steering = 0.0;    // ground truth
double error = 0.0;              // absolute error

// Clockwise vs Counterclockwise counter
int CW = 0; // positive if clockwise

// Position of Cones - used in Steering Calculation
cv::Point midYellow(0, 0); // center of rect containing yellow cone
cv::Point midBlue(0, 0);   // center of rect containing blue cone

// Comparing Calculated Steering Wheel Angle with Ground Truth
int totalFrames = 0;         // number of non-zero steering frames
int totalCorrect = 0;        // number of correct steering frames
double percentCorrect = 0.0; // % of frames within 25% of actual steering value

// Function declarations
double steering_function(double X); // Steering Function
cv::Point processContour(const std::vector<cv::Point> &contour, cv::Mat &image, const cv::Scalar &color, int detection_threshold);

// Utilities (mainly for testing)
std::string filename = "/tmp/plotting_data.csv";
void writeDataEntry(std::ofstream &file, const std::string &ts, const std::string &calculatedValue, const std::string &actualValue);

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};

    // Write data to CSV file for data analysis (disabled by default)
    std::ofstream file;
    file.open(filename, std::ios_base::app); // Opens data file in append mode
    // Handle errors with file opening
    if (!file.is_open())
    {
        std::cout << "Failed to open data file: " << filename << std::endl;
        return 1; // Return error code
    }

    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            std::string timeStampGSR;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex, &timeStampGSR](cluon::data::Envelope &&env)
            {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);

                // locks twice, to get image, to get data
                // CHANGE HERE
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                // std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Ultrasound Sensor Readings
            opendlv::proxy::DistanceReading ultrasound;
            std::mutex ultrasoundMutex;
            std::string timeStampUS;
            auto onDistanceReading = [&ultrasound, &ultrasoundMutex, &timeStampUS](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(ultrasoundMutex);
                ultrasound = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
                if (env.senderStamp() == 0)
                {
                    distanceUS = ultrasound.distance();
                }
            };

            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

            // Angular Velocity Sensor Readings
            opendlv::proxy::AngularVelocityReading angularVelocity;
            std::mutex angularMutex;
            std::string timeStampAngularVelocity;
            auto onAngularVelocityReading = [&angularVelocity, &angularMutex, &timeStampAngularVelocity](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(angularMutex);
                angularVelocity = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(env));
                angularVelocityZ = angularVelocity.angularVelocityZ();
            };

            od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(), onAngularVelocityReading);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());

                    // Crop image here
                    croppedImg = wrapped(roi).clone();
                }
                // Get the time for each image
                std::pair<bool, cluon::data::TimeStamp> tStamp = sharedMemory->getTimeStamp();
                // Convert the time to microseconds
                std::string timeStamp = std::to_string(cluon::time::toMicroseconds(tStamp.second));

                sharedMemory->unlock();

                //  Blurring
                cv::GaussianBlur(croppedImg, blurredCroppedImg, cv::Size(101, 101), 2.5);

                // Convert the copied image into hsv color space
                cv::cvtColor(blurredCroppedImg, hsvImage, cv::COLOR_BGR2HSV);

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

                // Initialize vectors to save contours from image detection
                std::vector<cv::Moments> muYellow(yellowContours.size());
                std::vector<cv::Moments> muBlue(blueContours.size());

                // Print timestamp
                std::string messageTimeStamp = +"ts: " + timeStamp + ";";
                cv::putText(blurredCroppedImg, messageTimeStamp, cv::Point(5, 10), cv::FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(255, 255, 255), 1);

                /****************** OBJECT DETECTION **********************************************/
                int detection_threshold = 10;

                // Detect yellow cones
                int max_yellow_contour_area = 0;
                int index_yellow = -1;
                for (size_t i = 0; i < yellowContours.size(); i++)
                {
                    cv::Rect bounding_rect_yellow = cv::boundingRect(yellowContours[i]);
                    int area = bounding_rect_yellow.width * bounding_rect_yellow.height;
                    // Find maximum area contour
                    if (area > max_yellow_contour_area)
                    {
                        max_yellow_contour_area = area;
                        index_yellow = i;
                    }
                }

                // Detect blue cones
                int max_blue_contour_area = 0;
                int index_blue = -1;
                for (size_t i = 0; i < blueContours.size(); i++)
                {
                    cv::Rect bounding_rect_blue = cv::boundingRect(blueContours[i]);
                    int area = bounding_rect_blue.width * bounding_rect_blue.height;
                    // Find maximum area contour
                    if (area > max_blue_contour_area)
                    {
                        max_blue_contour_area = area;
                        index_blue = i;
                    }
                }

                // Assign midpoint to contour rect
                if (index_yellow != -1)
                {
                    // Set yellowCone detection flag to 1
                    yellowCone = 1;
                    // Save midpoint of yellow cone contour
                    midYellow = processContour(yellowContours[index_yellow], blurredCroppedImg, cv::Scalar(0, 255, 255), detection_threshold);
                }

                // Assign midpoint to contour rect
                if (index_blue != -1)
                {
                    // Set blueCone detection flag to 1
                    blueCone = 1;
                    // Save midpoint of blue cone contour
                    midBlue = processContour(blueContours[index_blue], blurredCroppedImg, cv::Scalar(255, 0, 0), detection_threshold);
                }

                /****************** STEERING CALCULATION **********************************************/
                // Calculate steering angle (not optimized)
                if (blueCone && yellowCone)
                {
                    // check CW or CCW
                    if (midBlue.x / midBlue.y < midYellow.x / midYellow.y)
                    {
                        CW++;
                    }
                }

                // Appling steering function to angular velocity Z sensor reading
                steeringWheelAngle = steering_function(angularVelocityZ);

                // Apply offsets - based on trends observed from image analysis
                if (CW < 0)
                {
                    // Case: CCW
                    if (midBlue.x < 500)
                    {
                        steeringWheelAngle = steeringWheelAngle + 0.05;
                    }
                    if (midYellow.x > 125)
                    {
                        steeringWheelAngle = steeringWheelAngle - 0.05;
                    }
                    else
                    {
                        steeringWheelAngle = steeringWheelAngle + 0.05;
                    }
                }
                else
                {
                    // Case: CW
                    if (midBlue.x > 200)
                    {
                        steeringWheelAngle = steeringWheelAngle - 0.05;
                    }
                    if (midYellow.x < 500)
                    {
                        steeringWheelAngle = steeringWheelAngle + 0.05;
                    }
                }

                // Use multiplier at close distances
                if (distanceUS < 0.2)
                {
                    steeringWheelAngle = 1.2 * steeringWheelAngle;
                }

                // Apply thresholds to steer hard left and hard right
                if (steeringWheelAngle > 0.155)
                {
                    steeringWheelAngle = 0.22;
                }
                else if (steeringWheelAngle < -0.15)
                {
                    steeringWheelAngle = -0.22;
                }

                /************** COMPARE TO ACTUAL VALUE OF STEERING ANGLE *******************************/
                // Check the value of steering angle
                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    actual_steering = gsr.groundSteering();
                    std::cout << "group_21;" << timeStamp << ";" << steeringWheelAngle << std::endl;
                }

                // Check if there was 0 steering
                if (abs(actual_steering) < 0.0001)
                {
                    // There is 0 steering, so we do not count these frames!!
                }
                else
                {
                    // Increment counter for steering frames (denominator of percentage calculation)
                    totalFrames++;

                    // Calculate error
                    error = abs(steeringWheelAngle - actual_steering);

                    // We wish to be within 25 percent of the actual steering angle (ground truth)
                    if (abs(error) <= abs(0.25 * actual_steering))
                    {
                        // Error is within 25 percent -- great!!
                        totalCorrect++; // increment total correct
                    }

                    // Display on image which direction algorithm steers
                    if (steeringWheelAngle > 0)
                    {
                        cv::putText(blurredCroppedImg, "LEFT", cv::Point(5, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    }
                    else if (steeringWheelAngle < 0)
                    {
                        cv::putText(blurredCroppedImg, "RIGHT", cv::Point(5, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    }

                    // Print percent correct
                    percentCorrect = (static_cast<double>(totalCorrect) / totalFrames * 100.0);
                    // std::cout << "Steering Frames: " << totalFrames << " Correct: "<< percentCorrect << "%" << std::endl;
                    // std::cout << steeringWheelAngle << "," << actual_steering << std::endl;
                }

                // Write to file for data analysis (disabled by default)
                writeDataEntry(file, timeStamp, std::to_string(steeringWheelAngle), std::to_string(actual_steering));

                // Reset global variables before next frame
                blueCone = 0;   // reset flag for blueCone found
                yellowCone = 0; // reset flag for yellowCone found
                error = 0.0;    // reset error

                // Display image on your screen.
                if (VERBOSE)
                {
                    // cv::imshow(sharedMemory->name().c_str(), img);
                    cv::imshow("SteeringView - Group_21 Microservice", blurredCroppedImg);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    // Close the file when done
    file.close();

    return retCode;
}

// Define the steering function from curve fitting
double steering_function(double X)
{
    // Coefficients
    double a = 0.14973124;  // approximately half of steering range
    double b = 0.02949003;  // scaling factor for angular velocity Z
    double c = -0.00177955; // this can even be zero!

    // Calculate and return the result
    return a * std::atan(b * X) + c;
}

// Function to process contours -- finds midpoint and draws box around cone
cv::Point processContour(const std::vector<cv::Point> &contour, cv::Mat &image, const cv::Scalar &color, int detection_threshold)
{
    cv::Rect bounding_rect = cv::boundingRect(contour);
    int area = bounding_rect.width * bounding_rect.height;
    if (area > detection_threshold)
    {
        // Create bounding rectangle
        cv::rectangle(image, bounding_rect, color, 1);
        // Find midpoint of rectangle
        cv::Point midpoint(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y + bounding_rect.height / 2);
        // Draw midpoint
        cv::circle(image, midpoint, 2, color, -1);
        // Put coordinates as text on display image
        std::ostringstream coords;
        coords << "x: " << midpoint.x << ", y: " << midpoint.y;
        cv::putText(image, coords.str(), cv::Point(midpoint.x + 5, 50), cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
        // returns center x,y coordinate of contour rect
        return midpoint;
    }
    // Return an invalid point if area is less than detection threshold
    return cv::Point(-1, -1);
}

// Function that is called every frame to write the plotting data
void writeDataEntry(std::ofstream &file, const std::string &ts, const std::string &calculatedValue, const std::string &actualValue)
{
    // Writes data to file
    file << ts << "," << calculatedValue << "," << actualValue << "\n";
}