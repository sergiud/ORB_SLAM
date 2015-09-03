/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* Copyright (C) 2015 Sergiu Dotenco <sergiu.dotenco@fau.de> (University of Erlangen-Nürnberg)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Converter.h"
#include "FramePublisher.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPublisher.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

namespace {

const char* const banner =
    "ORB SLAM\n"
    "Copyright (C) 2015 Sergiu Dotenco\n"
    "License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>\n"
    "This is free software: you are free to change and redistribute it.\n"
    "There is NO WARRANTY, to the extent permitted by law.\n"
    ;

void usage(const boost::program_options::options_description& desc)
{
    std::cout
        << banner <<
        "\n"
        "usage: gvs [options] input gyro calibration"
        "\n"
        << desc <<
        "\n"
        ;
}

void help(const boost::program_options::options_description& desc)
{
    usage(desc);

    std::cout <<
        "\n"
        "Report bugs to: sergiu.dotenco@th-nuernberg.de\n"
        //"pkg home page: <http://www.gnu.org/software/pkg/>"
        "General help using GNU software: <http://www.gnu.org/gethelp/>\n"
        ;
}

void version()
{
    std::cout << banner;
}

} // namespace

int main(int argc, char **argv)
{
    namespace po = boost::program_options;

    po::positional_options_description pdesc;
    pdesc.add("input", 1);

    boost::filesystem::path inVideoFileName;
    boost::filesystem::path calibFileName;
    boost::filesystem::path vocabFileName;
    boost::filesystem::path trajFileName;
    int features = 1000;
    float scale = 1.2f;
    int levels = 8;
    int fastTh = 20;
    int score = 1;

    po::options_description opts("available options");
    opts.add_options()
        ("input,i", (po::value<boost::filesystem::path>(&inVideoFileName))->required()->value_name("<file>"), "input video file name")
        ("calibration,c", (po::value<boost::filesystem::path>(&calibFileName))->required()->value_name("<file>"), "calibration file name")
        ("vocabulary,b", (po::value<boost::filesystem::path>(&vocabFileName))->required()->value_name("<file>"), "vocabulary file name")
        ("trajectory", (po::value<boost::filesystem::path>(&trajFileName))->value_name("<file>"), "trajectory file name")
        ("features,f", po::value<int>(&features)->value_name("<number>"), "number of features")
        ("scale,s", po::value<float>(&scale)->value_name("<factor>"), "scale factor")
        ("level,l", po::value<int>(&levels)->value_name("<number>"), "number of pyramid levels")
        ("threshold,t", po::value<int>(&fastTh)->value_name("<value>"), "FAST threshold")
        ("no-motion-model,m", "disables constant velocity motion mode")
        ("help,h", "show help")
        ("version,v", "show version information")
        ;

    po::variables_map vars;

    po::store(po::command_line_parser(argc, argv)
        .options(opts)
        .positional(pdesc)
        .run(), vars);

    if (vars.empty()) {
        usage(opts);
        return EXIT_FAILURE;
    }

    if (vars.count("help")) {
        help(opts);
        return EXIT_SUCCESS;
    }

    if (vars.count("version")) {
        version();
        return EXIT_SUCCESS;
    }

    try {
        po::notify(vars);
    }
    catch (std::exception& e) {
        std::cerr << "error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;

    // Load ORB vocabulary
    ORB_SLAM::ORBVocabulary vocabulary;

    if (!vocabFileName.empty()) {
        std::cout << boost::format("reading vocabulary %1%...") % vocabFileName;
        vocabulary.loadFromTextFile(vocabFileName.string());

        std::cout << ' ' << "done" << std::endl;
    }

    // Create KeyFrame database
    ORB_SLAM::KeyFrameDatabase database(vocabulary);

    // Create the map
    ORB_SLAM::Map world;

    FramePub.SetMap(&world);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&world);

    cv::FileStorage calibFs(calibFileName.string(), cv::FileStorage::READ);

    cv::Mat K;
    cv::Mat distCoeffs;

    calibFs["camera_matrix"] >> K;
    calibFs["distortion_coefficients"] >> distCoeffs;

    cv::VideoCapture capture(inVideoFileName.string());

    if (!capture.isOpened()) {
        std::cerr << "error: failed to open video file" << std::endl;
        return EXIT_FAILURE;
    }

    // Initialize the tracking thread and launch
    ORB_SLAM::Tracking tracker(&vocabulary, &FramePub, &MapPub, &world, cv::Mat1f(K), cv::Mat1f(distCoeffs), capture.get(CV_CAP_PROP_FPS),
            features, scale, levels, fastTh, score, vars.count("no-motion-model") == 0);

    tracker.SetKeyFrameDatabase(&database);

    // Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping localMapper(&world);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&localMapper);

    // Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing loopCloser(&world, &database, &vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &loopCloser);

    // Set pointers between threads
    tracker.SetLocalMapper(&localMapper);
    tracker.SetLoopClosing(&loopCloser);

    localMapper.SetTracker(&tracker);
    localMapper.SetLoopCloser(&loopCloser);

    loopCloser.SetTracker(&tracker);
    loopCloser.SetLocalMapper(&localMapper);

    cv::Mat image;
    bool stop = false;

    while (!stop) {
        capture >> image;

        if (!image.empty()) {
            FramePub.Refresh();
            MapPub.Refresh();

            tracker.CheckResetByPublishers();

            cv::Mat tmp;
            cv::cvtColor(image, tmp, CV_BGR2GRAY);
            tracker.Track(tmp);

            cv::imshow("ORB_SLAM", FramePub.DrawFrame());
            cv::waitKey(1);
        }
        else
            stop = true;
    }

    localMappingThread.interrupt();
    loopClosingThread.interrupt();
    localMappingThread.join();
    loopClosingThread.join();

    if (!trajFileName.empty()) {
        // Save keyframe poses at the end of the execution
        boost::filesystem::ofstream f(trajFileName);

        vector<ORB_SLAM::KeyFrame*> vpKFs = world.GetAllKeyFrames();
        std::sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM::KeyFrame::lId);

        cout << endl << "Saving Keyframe Trajectory to " <<  trajFileName << endl;
        f << fixed;

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM::KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = ORB_SLAM::Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
    }

	return EXIT_SUCCESS;
}
