/**
* This file is a modified version of ORB-SLAM2.<https://github.com/raulmur/ORB_SLAM2>
*
* This file is part of DynaSLAM.
* Copyright (C) 2018 Berta Bescos <bbescos at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/bertabescos/DynaSLAM>.
*
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<csignal>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include<opencv2/core/core.hpp>

#include "Geometry.h"
#ifdef DYNA_SLAM_WITH_MASKRCNN
#include "MaskNet.h"
#endif
#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

namespace
{
volatile sig_atomic_t g_request_shutdown = 0;

void HandleSignal(int /*signum*/)
{
    if (g_request_shutdown)
    {
        _exit(130);
    }
    g_request_shutdown = 1;
}
}

int main(int argc, char **argv)
{
#ifndef DYNA_SLAM_WITH_MASKRCNN
    if(argc == 6 || argc == 7)
    {
        cerr << endl << "This build of DynaSLAM was compiled without Mask R-CNN support." << endl;
        cerr << "Rebuild with -DDYNA_SLAM_WITH_MASKRCNN=ON to enable (path_to_masks/path_to_output)." << endl;
        return 1;
    }
#endif

    if(argc != 5 && argc != 6 && argc != 7)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association (path_to_masks) (path_to_output)" << endl;
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    std::cout << "nImages: " << nImages << std::endl;

    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Initialize Mask R-CNN
#ifdef DYNA_SLAM_WITH_MASKRCNN
    DynaSLAM::SegmentDynObject *MaskNet = nullptr;
    if (argc==6 || argc==7)
    {
        cout << "Loading Mask R-CNN. This could take a while..." << endl;
        MaskNet = new DynaSLAM::SegmentDynObject();
        cout << "Mask R-CNN loaded!" << endl;
    }
#endif

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.reserve(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Dilation settings
    int dilation_size = 15;
    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                           cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                           cv::Point( dilation_size, dilation_size ) );

    if (argc==7)
    {
        std::string dir = string(argv[6]);
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir = string(argv[6]) + "/rgb/";
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir = string(argv[6]) + "/depth/";
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        dir = string(argv[6]) + "/mask/";
        mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    // Main loop
        cv::Mat imRGB, imD;
        cv::Mat imRGBOut, imDOut,maskOut;

    for(int ni=0; ni<nImages; ni++)
    {
        if (g_request_shutdown)
        {
            cout << endl << "Received shutdown signal, stopping early at frame " << ni
                 << "/" << nImages << " ..." << endl;
            break;
        }

        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED);

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Segment out the images
        cv::Mat mask = cv::Mat::ones(480,640,CV_8U);
#ifdef DYNA_SLAM_WITH_MASKRCNN
        if (argc == 6 || argc == 7)
        {
            cv::Mat maskRCNN;
            maskRCNN = MaskNet->GetSegmentation(imRGB,string(argv[5]),vstrImageFilenamesRGB[ni].replace(0,4,""));
            cv::Mat maskRCNNdil = maskRCNN.clone();
            cv::dilate(maskRCNN,maskRCNNdil, kernel);
            mask = mask - maskRCNNdil;
        }
#endif

        // Pass the image to the SLAM system
        if (argc == 7){SLAM.TrackRGBD(imRGB,imD,mask,tframe,imRGBOut,imDOut,maskOut);}
        else {SLAM.TrackRGBD(imRGB,imD,mask,tframe);}

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        if (argc == 7)
        {
            cv::imwrite(string(argv[6]) + "/rgb/" + vstrImageFilenamesRGB[ni],imRGBOut);
            vstrImageFilenamesD[ni].replace(0,6,"");
            cv::imwrite(string(argv[6]) + "/depth/" + vstrImageFilenamesD[ni],imDOut);
            cv::imwrite(string(argv[6]) + "/mask/" + vstrImageFilenamesRGB[ni],maskOut);
        }

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(static_cast<float>(ttrack));

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    if(!vTimesTrack.empty())
    {
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(size_t i=0; i<vTimesTrack.size(); i++)
        {
            totaltime += vTimesTrack[i];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
        cout << "mean tracking time: " << totaltime/static_cast<float>(vTimesTrack.size()) << endl;
    }

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

#ifdef DYNA_SLAM_WITH_MASKRCNN
    delete MaskNet;
    MaskNet = nullptr;
#endif

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
