/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<string>     // std::string, std::to_string
#include<Eigen/Dense>
#include<Eigen/Core>
#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void LoadOdom(const string &strOdomPath, vector<double> &vTimeStamps, vector<g2o::SE2> &vOdo);

void LoadOdom_SE3Quat(const string &strOdomPath, vector<double> &vTimeStamps, vector<g2o::SE3Quat> &vOdo);
double ttrack_tot = 0;

int main(int argc, char **argv)
{
    const int num_seq = argc - 4;
    cout << "Num seq: " << num_seq << endl;
    bool bFileName = true;
    string file_name;
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_odom_openloris path_to_vocabulary path_to_settings path_to_sequence_1 path_to_sequence_2 ... (name_sequence)" << endl;
        return 1;
    }

    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }
    int seq; int tot_images = 0;
    // Retrieve paths to images
    vector < vector<string> >               vstrImageFilenames;
    vector < vector<double> >               vTimestamps;
    vector < vector<g2o::SE2> >             vOdo;
    vector < vector<g2o::SE3Quat> >             vOdo_;
    vector < vector<double> >               vTimestampsOdom;
    vector < vector<double> >               vTimestampsOdom_;
    vector<int>                             nImages;
    vector<int>                             nOdo;
    vstrImageFilenames.resize(num_seq);
    vTimestamps.resize(num_seq);
    vOdo.resize(num_seq);
    vTimestampsOdom.resize(num_seq);
    vOdo_.resize(num_seq);
    vTimestampsOdom_.resize(num_seq);
    nImages.resize(num_seq);
    nOdo.resize(num_seq);
    
    for(seq = 0; seq < num_seq; seq++)
    {
        string pathSeq(argv[seq + 3]);
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(pathSeq, vstrImageFilenames[seq], vTimestamps[seq]);
        std::cout << "LOAED " << vTimestamps[seq].size() << " img(s)" <<  std::endl;
        cout << "Loading odom for sequence " << seq << "...";
        LoadOdom(pathSeq, vTimestampsOdom[seq],vOdo[seq]);
        std::cout << "LOAED" << std::endl;
        cout << "Loading odom for sequence " << seq << "...";
        LoadOdom_SE3Quat(pathSeq, vTimestampsOdom_[seq],vOdo_[seq]);
        std::cout << "LOAED" << std::endl;
        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nOdo[seq] = vTimestampsOdom[seq].size();

        if((nImages[seq]<=0)||(nOdo[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }
    }
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::ODOM_MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence(s) ..." << endl;

    // Main loop
    cv::Mat im;
    int proccIm=0;
    for ( seq = 0; seq < num_seq; seq++)
    {     
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));   
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],0);
            clahe->apply(im,im);
            double tframe = vTimestamps[seq][ni];
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: " << vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            SLAM.TrackOdomMono(im, vOdo[seq][ni], tframe, vOdo_[seq][ni]);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestamps[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();
   
    // Save camera trajectory
    if(bFileName)
    {
        SLAM.SaveBodyKeyFrameTrajectoryTUM("kf_body_" + file_name + ".txt");
        SLAM.SaveBodyTrajectoryTUM("f_body_" + file_name + ".txt"); 
        SLAM.SaveKeyFrameTrajectoryTUM("kf_cam_" + file_name + ".txt");
        cout << "trajectory saved!" << endl;
    }
    else
    {
        SLAM.SaveBodyKeyFrameTrajectoryTUM("KeyFrameTrajectory_cam.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_cam.txt");
        cout << "trajectory saved!" << endl;
    }
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    
    string strPathLeft = strPathToSequence + "/color.txt";
    fTimes.open(strPathLeft.c_str());
    vTimestamps.reserve(5000);
    vstrImageFilenames.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if (s[0]=='#')
        {
            continue;
        }
        
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(strPathToSequence + "/" + sRGB);
        }
    }
    fTimes.close();
}

void LoadOdom(const string &strOdomPath, vector<double> &vTimeStamps, vector<g2o::SE2> &vOdo)
{
    ifstream fOdom;
    fOdom.open(strOdomPath + "/odom_yaw.txt");
    vOdo.reserve(5000);
    double ts;
    Eigen::Vector3d xyr;
    std::cout<<"Reading Odometry Data:...\n";
    while(!fOdom.eof())
    {
        string s;
        getline(fOdom,s);
        //std::cout<<s<<std::endl;
        if(s[0] == '#')
            continue;
        if(!s.empty())
        {
            stringstream ss(s);
            ss >> ts;
            vTimeStamps.push_back(ts);
            ss >> xyr(0) >> xyr(1) >> xyr(2);
            vOdo.push_back(g2o::SE2(xyr));
        }
    }
    std::cout<<"Done..! Reading "<< vOdo.size()<<std::endl;
}
void LoadOdom_SE3Quat(const string &strOdomPath, vector<double> &vTimeStamps, vector<g2o::SE3Quat> &vOdo)
{
    ifstream fOdom;
    fOdom.open(strOdomPath + "/odom_interp_se3quat.txt");
    vOdo.reserve(5000);
    double ts;
    g2o::Vector7d v;
    g2o::SE3Quat se3quat;
    std::cout<<"Reading Odometry Data:...\n";
    while(!fOdom.eof())
    {
        string s;
        getline(fOdom,s);
        if(s[0] == '#')
            continue;
        if(!s.empty())
        {
            stringstream ss(s);
            ss >> ts;
            vTimeStamps.push_back(ts);
            ss >> v(0) >> v(1) >> v(2) >> v(3) >> v(4) >> v(5) >> v(6);
            se3quat.fromVector(v);
            vOdo.push_back(se3quat);
        }
    }
    std::cout<<"Done..! Read "<< vOdo.size()<<std::endl;

}