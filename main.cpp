/*
This code is the implementation of our paper "TextSLAM: Visual SLAM with Semantic Planar Text Features."

Author: Boying Li   < LeeBY2016@outlook.com >

If you use any code of this repo in your work, please cite our papers:
[1] Li Boying, Zou Danping, et al. "TextSLAM: Visual SLAM with Semantic Planar Text Features."
[2] Li Boying, Zou Danping, et al. "TextSLAM: Visual slam with planar text features."
*/

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <cmath>
#include "ceres/ceres.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <system.h>
#include <setting.h>
#include <tool.h>

using namespace std;

int main(int argc, char **argv)
{
    srand(0);
    TextSLAM::tool Tool;

    // 1. input & output. terminal: ./TextSLAM ../yaml/xxx.yaml
    if(argc!=2){
        throw runtime_error("wrong number of argument.");
        return 1;
    }
    string argv1 = (string)argv[1];
    TextSLAM::setting Set(argv1);

    // 2. get Img, TextInfo
    vector<string> vImg_Name, vImg_Idx;
    vector<double> vImg_Time;
    Tool.ReadImage(Set.sReadPath_ImgList, vImg_Name, vImg_Idx, vImg_Time);
    int vImg_num = vImg_Name.size();

    vector<float> vTimePerImg;
    vTimePerImg.resize(vImg_num);

    // 4. system begin
    cv::Mat im, imUn;
    TextSLAM::system SLAM(Set.mK, &Set, vImg_num);
    for(size_t ni = 0; ni<vImg_num; ni++){

        // a) Read text info
        vector<vector<Eigen::Matrix<double,2,1>>> vTextDete;
        vector<TextSLAM::TextInfo> vTextMean;
        Tool.ReadText(Set.sReadPath + vImg_Idx[ni], vTextDete, vTextMean, Set.eExp_name, Set.Flag_noText);
        assert(vTextDete.size()==vTextMean.size());

        // b) Read image info
        im = cv::imread(Set.sReadPath + vImg_Name[ni], cv::IMREAD_UNCHANGED);
        double tframe = vImg_Time[ni];
        if(im.empty()){
            cerr << endl << "Failed to load image at: "
                 << Set.sReadPath << vImg_Name[ni] << endl;
            exit(-1);
        }

        // c) undistor Img
        cv::undistort(im, imUn, Set.mKcv, Set.mDistcv, Set.mKcv);

        // ---- log ----
        if(ni%500==0){
            cout << "processing image: "<<vImg_Name[ni]<<endl;
            cout<<"......"<<endl;
        }
        std::chrono::steady_clock::time_point t_Begin = std::chrono::steady_clock::now();
        // ---- log ----

        // d) pass img to tracker
        SLAM.TrackMonocular(imUn,tframe, Set.mK, vTextDete, vTextMean);

        // ---- log ----
        std::chrono::steady_clock::time_point t_End = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t_End - t_Begin).count();
        vTimePerImg[ni]=ttrack;
        // ---- log ----
    }

    // "keyframe.txt"
    SLAM.RecordKeyFrame();

    // ---- Timing Statistics (same format as NavOCR/PaddleOCR for fair comparison) ----
    if (vImg_num > 0) {
        double total_time = 0.0;
        double min_time = vTimePerImg[0];
        double max_time = vTimePerImg[0];
        for (size_t i = 0; i < vImg_num; i++) {
            total_time += vTimePerImg[i];
            if (vTimePerImg[i] < min_time) min_time = vTimePerImg[i];
            if (vTimePerImg[i] > max_time) max_time = vTimePerImg[i];
        }
        double avg_time = total_time / vImg_num;

        cout << "============================================================" << endl;
        cout << "TextSLAM Final Statistics:" << endl;
        cout << "  Total frames: " << vImg_num << endl;
        cout << "  Total processing time: " << total_time << "s" << endl;
        cout << "  Avg processing time: " << avg_time << "s" << endl;
        cout << "  Min processing time: " << min_time << "s" << endl;
        cout << "  Max processing time: " << max_time << "s" << endl;
        cout << "  Avg FPS: " << 1.0 / avg_time << endl;
        cout << "============================================================" << endl;

        // Save timing data to file
        string timing_file = Set.sReadPath + "timing_statistics.txt";
        ofstream timing_out(timing_file);
        if (timing_out.is_open()) {
            timing_out << "=== TextSLAM Timing Statistics ===" << endl;
            timing_out << "Total frames: " << vImg_num << endl;
            timing_out << "Total processing time: " << total_time << "s" << endl;
            timing_out << "Avg processing time: " << avg_time << "s" << endl;
            timing_out << "Min processing time: " << min_time << "s" << endl;
            timing_out << "Max processing time: " << max_time << "s" << endl;
            timing_out << "Avg FPS: " << 1.0 / avg_time << endl;
            timing_out << endl << "Per-frame timing (seconds):" << endl;
            for (size_t i = 0; i < vImg_num; i++) {
                timing_out << i << "," << vTimePerImg[i] << endl;
            }
            timing_out.close();
            cout << "Timing statistics saved to: " << timing_file << endl;
        }
    }

    return 0;
}
