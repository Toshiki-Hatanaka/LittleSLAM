#ifndef SLAM_EDGE_CLOUD_H_
#define SLAM_EDGE_CLOUD_H_

#include "SlamLauncher.h"
#include <vector>
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#include<iostream>
#include<fstream>
#endif


class SlamEdgeCloud{
    private:
        SlamLauncher sl[5];                   // SlamLauncher
        char *filename[5];                    // データファイル名
        int startN[5]={0, 0, 0, 0, 0};        // 開始スキャン番号
        bool eof[5];

        MapDrawer mdrawerWorld;
        FrameworkCustomizer *fcustom;     // フレームワークの改造
        PoseGraph *pgCloud;
        LoopDetectorSS *lpss;
        SlamBackEnd sback;
        std::vector<PoseArc*> loopArcs;         // ループアークの集合。アークは片方向のみもつ
        int edgeNumber;
        size_t cnt = 0;  
        bool eofAll = false;
        int drawSkip;
        int keyframeSkip;
        
        int curEdgeId;
        int refEdgeId;
        int firstEdgeNodeSize;

        boost::timer tim;
        double totalTime=0;
        double totalTimePrev=0;
        double totalTimeDraw=0;
        double totalTimeScanMatch[2] = {0, 0};
        double totalTimeMerge = 0;

        std::vector<Pose2D> truePoses[2];
        std::vector<double> disdiff[2];
        std::vector<double> disdiffaverage[2];
        std::map<int, size_t> feedbackPoints[2];
    public:
        SlamEdgeCloud() :  drawSkip(10), keyframeSkip(10), fcustom(new FrameworkCustomizer()), pgCloud(new PoseGraph()), lpss(new LoopDetectorSS()){
        }
        void mainProcess(int argc, char *argv[], int idx);
        void setup(int argc, char *argv[], int idx);
        void cloudRun();
        void makeLoopArcCloud(LoopInfo* info, PoseGraph* pg);
};




#endif