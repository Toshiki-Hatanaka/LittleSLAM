#ifndef SLAM_EDGE_CLOUD_H_
#define SLAM_EDGE_CLOUD_H_

#include "SlamLauncher.h"
#include <vector>
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
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
    public:
        SlamEdgeCloud() :  drawSkip(10), keyframeSkip(10), fcustom(new FrameworkCustomizer()), pgCloud(new PoseGraph()), lpss(new LoopDetectorSS()){
        }
        void mainProcess(int argc, char *argv[], int idx);
        void setup(int argc, char *argv[], int idx);
        void cloudRun();
        void makeLoopArcCloud(LoopInfo* info, PoseGraph* pg);
};




#endif