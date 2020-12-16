#include "SlamEdgeCloud.h"

using namespace std;                       // C++標準ライブラリの名前空間を使う



void SlamEdgeCloud::mainProcess(int argc, char *argv[], int idx){
    edgeNumber = atoi(argv[idx]); //エッジ端末の数

    if(edgeNumber == 0){                           //エッジ数のところの入力エラー
      printf("Error: invalid arguments(2).\n");
      return;
    }
    if(argc != edgeNumber * 2 + 3){
      printf("Error: invalid arguments(3).\n");    //ちゃんとファイル名とstartNを正しく入力しているかの確認
      return;
    }
    idx++;
    //各slにファイルとstartNとframeworkをセット
    setup(argc, argv, idx);

    mdrawerWorld.initGnuplot();                   // gnuplot初期化
    mdrawerWorld.setAspectRatio(-0.9);            // x軸とy軸の比（負にすると中身が一定）

    while(!eofAll){
        cloudRun();
    }

    printf("SlamLauncher finished.\n");
    while(true) {
        usleep(1000);                        // Linuxではusleep
    }
    return;    
    
}

//各slにファイルとstartNとframeworkをセット
void SlamEdgeCloud::setup(int argc, char *argv[], int idx){
    for(int i = 0; i < edgeNumber; i++){
      filename[i] = argv[idx];
      startN[i] = atoi(argv[idx + 1]);
      bool flag = sl[i].setFilename(filename[i]);
      if(!flag){
        return;
      }
      sl[i].setStartN(startN[i]);
      sl[i].setupEC(i);
      sl[i].customizeFramework();
      printf("この地図のエッジIDは%d\n", sl[i].getPointCloudMap()->GetEdgeId());

      eof[i] = sl[i].getEof();
      idx += 2;

    }
}

void SlamEdgeCloud::cloudRun(){
    //ただループでsfront.process()と描画を行っているだけ
    while(!eofAll){
      for(int i = 0; i < edgeNumber; i++){
        sl[i].runEC();
        eof[i] = sl[i].getEof();
      }

      //SLAMが終了したかのチェック
      eofAll = true;
      for(int i = 0; i < edgeNumber; i++){
        if(eof[i] == false){
          eofAll = false;
        }
      }

      //クラウドによるループ検出
      if (cnt > keyframeSkip && cnt%keyframeSkip==0) {       // キーフレームのときだけ行う
        //最初はエッジ1がエッジ0の軌跡を見つけるだけで
        //const std::vector<Submap> submaps = ((PointCloudMapLP*)sl[0].getPointCloudMap())->submaps;
        LoopDetectorSS *lpss = new LoopDetectorSS();

        fcustom->setupLpss(*lpss);
        lpss->detectLoopCloud(sl[0].getPointCloudMap(), sl[1].getPointCloudMap(),cnt);
        lpss->detectLoopCloud(sl[1].getPointCloudMap(), sl[0].getPointCloudMap(),cnt);
        
        delete lpss;
      }
      //drawSkipごとに描画    
      if(cnt % drawSkip == 0 && cnt != 0){
        //よくわからんから２つ限定でやろう
          mdrawerWorld.drawMapWorld(*sl[0].getPointCloudMap(), *sl[1].getPointCloudMap(), edgeNumber);
        //mdrawerWorld.drawMapMove(*sl[0].getPointCloudMap(), 5, 5, (double)1/2 * M_PI);
      }
      ++cnt;
    }
}
void SlamEdgeCloud::test (int argc, char *argv[], int idx){
    return;
}

