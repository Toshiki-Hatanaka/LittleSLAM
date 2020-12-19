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
    //printf("この地図のエッジIDは%d\n", sl[i].getPointCloudMap()->GetEdgeId());

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
    //ここだと同時にinfoに入った時更新されない?
      std::vector<PoseGraph*> pgs;
      for(int i = 0; i < edgeNumber; i++){
        pgs.emplace_back(sl[i].getPoseGraph());
      }
      //pgCloudに各エッジのノードとアーク情報をコピーしてくる
      PoseGraph *pgCloud = new PoseGraph();
      for(int i = 0; i < edgeNumber; i++){
        std::copy(pgs[i]->nodes.begin(), pgs[i]->nodes.end(), std::back_inserter(pgCloud->nodes));
        std::copy(pgs[i]->arcs.begin(), pgs[i]->arcs.end(), std::back_inserter(pgCloud->arcs));
      }
      //２つ限定の書き方してる
      firstEdgeNodeSize = pgs[0]->nodes.size();

      for(int i = 0; i < edgeNumber; i++){
        LoopDetectorSS *lpss = new LoopDetectorSS();
        fcustom->setupLpss(*lpss);
        LoopInfo *info = nullptr;
        //2つ限定の書き方してる
        curEdgeId = i;
        refEdgeId = 1 -i;
        info = lpss->detectLoopCloud(sl[curEdgeId].getPointCloudMap(), sl[refEdgeId].getPointCloudMap(),cnt);
        if(info != nullptr){
        //printf("グローバルのポーズグラフはノードが%lu個\n", pgCloud->nodes.size());

          makeLoopArcCloud(info, pgCloud);                        //ループアークのvectorに登録
          for(int i = 0; i < loopArcs.size(); i++){
            PoseArc arc = loopArcs[i];
            //printf("このアークはエッジ%dの%d番からエッジ%dの%d番へ\n", arc.src->edgeId, arc.src->nid, arc.dst->edgeId, arc.dst->nid);
            pgCloud->addArc(&arc);
          }
          sback.setPoseGraph(pgCloud);
          Pose2D newPose = sback.adjustPoses();                      //新しいポーズが得られた
          for(int j = 0; j < edgeNumber; j++){
            sback.remakeMapsCloud(sl[j].getPointCloudMap(), firstEdgeNodeSize);
          }

          printf("ポーズ調整抜けた\n");
        }
        //謎
        //ここでバグってるよ
        delete info;
        delete lpss;
      }      
      delete pgCloud;
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
void SlamEdgeCloud::makeLoopArcCloud(LoopInfo* info, PoseGraph* pg){

  if (info->arcked)                                             // infoのアークはすでに張ってある
    return;
  info->setArcked(true);
  int srcCloudId = firstEdgeNodeSize * info->refEdgeId + info->refId;
  int dstCloudId = firstEdgeNodeSize * info->curEdgeId + info->curId;
  //printf("エッジ%dの%dからエッジ%dの%dへ、実際は%dから%dへ\n",info->refEdgeId, info->refId, info->curEdgeId, info->curId, srcCloudId,dstCloudId);
  Pose2D srcPose = pg->nodes[srcCloudId]->pose;                   // 前回訪問点の位置
  Pose2D dstPose(info->pose.tx, info->pose.ty, info->pose.th);    // 再訪点の位置
  Pose2D relPose;
  Pose2D::calRelativePose(dstPose, srcPose, relPose);          // ループアークの拘束

  // アークの拘束は始点ノードからの相対位置なので、共分散をループアークの始点ノード座標系に変換
  Eigen::Matrix3d cov;
  CovarianceCalculator::rotateCovariance(srcPose, info->cov, cov, true);    // 共分散の逆回転

  PoseArc *arc = pg->makeArc(srcCloudId, dstCloudId, relPose, cov);        // ループアーク生成
  //PoseArc arc = *(pg->makeArcCloud(info->refId, srcCloudId, info->curId, dstCloudId, relPose, cov));        // ループアーク生成
  loopArcs.emplace_back(*arc);
}

