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
    sl[i].customizeFramework();               //基本的にcustmizeHで設定
    eof[i] = sl[i].getEof();                  //実行終了したかのチェック
    idx += 2;
  }
}

void SlamEdgeCloud::cloudRun(){
  //ループで各エッジがsfront.process()と描画を行っている
  sback.setPoseGraph(pgCloud);
  fcustom->setupLpss(*lpss);
  while(!eofAll){
    for(int i = 0; i < edgeNumber; i++){
      sl[i].runEC();
      eof[i] = sl[i].getEof();                //実行終了したかのチェック　//COM
    }

    //SLAMが終了したかのチェック
    eofAll = true;
    for(int i = 0; i < edgeNumber; i++){
      if(eof[i] == false){
        eofAll = false;
      }
    }
    
    //ここからクラウドとエッジのデータのやりとりが沢山ある
    //クラウドによるループ検出
    if (cnt > keyframeSkip && cnt%keyframeSkip==0) {       // キーフレームのときだけ行う
      for(int i = 0; i < edgeNumber; i++){
        std::vector<PoseGraph*> pgs;
        for(int j = 0; j < edgeNumber; j++){
          pgs.emplace_back(sl[j].getPoseGraph());             //COM
        }
        //pgCloudに各エッジのノードとアーク情報をコピーしてくる
        pgCloud->nodes.clear();
        pgCloud->arcs.clear();
        for(int j = 0; j < edgeNumber; j++){
          std::copy(pgs[j]->nodes.begin(), pgs[j]->nodes.end(), std::back_inserter(pgCloud->nodes));          
          std::copy(pgs[j]->arcs.begin(), pgs[j]->arcs.end(), std::back_inserter(pgCloud->arcs));             
        }
        //２つ限定の書き方してる
        firstEdgeNodeSize = pgs[0]->nodes.size(); 
        curEdgeId = i;
        refEdgeId = 1 -i;
        //ここ無駄かもしれないね

        LoopInfo *info = nullptr;
        //2つ限定の書き方してる

        //一方のエッジ（curEdge)の現在位置を見て、その位置がもう一方のエッジが作成した地図（refmap）の近くならマージ可能かを疑う
        //その候補地についてICPを行い、refmapにおけるcurEdgeの修正位置を求める
        //２つのエッジの間でループアークを結ぶのに必要な情報がinfoに格納される
        info = lpss->detectLoopCloud(sl[curEdgeId].getPointCloudMap(), sl[refEdgeId].getPointCloudMap(),cnt);    //COM

        //正しく検出され、再訪点が求められている時
        if(info != nullptr){
          makeLoopArcCloud(info, pgCloud);                        //クラウドが持つループアークのvectorに登録
          for(int j = 0; j < loopArcs.size(); j++){
            PoseArc *arc = loopArcs[j];

            //これまで作ったループアークをすべてpgCloudにセット
            pgCloud->addArc(arc);                                                           
          }

          //2つめの重なりのあたりでバグ
          //sback.setPoseGraph(pgCloud);                             //もっと上の方でもいいのでは
          sback.adjustPoses(firstEdgeNodeSize);                      //新しいポーズが得られた
          for(int j = 0; j < edgeNumber; j++){
            sback.remakeMapsCloud(sl[j].getPointCloudMap(), firstEdgeNodeSize);                               //COM
          }
        }
        //謎
        delete info;
      }      
    }
    //drawSkipごとに描画    
    if(cnt % drawSkip == 0 && cnt != 0){
      //２つ限定でやっている
        mdrawerWorld.drawMapWorld(*sl[0].getPointCloudMap(), *sl[1].getPointCloudMap(), edgeNumber);
    }
    ++cnt;
  }
}
void SlamEdgeCloud::makeLoopArcCloud(LoopInfo* info, PoseGraph* pg){
  //ここでのpgはpgCloudである
  //infoには再訪点の位置、共分散、再訪点のノードid・エッジid、前回訪問点のノードid・エッジidが格納されている
  if (info->arcked)                                             // infoのアークはすでに張ってある
    return;
  info->setArcked(true);
  int srcCloudId = firstEdgeNodeSize * info->refEdgeId + info->refId;               //pgCloudに関する通し番号
  int dstCloudId = firstEdgeNodeSize * info->curEdgeId + info->curId;               //pgCloudに関する通し番号
  //printf("アーク張る直前: エッジ%dの%dからエッジ%dの%dへ、実際は%dから%dへ\n",info->refEdgeId, info->refId, info->curEdgeId, info->curId, srcCloudId,dstCloudId);
  Pose2D srcPose = pg->nodes[srcCloudId]->pose;                   // 前回訪問点の位置
  Pose2D dstPose(info->pose.tx, info->pose.ty, info->pose.th);    // 再訪点の位置
  Pose2D relPose;
  Pose2D::calRelativePose(dstPose, srcPose, relPose);          // ループアークの拘束

  // アークの拘束は始点ノードからの相対位置なので、共分散をループアークの始点ノード座標系に変換
  Eigen::Matrix3d cov;
  CovarianceCalculator::rotateCovariance(srcPose, info->cov, cov, true);    // 共分散の逆回転

  PoseArc *arc = pg->makeArc(srcCloudId, dstCloudId, relPose, cov);        // ループアーク生成
  //ループアークであることを明示的に
  arc->loop = true;
  loopArcs.emplace_back(arc);
}

