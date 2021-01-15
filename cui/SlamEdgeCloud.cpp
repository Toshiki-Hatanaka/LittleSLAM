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

    printf("総実行時間は%lf, 総描画時間は%lf, 総マージ時間は%lf\n", totalTime, totalTimeDraw, totalTimeMerge);
    printf("エッジ0のSLAM実行時間は%lf, エッジ1のSLAM実行時間は%lf\n", totalTimeScanMatch[0], totalTimeScanMatch[1]);
    printf("SlamLauncher finished.\n");
    while(true) {
      usleep(1000);                        // Linuxではusleep
    }
    return;    
}

//各slにファイルとstartNとframeworkをセット
void SlamEdgeCloud::setup(int argc, char *argv[], int idx){


  // 正解データ読み込み
  for(int i = 0; i < edgeNumber; i++){
    const char *ansFileName;
    if(i == 0){
      ansFileName = "truePose0.txt";
    }
    else{
      ansFileName = "truePose1.txt";
    }
    std::ifstream ifs(ansFileName);

    double tx, ty, th;
    while(!ifs.eof()){
      ifs >> tx >> ty >> th;
      //std::cout << tx << " " << ty << " " << th << std::endl;
      Pose2D pose;
      pose.tx = tx, pose.ty = ty, pose.th = th;
      truePoses[i].emplace_back(pose);
    }
  }
  

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
    double t1[2];
    for(int i = 0; i < edgeNumber; i++){
      sl[i].runEC();
      eof[i] = sl[i].getEof();                //実行終了したかのチェック　//COM
      t1[i] = tim.elapsed();
    }

    //SLAMが終了したかのチェック
    eofAll = true;
    for(int i = 0; i < edgeNumber; i++){
      if(eof[i] == false){
        eofAll = false;
      }
    }
    //クラウドによるループ検出
    //if (cnt < 0){                                            //呼ばない時
    //if (cnt > keyframeSkip){                               //毎フレーム
    if (cnt > keyframeSkip && cnt%keyframeSkip==0) {       // キーフレームのときだけ行う
      for(int i = 0; i < edgeNumber; i++){
        if(!eof[i]){
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
            //sback.setPoseGraph(pgCloud);                             //もっと上の方でもいいのでは
            sback.adjustPoses(firstEdgeNodeSize);                      //新しいポーズが得られた
            for(int j = 0; j < edgeNumber; j++){
              sback.remakeMapsCloud(sl[j].getPointCloudMap(), firstEdgeNodeSize);                               //COM
              feedbackPoints[j][cnt] = sl[j].getPointCloudMap()->globalMap.size();
            }
          }
          //謎
          delete info;
        }
      }      
    }

    double t2 = tim.elapsed();
    //drawSkipごとに描画    
    if(cnt % drawSkip == 0 && cnt != 0){
      //２つ限定でやっている
      mdrawerWorld.drawMapWorld(*sl[0].getPointCloudMap(), *sl[1].getPointCloudMap(), edgeNumber);
    }
    double t3 = tim.elapsed();

    //二乗誤差測定
    
    for(int i = 0; i < edgeNumber; i++){
      PoseGraph *pg = sl[i].getPoseGraph();
      vector<PoseNode*> &nodes = pg->nodes;
      double d = 0;
      for(int j = 0; j < nodes.size(); j++){
        PoseNode *node = nodes[j];
        Pose2D pose = node->pose; 
        Pose2D truePose = truePoses[i][j];

        d += sqrt( pow((truePose.tx - pose.tx), 2) + pow((truePose.ty - pose.ty), 2) );
      }
      disdiff[i].emplace_back(d);
      disdiffaverage[i].emplace_back(d/nodes.size());
    }
    
    ++cnt;

    totalTime = t3;
    totalTimeDraw += t3 - t2;
    totalTimeMerge += t2 - t1[1];
    totalTimeScanMatch[1] += t1[1] - t1[0];
    totalTimeScanMatch[0] += t1[0] - totalTimePrev;
    
    totalTimePrev = totalTime;
  }
  for(int i = 0; i < edgeNumber; i++){
    const char *outFileTruePose;
    if(i == 0){
      outFileTruePose = "myPoseout0.txt";
    }
    else{
      outFileTruePose = "myPoseout1.txt";
    }
    std::ofstream ofsTrue(outFileTruePose);
      PoseGraph *pg = sl[i].getPoseGraph();
      vector<PoseNode*> &nodes = pg->nodes;
      for(int j = 0; j < nodes.size(); j++){
        PoseNode *node = nodes[j];
        Pose2D pose = node->pose; 
        ofsTrue << pose.tx << " " << pose.ty << " " << pose.th << endl;
      }
    ofsTrue.close();
  }

  for(int i = 0; i < edgeNumber; i++){
    const char *outFileName;
    const char *outFileNameaverage;
    const char *outFileFeedback;
    if(i == 0){
      outFileName = "disdiff0.txt";
      outFileNameaverage = "disdiffaverage0.txt";
      outFileFeedback = "feedbackPoints0.txt";
    }
    else{
      outFileName = "disdiff1.txt";
      outFileNameaverage = "disdiffaverage1.txt";
      outFileFeedback = "feedbackPoints1.txt";
    }
    std::ofstream ofs(outFileName);
    std::ofstream ofsaverage(outFileNameaverage);
    std::ofstream ofsfeedback(outFileFeedback);
    for(int j = 0; j < disdiff[i].size(); j++){
      ofs << j << " " << disdiff[i][j] << endl;
      ofsaverage << j << " " << disdiffaverage[i][j] << endl;
    }

    for(auto itr = feedbackPoints[i].begin(); itr != feedbackPoints[i].end(); ++itr){
      ofsfeedback << itr->first << " " << itr->second << endl;
    }
    ofs.close();
    ofsaverage.close();
    ofsfeedback.close();
  }
  double t4 = tim.elapsed();
  printf("書き込み時間は%lf\n", t4 - totalTime);

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

