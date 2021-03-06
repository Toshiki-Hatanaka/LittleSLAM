﻿/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file SlamLauncher.h
 * @author Masahiro Tomono
 ****************************************************************************/

#ifndef SLAM_LAUNCHER_H_
#define SLAM_LAUNCHER_H_

#include <vector>
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <unistd.h>
#endif

#include <boost/timer.hpp>
#include "SensorDataReader.h"
#include "PointCloudMap.h"
#include "PointCloudMapLP.h"
#include "SlamFrontEnd.h"
#include "SlamBackEnd.h"
#include "MapDrawer.h"
#include "FrameworkCustomizer.h"
#include "PoseGraph.h"

/////////////

class SlamLauncher
{
private:
  int startN;                      // 開始スキャン番号
  int drawSkip;                    // 描画間隔
  bool odometryOnly;               // オドメトリによる地図構築か

  size_t cnt = 0;                  // 処理の論理時刻
  Scan2D scan;
  bool eof;
  boost::timer tim;
  double totalTime=0, totalTimeDraw=0, totalTimeRead=0;

  double totalTimeSLAM = 0;
  int edgeId = 0;
  std::vector<int> lpsNum;
  std::vector<Pose2D> Posessingle;
  bool written = false;

  Pose2D ipose;                    // オドメトリ地図構築の補助データ。初期位置の角度を0にする
  Pose2D lidarOffset;              // レーザスキャナとロボットの相対位置
  SensorDataReader sreader;        // ファイルからのセンサデータ読み込み
  //PointCloudMapLP *pcmap;            // 点群地図
  PointCloudMap *pcmap;
  SlamFrontEnd sfront;             // SLAMフロントエンド
  MapDrawer mdrawer;               // gnuplotによる描画
  FrameworkCustomizer fcustom;     // フレームワークの改造

public:
  SlamLauncher() : startN(0), drawSkip(10), odometryOnly(false), pcmap(nullptr) {
  }

  ~SlamLauncher() {
  }

///////////

  void setStartN(int n) {
    startN = n;
  }

  void setOdometryOnly(bool p) {
    odometryOnly = p;
  }

  bool getEof(){
    return eof;
  }

  PointCloudMap *getPointCloudMap() {
    return(pcmap);
  }

  PoseGraph *getPoseGraph() {
    return(sfront.getPoseGraph());
  }


///////////

  void run();
  void showScans();
  void mapByOdometry(Scan2D *scan);
  bool setFilename(char *filename);
  void skipData(int num);
  void customizeFramework();
  void setupEC(int edgeId);
  void runEC();

};

#endif
