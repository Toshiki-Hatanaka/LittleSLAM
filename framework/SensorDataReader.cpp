/****************************************************************************
 * LittleSLAM: 2D-Laser SLAM for educational use
 * Copyright (C) 2017-2018 Masahiro Tomono
 * Copyright (C) 2018 Future Robotics Technology Center (fuRo),
 *                    Chiba Institute of Technology.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @file SensorDataReader.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SensorDataReader.h"

using namespace std;

// ファイルからスキャンを1個読む
bool SensorDataReader::loadScan(size_t cnt, Scan2D &scan) {
  bool isScan=false;
  while (!inFile.eof() && !isScan) {     // スキャンを読むまで続ける
    isScan = loadLaserScan(cnt, scan);
  }

  if (isScan) 
    return(false);                       // まだファイルが続くという意味
  else
    return(true);                        // ファイルが終わったという意味
}

//////////////

// ファイルから項目1個を読む。読んだ項目がスキャンならtrueを返す。
bool SensorDataReader::loadLaserScan(size_t cnt, Scan2D &scan) {
  string type;                           // ファイル内の項目ラベル
  inFile >> type;
  if (type == "LASERSCAN") {             // スキャンの場合
    scan.setSid(cnt);

    int sid, sec, nsec;
    inFile >> sid >> sec >> nsec;        // これらは使わない

    vector<LPoint2D> lps;
    int pnum;                            // スキャン点数
    inFile >> pnum;
    lps.reserve(pnum);
    for (int i=0; i<pnum; i++) {
      float angle, range;
      inFile >> angle >> range;          // スキャン点の方位と距離
      angle += angleOffset;              // レーザスキャナの方向オフセットを考慮
      if (range <= Scan2D::MIN_SCAN_RANGE || range >= Scan2D::MAX_SCAN_RANGE) {
//      if (range <= Scan2D::MIN_SCAN_RANGE || range >= 3.5) {         // わざと退化を起こしやすく
        continue;
      }

      LPoint2D lp;
      lp.setSid(cnt);                    // スキャン番号はcnt（通し番号）にする
      lp.calXY(range, angle);            // angle,rangeから点の位置xyを計算
      lps.emplace_back(lp);
    }
    scan.setLps(lps);

    // スキャンに対応するオドメトリ情報
    Pose2D &pose = scan.pose;
    inFile >> pose.tx >> pose.ty;
    double th;
    inFile >> th;
    pose.setAngle(RAD2DEG(th));          // オドメトリ角度はラジアンなので度にする
    pose.calRmat();

    return(true);
  }
  else if(type == "FLASER"){            //データセット
    scan.setSid(cnt);
    int sid;
    inFile >> sid;        // これらは使わない
    //printf("idは%d, ", sid);

    vector<LPoint2D> lps;
    int pnum;                            // スキャン点数
    inFile >> pnum;
    //printf("点数は%d, ", pnum);
    lps.reserve(pnum);
    float nowangle = 0;
    for (int i=0; i<pnum; i++) {
      float angle, range;
      
      inFile >> range;          // スキャン点の方位と距離
      angle = nowangle;
      angle += angleOffset;              // レーザスキャナの方向オフセットを考慮
      nowangle += 2.0;                  // 次のために角度をセット
      //printf("角度は%f距離は%f ", angle, range);
      if (range <= Scan2D::MIN_SCAN_RANGE || range >= Scan2D::MAX_SCAN_RANGE) {
//      if (range <= Scan2D::MIN_SCAN_RANGE || range >= 3.5) {         // わざと退化を起こしやすく
        continue;
      }

      LPoint2D lp;
      lp.setSid(cnt);                    // スキャン番号はcnt（通し番号）にする
      lp.calXY(range, angle);            // angle,rangeから点の位置xyを計算
      lps.emplace_back(lp);


    }
    //printf("\n");
    scan.setLps(lps);

    // スキャンに対応するオドメトリ情報
    Pose2D &pose = scan.pose;
    inFile >> pose.tx >> pose.ty;
    double th;
    inFile >> th;
    pose.setAngle(RAD2DEG(th));          // オドメトリ角度はラジアンなので度にする
    pose.calRmat();
    //printf("オドメトリはx = %f, y = %f, θ = %f\n", pose.tx, pose.ty, th);

    // 読み飛ばす
    float a1, a2, a3, a4;
    inFile >> a1 >> a2 >> a3 >> a4;
    string s;
    inFile >> s;
    float a5;
    inFile >> a5;
    return(true);
  }
  else {                                 // スキャン以外の場合
    string line;
    getline(inFile, line);               // 読み飛ばす

    return(false);
  }
}
