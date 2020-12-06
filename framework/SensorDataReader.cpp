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
    isScan = loadLaserScanNew(cnt, scan);
  }

  if (isScan) 
    return(false);                       // まだファイルが続くという意味
  else
    return(true);                        // ファイルが終わったという意味
}

//////////////
//スキャン歪みを考慮したもの。
bool SensorDataReader::loadLaserScanNew(size_t cnt, Scan2D &scan) {
  string FLASER;
  inFile >> FLASER;                      // ファイル内の項目ラベル
  if(FLASER == "FLASER"){
    vector<LPoint2D> lps;
    int cnt;
    inFile >> cnt;                       //スキャン番号
    int pnum;                            // スキャン点数
    inFile >> pnum;
    //printf("点数は%d, ", pnum);
    lps.reserve(pnum);
    double range_old[500];
    for (int i=0; i<pnum; i++) {
    inFile >> range_old[i];
    }
    double x_s, y_s, theta_s;             //0番目の点におけるオドメトリ値
    inFile >> x_s >> y_s >> theta_s;

    //読み飛ばす
    float odo_x, odo_y, odo_theta, timestamp_s, logTimestamp_s;
    string hostname_s;
    inFile >> odo_x >> odo_y >> odo_theta >> timestamp_s;
    inFile >> hostname_s;
    inFile >> logTimestamp_s;

    string ODOM;
    inFile >> ODOM;                      // ファイル内の項目ラベル

    double x_g, y_g, theta_g;
    inFile >> x_g >> y_g >> theta_g;    //pnum番目の点におけるオドメトリ値

    //読み飛ばす
    double tv_g, rv_g, accel_g, timestamp_g, logTimestamp_g;
    string hostname;
    inFile >> tv_g >> rv_g >> accel_g >> timestamp_g;
    inFile >> hostname;
    inFile >> logTimestamp_g;

    //1スキャンの間の移動量
    double dx, dy, dtheta;
    dx = x_g - x_s;
    dy = y_g - y_s;
    dtheta = theta_g - theta_s;   //ラジアン

    for(int i = 0; i < pnum; i++){
    
      float range, angle;
      range = range_old[i];
      angle = (float)180/(pnum - 1) * i + angleOffset; //弧度法

      /*
      angle = (float)i/(pnum - 1) * (180 - d_theta) + angleOffset;
      range = range_old[i] - (float)i * sqrt(pow(d_x, 2) + pow(d_y, 2)) /(pnum - 1);
      */
      if (range <= Scan2D::MIN_SCAN_RANGE || range >= Scan2D::MAX_SCAN_RANGE) {
        continue;
      }
      LPoint2D lp;
      lp.setSid(cnt);                    // スキャン番号はcnt（通し番号）にする
      lp.calXY(range, angle);
      //lp.calXYCorrect(range, angle, dx, dy, dtheta, i, pnum);            // angle,rangeから点の位置xyを計算
      lps.emplace_back(lp);
    }
    scan.setLps(lps);

    // スキャンに対応するオドメトリ情報
    Pose2D &pose = scan.pose;
    pose.tx = x_s;
    pose.ty = y_s;
    pose.setAngle(RAD2DEG(theta_s));          // オドメトリ角度はラジアンなので度にする
    pose.calRmat();
    
    return(true);
  }
  else{
    string line;
    getline(inFile, line);               // 読み飛ばす

    return(false);
  }

}

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
    printf("idは%d\n, ", sid);

    vector<LPoint2D> lps;
    int pnum;                            // スキャン点数
    inFile >> pnum;
    //printf("点数は%d, ", pnum);
    lps.reserve(pnum);
    for (int i=0; i<pnum; i++) {
      float angle, range;
      
      inFile >> range;          // スキャン点の方位と距離
      angle = (float)180/(pnum - 1) * i + angleOffset;
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
    float x1, y1, theta1;
    inFile >> x1 >> y1 >> theta1;
    
    //読み飛ばす
    float timestamp1;
    inFile >> timestamp1;
    string name;
    inFile >> name;
    float logTimestamp1;
    inFile >> logTimestamp1;
    return(true);
  }
  else {                                 // スキャン以外の場合
    string line;
    getline(inFile, line);               // 読み飛ばす

    return(false);
  }
}
