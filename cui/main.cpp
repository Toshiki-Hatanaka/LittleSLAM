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
 * @file main.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SlamLauncher.h"

int main(int argc, char *argv[]) {
  bool scanCheck=false;              // スキャン表示のみか
  bool odometryOnly=false;           // オドメトリによる地図構築か
  bool edgeCloudSLAM=false;
  char *filename[5];                    // データファイル名
  int startN[5]={0, 0, 0, 0, 0};        // 開始スキャン番号
  SlamLauncher sl[5];
  bool eof[5];

  if (argc < 2) {
    printf("Error: too few arguments.\n");
    return(1);
  }

  // コマンド引数の処理
  int idx=1;
  // コマンドオプションの解釈（'-'のついた引数）
  if (argv[1][0] == '-') {
    for (int i=1; ; i++) {
      char option = argv[1][i];
      if (option == NULL)
        break;
      else if (option == 's')        // スキャン表示のみ
        scanCheck = true;
      else if (option == 'o')        // オドメトリによる地図構築
        odometryOnly = true;
      else if (option == 'e'){
        edgeCloudSLAM = true;
      }
    }
    printf("SlamLauncher: startN=%d, scanCheck=%d, odometryOnly=%d, edgeCloudSLAM=%d\n", startN[0], scanCheck, odometryOnly, edgeCloudSLAM);
    if (argc == 2) {
      printf("Error: no file name.\n");
      return(1);
    }
    ++idx;
  }
  if(!edgeCloudSLAM){                  //シングルモード
    if (argc >= idx+1)                 // '-'ある場合idx=2、ない場合idx=1
      filename[0] = argv[idx];
    if (argc == idx+2)                 // argcがidxより2大きければstartNがある
      startN[0] = atoi(argv[idx+1]);
    else if (argc >= idx+2) {
      printf("Error: invalid arguments.\n");
      return(1);
    }

    printf("filename=%s\n", filename[0]);

    // ファイルを開く

    bool flag = sl[0].setFilename(filename[0]);
    if (!flag)
      return(1);

    sl[0].setStartN(startN[0]);              // 開始スキャン番号の設定

    // 処理本体
    if (scanCheck)
      sl[0].showScans();
    else {                             // スキャン表示以外はSlamLauncher内で場合分け
      sl[0].setOdometryOnly(odometryOnly);
      sl[0].customizeFramework();
      sl[0].run();
    }
  }
  // エッジクラウドモード　
  // LittleSLAM -w 2 dataset1 startN1 dataset2 startN2 
  else{
    int edgeNumber = atoi(argv[idx]); //エッジ端末の数

    if(edgeNumber == 0){                           //エッジ数のところの入力エラー
      printf("Error: invalid arguments(2).\n");
      return(1);
    }
    if(argc != edgeNumber * 2 + 3){
      printf("Error: invalid arguments(3).\n");    //ちゃんとファイル名とstartNを正しく入力しているかの確認
      return(1);
    }
    idx++;
    //各slにファイルとstartNとframeworkをセット
    for(int i = 0; i < edgeNumber; i++){
      filename[i] = argv[idx];
      startN[i] = atoi(argv[idx + 1]);
      bool flag = sl[i].setFilename(filename[i]);
      if(!flag){
        return(1);
      }
      sl[i].setStartN(startN[i]);
      sl[i].customizeFramework();
      printf("エッジ%dのfilename=%s, startN=%d\n", i, filename[i], startN[i]);
      sl[i].setupEC();
      eof[i] = sl[i].getEof();
      idx += 2;
    }
    bool eofAll = false;

    //ただループでsfront.process()と描画を行っているだけ
    while(!eofAll){
      for(int i = 0; i < edgeNumber; i++){
        sl[i].runEC();
        eof[i] = sl[i].getEof();
      }
      eofAll = true;
      for(int i = 0; i < edgeNumber; i++){
        if(eof[i] == false){
          eofAll = false;
        }
      }
    }
    usleep(10000);  
    
  }
  return(0);
}
