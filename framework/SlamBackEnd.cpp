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
 * @file SlamBackEnd.cpp
 * @author Masahiro Tomono
 ****************************************************************************/

#include "SlamBackEnd.h"
#include "P2oDriver2D.h"

using namespace std;

////////// ポーズ調整 //////////

Pose2D SlamBackEnd::adjustPoses(int firstEdgeNodeSize) {
//  pg->printArcs();
//  pg->printNodes();

  newPoses.clear();

  P2oDriver2D p2o;
  p2o.doP2o(*pg, newPoses, firstEdgeNodeSize, 5);                 // 5回くり返す

  return(newPoses.back());
}

/////////////////////////////

void SlamBackEnd::remakeMaps() {
  // PoseGraphの修正
  vector<PoseNode*> &pnodes = pg->nodes;      // ポーズノード
  for (size_t i=0; i<newPoses.size(); i++) {
    Pose2D &npose = newPoses[i];
    PoseNode *pnode = pnodes[i];              // ノードはロボット位置と1:1対応
    pnode->setPose(npose);                    // 各ノードの位置を更新
  }
  //printf("newPoses.size=%lu, nodes.size=%lu\n", newPoses.size(), pnodes.size());

  // PointCloudMapの修正

  //コアダンプしてる
  pcmap->remakeMaps(newPoses);
}

//各ノードに行う
void SlamBackEnd::remakeMapsCloud(PointCloudMap *pcmap, int firstEdgeNodesSize){
  //ここのpgはpgCloud
  vector<PoseNode*> &pnodes = pg->nodes;      // ポーズノード
  int edgeId = pcmap->edgeId;
  size_t startIndex = firstEdgeNodesSize * edgeId;
  size_t endIndex = firstEdgeNodesSize * edgeId + pcmap->poses.size();

  //printf("pgCloudのノード数は%d, エッジIDは%d, startIndexは%zu, endIndexは%zu\n", pg->nodes.size(), edgeId, startIndex, endIndex);
  std::vector<Pose2D> newPosesEach;            // ポーズ調整後の姿勢

  for (size_t i = startIndex; i < endIndex; i++) {
    Pose2D &npose = newPoses[i];
    newPosesEach.emplace_back(npose);
    PoseNode *pnode = pnodes[i];              // ノードはロボット位置と1:1対応
    pnode->setPose(npose);                    // 各ノードの位置を更新

    //各posegraphは？
  }

  // PointCloudMapの修正

  //コアダンプしてる
  pcmap->remakeMaps(newPosesEach);
}