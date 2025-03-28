#pragma once

#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <stdio.h>
#include <string>

void LoadIntrinsic(const std::string &filename, Eigen::Matrix3d &K,
                   std::vector<double> &dist, int &img_col, int &img_row) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in(filename, std::ios::binary);

  // std::ifstream in;
  // in.open(filename);
  if (!in.is_open()) {
    std::cout << "Error Opening " << filename << std::endl;
    return;
  }
  if (reader.parse(in, root, false)) {
    Json::Value::Members name = root.getMemberNames();
    std::string id = *(name.begin());
    std::cout << id << std::endl;
    Json::Value intri = root[id]["param"]["cam_K"]["data"];
    Json::Value d = root[id]["param"]["cam_dist"]["data"];
    img_col = root[id]["param"]["img_dist_w"].asInt();
    img_row = root[id]["param"]["img_dist_h"].asInt();
    int dist_col = root[id]["param"]["cam_dist"]["cols"].asInt();
    K << intri[0][0].asDouble(), intri[0][1].asDouble(), intri[0][2].asDouble(),
        intri[1][0].asDouble(), intri[1][1].asDouble(), intri[1][2].asDouble(),
        intri[2][0].asDouble(), intri[2][1].asDouble(), intri[2][2].asDouble();
    dist.clear();
    for (int i = 0; i < dist_col; i++) {
      dist.push_back(d[0][i].asDouble());
    }
  }
  in.close();

  return;
}
