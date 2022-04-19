#include <iostream>

#include "icp.h"

int main() {

    ICP icp("/home/huangyong/test/icp/data/bunny1.pcd", "/home/huangyong/test/icp/data/bunny2.pcd");
    icp.runSVDMatch();
    icp.runOptimationMatch();

    return 0;
}
