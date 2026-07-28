#include <cassert>
#include <iostream>

#include <Eigen/Dense>

#include "patchwork/patchwork.h"

int main() {
  patchwork::PatchworkParams params;
  patchwork::PatchWork pw(params);

  Eigen::MatrixXf cloud(0, 4);
  pw.estimateGround(cloud);

  assert(pw.getGround().rows() == 0);
  assert(pw.getNonground().rows() == 0);
  std::cout << "patchwork smoke: ok" << std::endl;
  return 0;
}
