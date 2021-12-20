#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;
int main() {
  // Eigen::Vector3f p(2.0f, 1.0f, 1.0f);
  // float cosA = cos((45.0 / 360) * 2 * M_PI);
  // float sinA = sin((45.0 / 360) * 2 * M_PI);
  // Eigen::Matrix<float, 3, 3> m{
  //     {cosA, -sinA, 1.0f}, {sinA, cosA, 2.0f}, {0.0f, 0.0f, 1.0f}};
  // p = m * p;
  // // std::cout << p << std::endl;
  // Vector3f v1{1, 2, 1};
  // Vector3f v2{2, 3, 1};
  // std::cout << v1.cross(v2);
  const Vector3f _v[3] = {{0, 0, 1}, {0, 4, 1}, {3, 0, 1}};
  Vector3f point{-1, 1, 1};
  bool flag = true;
  Vector3f AB = _v[0] - _v[1];
  Vector3f AO = _v[0] - point;
  Vector3f BC = _v[1] - _v[2];
  Vector3f BO = _v[1] - point;
  Vector3f CA = _v[2] - _v[0];
  Vector3f CO = _v[2] - point;
  std::cout << AO.cross(AB) << '\n'
            << BO.cross(BC) << '\n'
            << CO.cross(CA) << std::endl;
  return 0;
}
