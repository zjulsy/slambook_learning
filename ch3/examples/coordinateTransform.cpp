#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
  q1.normalize();
  q2.normalize();
  Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
  Vector3d p1(0.5, 0, 0.2);

  Isometry3d T1w(q1), T2w(q2);
  T1w.pretranslate(t1);
  T2w.pretranslate(t2);
  Isometry3d T3w(q1);
  T3w.translate(t1);
  cout << "T3w : \n" << T3w.matrix() << endl;

  Vector3d p2 = T2w * T1w.inverse() * p1;
  cout << "T2w : \n" << T2w.matrix() << endl;
  cout << "T1w : \n" << T1w.matrix() << endl;
  cout << "T1w.inverse : \n" << T1w.inverse().matrix() << endl;

  cout << endl << p2.transpose() << endl;
  return 0;
}