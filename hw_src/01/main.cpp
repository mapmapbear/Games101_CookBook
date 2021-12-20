#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2],
      0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
  Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
  float cosA = cos(rotation_angle / 180.0 * MY_PI);
  float sinA = sin(rotation_angle / 180.0 * MY_PI);
  Eigen::Matrix<float, 4, 4> rotate{{cosA, -1 * sinA, 0, 0},
                                    {sinA, cosA, 0, 0},
                                    {0, 0, 1, 0},
                                    {0, 0, 0, 1}}; //单纯实现了关于z轴的旋转矩阵
  model = rotate * model;
  return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
  //在这个函数主要解决的是通过fov来计算投影平面的宽,高,长宽比算出长,
  // ZNear,ZFar分别是场景在透视投影中的近视点和远视点的深度
  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
  Matrix4f m0{{zNear, 0, 0, 0},
              {0, zNear, 0, 0},
              {0, 0, zNear + zFar, (-1) * zFar * zNear},
              {0, 0, 1, 0}};

  float t = zNear * tan(eye_fov / (2 * 180) * MY_PI);
  float b = -1 * t;
  float r = t * aspect_ratio;
  float l = -1 * r;
  float n = zNear;
  float f = zFar;

  Eigen::Matrix4f m1{{2 / (r - l), 0, 0, 0},
                     {0, 2 / (t - b), 0, 0},
                     {0, 0, 2 / (n - f), 0},
                     {0, 0, 0, 1}};
  m1 = m1.cast<float>();

  Matrix4f m2{{1, 0, 0, -1 * (l + r) / 2},
              {0, 1, 0, -1 * (b + t) / 2},
              {0, 0, 1, -1 * (n + f) / 2},
              {0, 0, 0, 1}};
  m2 = m2.cast<float>();
  projection = (m2 * m1 * m0);
  return projection;
}

int main(int argc, const char **argv) {
  float angle = 0;
  bool command_line = false;
  std::string filename = "output.png";

  if (argc >= 3) {
    command_line = true;
    angle = std::stof(argv[2]); // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    } else
      return 0;
  }

  rst::rasterizer r(700, 700);

  Eigen::Vector3f eye_pos = {0, 0, 5};

  std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

  std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

  auto pos_id = r.load_positions(pos);
  auto ind_id = r.load_indices(ind);

  int key = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite(filename, image);

    return 0;
  }

  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);

    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << '\n';

    if (key == 'a') {
      angle += 10;
    } else if (key == 'd') {
      angle -= 10;
    }
  }

  return 0;
}
