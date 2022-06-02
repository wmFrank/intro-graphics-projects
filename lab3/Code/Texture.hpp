//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u00,u01,u10,u11,v00,v01,v10,v11;
        u00 = std::floor(u_img);
        u01 = std::floor(u_img);
        u10 = std::ceil(u_img);
        u11 = std::ceil(u_img);
        v00 = std::floor(v_img);
        v01 = std::floor(v_img);
        v10 = std::ceil(v_img);
        v11 = std::ceil(v_img);
        auto u0 = image_data.at<cv::Vec3b>(v00, u00) + (u_img - u00) * (image_data.at<cv::Vec3b>(v10, u10) - image_data.at<cv::Vec3b>(v00, u00));
        auto u1 = image_data.at<cv::Vec3b>(v01, u01) + (u_img - u01) * (image_data.at<cv::Vec3b>(v11, u11) - image_data.at<cv::Vec3b>(v01, u01));
        auto color = u0 + (v_img - v00) * (u1 - u0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
