#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    //if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 8)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.0001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

void anti_aliasing(cv::Mat &window, cv::Point2f &point)
{
    int xmin = std::floor(point.x);
    int xmax = std::ceil(point.x);
    int ymin = std::floor(point.y);
    int ymax = std::ceil(point.y);
    double dis0 = std::sqrt(std::pow(point.x - xmin, 2) + std::pow(point.y - ymin, 2));
    double dis1 = std::sqrt(std::pow(point.x - xmin, 2) + std::pow(point.y - ymax, 2));
    double dis2 = std::sqrt(std::pow(point.x - xmax, 2) + std::pow(point.y - ymin, 2));
    double dis3 = std::sqrt(std::pow(point.x - xmax, 2) + std::pow(point.y - ymax, 2));
    double dis = std::min(std::min(std::min(dis0,dis1),dis2),dis3);
    window.at<cv::Vec3b>(ymin, xmin)[1] = std::round(std::max(255 * (dis / dis0),window.at<cv::Vec3b>(ymin, xmin)[1] * 1.0));
    window.at<cv::Vec3b>(ymax, xmin)[1] = std::round(std::max(255 * (dis / dis1),window.at<cv::Vec3b>(ymax, xmin)[1] * 1.0));
    window.at<cv::Vec3b>(ymin, xmax)[1] = std::round(std::max(255 * (dis / dis2),window.at<cv::Vec3b>(ymin, xmax)[1] * 1.0));
    window.at<cv::Vec3b>(ymax, xmax)[1] = std::round(std::max(255 * (dis / dis3),window.at<cv::Vec3b>(ymax, xmax)[1] * 1.0));
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size() == 1)
    {
        return control_points[0];
    }
    else
    {
        std::vector<cv::Point2f> tmp;
        for(int i = 0; i < control_points.size() - 1; i++)
        {
            auto &p_0 = control_points[i];
            auto &p_1 = control_points[i + 1];
            auto point = (1 - t) * p_0 + t * p_1;
            tmp.push_back(point);
        }
        return recursive_bezier(tmp,t);
    }

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(float t = 0.0; t <= 1.0; t += 0.0001)
    {
        auto point = recursive_bezier(control_points,t);
        anti_aliasing(window,point);
        //window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        //if (control_points.size() == 8) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
