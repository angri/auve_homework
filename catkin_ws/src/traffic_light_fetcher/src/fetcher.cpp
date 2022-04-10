#include "ros/ros.h"

#include "fetcher.h"

#include "cuda_utils.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <filesystem>
#include <memory>
#include <vector>
#include <fstream>


namespace fs = ::std::filesystem;


namespace traffic_light_fetcher {

typedef struct Detection
{
  int x, y, w, h;
  float prob;
} Detection;

bool readLabelFile(const std::string& filepath, std::vector<std::string>& labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    ROS_ERROR("Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels.push_back(label);
  }
  return true;
}

bool getTlrIdFromLabel(const std::vector<std::string>& labels, int& tlr_id)
{
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels.at(i) == "traffic_light") {
      tlr_id = i;
      return true;
    }
  }
  return false;
}

bool cvMat2CnnInput(
    const cv::Mat& in_img, std::vector<float>& data, int width_, int height_, int channel_)
{
  const std::vector<float> mean_{0.5, 0.5, 0.5};
  const std::vector<float> std_{0.5, 0.5, 0.5};

  cv::Mat resized;
  cv::resize(in_img, resized, cv::Size(width_, height_));

  cv::Mat pixels;
  resized.convertTo(pixels, CV_32FC3, 1.0 / 255, 0);
  std::vector<float> img;
  if (pixels.isContinuous()) {
    img.assign(
      reinterpret_cast<const float *>(pixels.datastart),
      reinterpret_cast<const float *>(pixels.dataend));
  } else {
    return false;
  }

  for (int c = 0; c < channel_; ++c) {
    for (int j = 0, hw = width_ * height_; j < hw; ++j) {
      data[c * hw + j] = (img[channel_ * j + 2 - c] - mean_[c]) / std_[c];
    }
  }
  return true;
}

bool rosMsg2CvMat(const sensor_msgs::ImageConstPtr& image_msg, cv::Mat& image)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "rgb8");
    image = cv_image->image;
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR(
      "Failed to convert sensor_msgs::msg::Image to cv::Mat \n%s", e.what());
    return false;
  }

  return true;
}

bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size)
{
  const int width = static_cast<int>(size.width);
  const int height = static_cast<int>(size.height);
  {
    const int x_min = 0, x_max = width - 2;
    const int y_min = 0, y_max = height - 2;
    lt.x = std::min(std::max(lt.x, x_min), x_max);
    lt.y = std::min(std::max(lt.y, y_min), y_max);
  }
  {
    const int x_min = lt.x + 1, x_max = width - 1;
    const int y_min = lt.y + 1, y_max = height - 1;
    rb.x = std::min(std::max(rb.x, x_min), x_max);
    rb.y = std::min(std::max(rb.y, y_min), y_max);
  }

  return true;
}

Detection cnnOutput2BoxDetection(
    const float* scores, const float* boxes,
    const int tlr_id, const int class_num_, const int detection_per_class_,
    const cv::Mat& in_img)
{
  if (tlr_id > class_num_ - 1) {
    return {0, 0, 0, 0, 0};
  }
  std::vector<float> tlr_scores;
  Detection det;
  for (int j = 0; j < detection_per_class_; ++j) {
    tlr_scores.push_back(scores[tlr_id + j * class_num_]);
  }
  std::vector<float>::iterator iter = std::max_element(tlr_scores.begin(), tlr_scores.end());
  size_t index = std::distance(tlr_scores.begin(), iter);
  size_t box_index = index * 4;
  cv::Point lt, rb;
  lt.x = boxes[box_index] * in_img.cols;
  lt.y = boxes[box_index + 1] * in_img.rows;
  rb.x = boxes[box_index + 2] * in_img.cols;
  rb.y = boxes[box_index + 3] * in_img.rows;
  fitInFrame(lt, rb, cv::Size(in_img.cols, in_img.rows));
  det.x = lt.x;
  det.y = lt.y;
  det.w = rb.x - lt.x;
  det.h = rb.y - lt.y;

  det.prob = tlr_scores[index];
  return det;
}

FetcherActor::FetcherActor(
    const std::string& onnx_file,
    const std::string& label_file,
    std::function<void(const sensor_msgs::ImageConstPtr&)> vis_cb,
    std::function<void(const std_msgs::BoolConstPtr&)> detected_cb,
    std::function<void(const geometry_msgs::Vector3ConstPtr&)> size_cb)
  : vis_cb_(vis_cb)
  , detected_cb_(detected_cb)
  , size_cb_(size_cb)
{
  std::vector<std::string> labels;

  if (readLabelFile(label_file, labels)) {
    if (!getTlrIdFromLabel(labels, tlr_id_)) {
      ROS_ERROR("Could not find tlr id");
      return;
    }
  }

  fs::path engine_path{onnx_file};
  engine_path.replace_extension("engine");

  const int max_batch_size = 8;
  const std::string mode = "FP32";
  if (fs::exists(engine_path)) {
    ROS_INFO("Found %s", engine_path.string().c_str());
    net_ptr_ = std::make_unique<ssd::Net>(engine_path, false);
    if (max_batch_size != net_ptr_->getMaxBatchSize()) {
      ROS_INFO(
        "Required max batch size %d does not correspond to Profile max batch size %d. Rebuild "
        "engine from onnx",
        max_batch_size, net_ptr_->getMaxBatchSize());
      net_ptr_ = std::make_unique<ssd::Net>(onnx_file, mode, max_batch_size);
      net_ptr_->save(engine_path);
    }
  } else {
    ROS_INFO(
      "Could not find %s, try making TensorRT engine from onnx",
      engine_path.string().c_str());
    net_ptr_ = std::make_unique<ssd::Net>(onnx_file, mode, max_batch_size);
    net_ptr_->save(engine_path);
  }

  channel_ = net_ptr_->getInputSize()[0];
  width_ = net_ptr_->getInputSize()[1];
  height_ = net_ptr_->getInputSize()[2];
  detection_per_class_ = net_ptr_->getOutputScoreSize()[0];
  class_num_ = net_ptr_->getOutputScoreSize()[1];
  ROS_INFO("Model input info: channel=%d, width=%d, height=%d", channel_, width_, height_);
  ROS_INFO(
      "Model output info: detection_per_class=%d, class_num=%d", detection_per_class_, class_num_);

  ready_ = true;
}

void FetcherActor::OnImage(const sensor_msgs::ImageConstPtr& msg) {
  if (!ready_) {
    return;
  }
  ros::Time now = ros::Time::now();
  ros::Duration delta = now - last_incoming_image_;
  if (delta.toSec() < kMinInterval) {
    ROS_INFO_THROTTLE(0.4, "too early to process an image");
    return;
  }
  last_incoming_image_ = now;

  if (msg->width < 2 || msg->height < 2) {
    ROS_INFO_THROTTLE(1.0, "image is too small, ignoring");
    return;
  }
  ROS_INFO("processing an image");

  cv::Mat original_image;
  rosMsg2CvMat(msg, original_image);

  auto data_d = cuda::make_unique<float[]>(channel_ * width_ * height_);
  auto scores_d = cuda::make_unique<float[]>(detection_per_class_ * class_num_);
  auto boxes_d = cuda::make_unique<float[]>(detection_per_class_ * 4);
  std::vector<void *> buffers = {data_d.get(), scores_d.get(), boxes_d.get()};
  std::vector<float> data(channel_ * width_ * height_);
  if (!cvMat2CnnInput(original_image, data, width_, height_, channel_)) {
    ROS_ERROR("Fail to preprocess image");
    return;
  }

  cudaMemcpy(data_d.get(), data.data(), data.size() * sizeof(float), cudaMemcpyHostToDevice);
  try {
    net_ptr_->infer(buffers, /* num_infer =*/ 1);
  } catch (std::exception & e) {
    ROS_ERROR("%s", e.what());
    return;
  }

  auto scores = std::make_unique<float[]>(detection_per_class_ * class_num_);
  auto boxes = std::make_unique<float[]>(detection_per_class_ * 4);
  cudaMemcpy(
      scores.get(), scores_d.get(), sizeof(float) * detection_per_class_ * class_num_,
      cudaMemcpyDeviceToHost);
  cudaMemcpy(
      boxes.get(), boxes_d.get(), sizeof(float) * detection_per_class_ * 4,
      cudaMemcpyDeviceToHost);

  auto detected_msg = boost::make_shared<std_msgs::Bool>();
  detected_msg->data = false;
  boost::shared_ptr<geometry_msgs::Vector3> size_msg = nullptr;

  Detection det = cnnOutput2BoxDetection(
      scores.get(), boxes.get(), tlr_id_, class_num_, detection_per_class_, original_image);
  if (det.prob > kProbThreshold) {
    ROS_INFO("found a traffic light at {%d, %d} with size %dx%d", det.x, det.y, det.w, det.h);
    cv::Rect rect(det.x, det.y, det.w, det.h);
    cv::rectangle(original_image, rect, cv::Scalar(255, 255, 0));
    detected_msg->data = true;
    size_msg = boost::make_shared<geometry_msgs::Vector3>();
    size_msg->x = det.w;
    size_msg->y = det.h;
  } else {
    ROS_INFO("no traffic light found (prob=%f)", det.prob);
  }
  sensor_msgs::ImagePtr vis_msg = cv_bridge::CvImage(
      std_msgs::Header(), "rgb8", original_image).toImageMsg();
  vis_cb_(vis_msg);
  detected_cb_(detected_msg);
  if (size_msg != nullptr) {
    size_cb_(size_msg);
  }
}

}  // namespace traffic_light_fetcher
