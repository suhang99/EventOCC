#include "evlc_screen/decoder.hpp"

#include <omp.h>

#include <bitset>

#include "evlc_common/timer.hpp"
#include "evlc_common/utils.hpp"

namespace evlc::screen {

Decoder::Decoder(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
                 ScreenMarker::Ptr marker, Visualizer::Ptr visualizer)
    : sae_(sae), marker_(marker), visualizer_(visualizer) {
  double thres;
  nh_private.getParam("receiver/decoder/cell_thres", thres);
  cell_thres_ = marker_->canvas_cell_size * marker_->canvas_cell_size * thres;

  nh_private.getParam("receiver/decoder/binary_thres", binary_thres_);

  nh_private.getParam("receiver/decoder/times", decode_times_);
  decode_count_ = 0;

  nh_private.getParam("sender/frequency", frequency_);
  period_ = 1.0 / frequency_;
}

std::string Decoder::decode(const std::array<cv::Point2d, 8>& trackers) {
  return decode(marker_->trackersToLocators(trackers));
}

std::string Decoder::decode(const std::array<cv::Point2d, 4>& locators) {
  // convert trackers to locators for convenience
  std::string data;
  auto bits = extractMarkerBits(locators, period_ / decode_times_);
  int margin = marker_->inner_locator_cells;
  int payload_size = marker_->payload_cells;

  BitMatrix payload = bits.block(margin, margin, payload_size, payload_size);
  bits_.emplace_back(payload);

  ++decode_count_;

  if (decode_count_ % decode_times_ == 0) {
    // Put all bits together
    BitMatrix bits_final = bits_.front();
    for (int i = 1; i < bits_.size(); ++i) {
      bits_final = (bits_final.array() || bits_[i].array()).matrix();
    }
    bits_.clear();

    // Serialize bits into string
    char s = 0;
    auto bit_array = bits_final.array();
    for (int i = 0; i < bit_array.size(); i += 8) {
      s = 0;
      for (int j = 0; j < 8; ++j) {
        s |= (static_cast<bool>(bit_array(i + j)) << (7 - j));
      }
      data += s;
    }
  }
  return data;
}

BitMatrix Decoder::extractMarkerBits(const std::array<cv::Point2d, 4>& locators, double period) {
  // Apply homography transformation to align marker
  cv::Mat image = sae_->convertToGray(period, true);
  cv::Mat H = marker_->getHomography(locators);
  auto canvas_size = marker_->locator_canvas_size;
  cv::Mat warped;
  cv::warpPerspective(image, warped, H, cv::Size(canvas_size, canvas_size));

  // Convert to binary image
  // cv::medianBlur(warped, warped, 5);
  cv::threshold(warped, warped, binary_thres_, 1, 0);

  auto size = marker_->canvas_cell_size;
  auto cells = marker_->inner_cells;
  BitMatrix bits(cells, cells);
  bits.setZero();

  for (auto row = 0; row < cells; ++row) {
    for (auto col = 0; col < cells; ++col) {
      if (cv::sum(warped(cv::Rect(col * size, row * size, size, size)))[0] > cell_thres_) {
        bits(row, col) = true;
      }
    }
  }
  return bits;
}

int Decoder::getMarkerRoll(const BitMatrix& bits) {
  std::vector<int> sum(4, 0);
  for (int i = 0; i < marker_->inner_cells; ++i) {
    sum[0] += bits(bits.rows() - 1, i);  // Bottom row
    sum[1] += bits(i, 0);                // Left column
    sum[2] += bits(0, i);                // Top row
    sum[3] += bits(i, bits.cols() - 1);  // Right column
  }

  double thres = marker_->inner_cells * 0.75;
  int count = std::count_if(sum.begin(), sum.end(), [thres](int x) { return x >= thres; });

  // Cannot identify locator
  if (count != 2) {
    return -1;
  }

  int roll = -1;
  for (int i = 0; i < 4; ++i) {
    if (sum[i] > thres and sum[(i + 1) % 4] > thres) {
      roll = i;
    }
  }
  if (roll < 0) {
    return -1;
  }
  return roll;
}

BitMatrix Decoder::alignMarkerBits(const BitMatrix& bits, int roll) {
  assert(roll <= 3 and roll >= 0);
  if (roll == 1) {
    // Roll bits by 90 degrees counter-clockwise
    return bits.transpose().colwise().reverse();
  } else if (roll == 2) {
    // Roll bits by 180 degrees counter-clockwise
    return bits.colwise().reverse().rowwise().reverse();
  } else if (roll == 3) {
    // Roll bits by 270 degrees counter-clockwise
    return bits.transpose().rowwise().reverse();
  }
  return bits;
}

}  // namespace evlc::screen