#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "lane_boundary_transformer/path_processing.hpp"

namespace {

lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth MakeLanePair(
    const std::vector<double> &xs, const std::vector<double> &centers, double half_width = 1.5) {
  lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth msg;
  for (std::size_t i = 0; i < xs.size(); ++i) {
    lane_parameter_msg::msg::LaneMarkingProjected left;
    lane_parameter_msg::msg::LaneMarkingProjected right;
    left.x = xs[i];
    right.x = xs[i];
    left.y = centers[i] + half_width;
    right.y = centers[i] - half_width;
    msg.markings_left.push_back(left);
    msg.markings_right.push_back(right);
  }
  return msg;
}

double MeanLateral(const std::vector<geometry_msgs::msg::PoseStamped> &path) {
  double sum = 0.0;
  for (const auto &pose : path) {
    sum += pose.pose.position.y;
  }
  return path.empty() ? 0.0 : sum / static_cast<double>(path.size());
}

}  // namespace

TEST(PathProcessing, StableInputProducesCenteredPath) {
  lane_boundary_transformer::PathProcessingConfig config;
  config.enable_interpolation = false;
  lane_boundary_transformer::LanePathProcessor processor(config);
  lane_boundary_transformer::PathProcessingMetrics metrics;

  const auto input = MakeLanePair({0.0, 1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 0.0});
  const auto output = processor.Process(input, &metrics);

  ASSERT_EQ(output.size(), 4u);
  EXPECT_NEAR(output.front().pose.position.y, 0.0, 1e-6);
  EXPECT_FALSE(metrics.used_history);
  EXPECT_FALSE(metrics.jump_suppressed);
}

TEST(PathProcessing, SingleFrameJumpIsSuppressed) {
  lane_boundary_transformer::PathProcessingConfig config;
  config.enable_interpolation = false;
  config.jump_distance_threshold = 0.5;
  config.jump_blend_alpha = 0.2;
  lane_boundary_transformer::LanePathProcessor processor(config);
  lane_boundary_transformer::PathProcessingMetrics metrics;

  auto baseline = MakeLanePair({0.0, 1.0, 2.0}, {0.0, 0.0, 0.0});
  auto jump = MakeLanePair({0.0, 1.0, 2.0}, {3.0, 3.0, 3.0});

  const auto first = processor.Process(baseline, &metrics);
  const auto second = processor.Process(jump, &metrics);

  ASSERT_EQ(first.size(), second.size());
  EXPECT_TRUE(metrics.jump_suppressed);
  EXPECT_LT(MeanLateral(second), 2.0);
}

TEST(PathProcessing, ShortMissingFramesHoldLastValidPath) {
  lane_boundary_transformer::PathProcessingConfig config;
  config.enable_interpolation = false;
  config.max_missing_frames = 2;
  lane_boundary_transformer::LanePathProcessor processor(config);
  lane_boundary_transformer::PathProcessingMetrics metrics;

  auto valid = MakeLanePair({0.0, 1.0, 2.0}, {0.2, 0.2, 0.2});
  lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth missing;

  const auto first = processor.Process(valid, &metrics);
  const auto second = processor.Process(missing, &metrics);

  EXPECT_FALSE(first.empty());
  EXPECT_EQ(first.size(), second.size());
  EXPECT_TRUE(metrics.used_history);
  EXPECT_EQ(metrics.missing_frames, 1);
}

TEST(PathProcessing, LongMissingFramesReturnEmptyPath) {
  lane_boundary_transformer::PathProcessingConfig config;
  config.enable_interpolation = false;
  config.max_missing_frames = 1;
  lane_boundary_transformer::LanePathProcessor processor(config);
  lane_boundary_transformer::PathProcessingMetrics metrics;

  auto valid = MakeLanePair({0.0, 1.0, 2.0}, {0.2, 0.2, 0.2});
  lane_parameter_msg::msg::LaneMarkingProjectedArrayBoth missing;

  processor.Process(valid, &metrics);
  processor.Process(missing, &metrics);
  const auto third = processor.Process(missing, &metrics);

  EXPECT_TRUE(third.empty());
  EXPECT_EQ(metrics.status, "missing_timeout");
}

TEST(PathProcessing, OutputDelayReturnsOlderPathAfterWarmup) {
  lane_boundary_transformer::PathProcessingConfig config;
  config.enable_interpolation = false;
  config.output_delay_frames = 2;
  lane_boundary_transformer::LanePathProcessor processor(config);
  lane_boundary_transformer::PathProcessingMetrics metrics;

  auto frame0 = MakeLanePair({0.0, 1.0}, {0.0, 0.0});
  auto frame1 = MakeLanePair({0.0, 1.0}, {1.0, 1.0});
  auto frame2 = MakeLanePair({0.0, 1.0}, {2.0, 2.0});

  const auto out0 = processor.Process(frame0, &metrics);
  const auto out1 = processor.Process(frame1, &metrics);
  const auto out2 = processor.Process(frame2, &metrics);

  EXPECT_NEAR(MeanLateral(out0), 0.0, 0.1);
  EXPECT_NEAR(MeanLateral(out1), 0.7, 0.4);
  EXPECT_NEAR(MeanLateral(out2), MeanLateral(out0), 0.2);
  EXPECT_TRUE(metrics.output_delayed);
}
