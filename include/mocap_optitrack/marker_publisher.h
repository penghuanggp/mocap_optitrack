/*
 * Copyright (c) 2026, Peng Huang
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MOCAP_OPTITRACK_MARKER_PUBLISHER_H
#define MOCAP_OPTITRACK_MARKER_PUBLISHER_H

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <mocap_optitrack/data_model.h>

namespace mocap_optitrack {

/// \brief Encapsulation of a single marker data publisher.
class MarkerPublisher {
public:
  MarkerPublisher(rclcpp::Node::SharedPtr &node, int markerId,
                  std::string const &topicName);
  ~MarkerPublisher();
  void publish(rclcpp::Time const &time, LabeledMarker const &marker,
               rclcpp::Logger logger);

private:
  int markerId;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher;
};

/// \brief Dispatches marker data to the correct publisher.
/// Dynamically creates publishers for new marker IDs and removes stale ones.
class MarkerPublishDispatcher {
  typedef std::shared_ptr<MarkerPublisher> MarkerPublisherPtr;
  typedef std::map<int, MarkerPublisherPtr> MarkerPublisherMap;
  typedef std::map<int, int> MissedFrameCountMap;

  static constexpr int STALE_GRACE_FRAMES =
      5; // Frames before removing stale publisher

  MarkerPublisherMap markerPublisherMap;
  MissedFrameCountMap missedFrameCount;
  std::set<int> seenMarkerIds; // Track markers seen in current frame

public:
  MarkerPublishDispatcher(rclcpp::Node::SharedPtr &node,
                          std::string const &topicBaseName);
  void publish(rclcpp::Time const &time,
               std::vector<LabeledMarker> const &markers,
               rclcpp::Logger logger);
  /// \brief Remove publishers for markers not seen for STALE_GRACE_FRAMES
  void removeStalePublishers(rclcpp::Logger logger);

private:
  rclcpp::Node::SharedPtr node;
  std::string topicBaseName;
};

} // namespace mocap_optitrack

#endif // MOCAP_OPTITRACK_MARKER_PUBLISHER_H
