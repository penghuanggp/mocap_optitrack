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
#include <mocap_optitrack/marker_publisher.h>

#include <sstream>

namespace mocap_optitrack {

namespace utilities {
geometry_msgs::msg::PointStamped getRosPoint(LabeledMarker const &marker) {
  geometry_msgs::msg::PointStamped pointStampedMsg;
  pointStampedMsg.point.x = marker.x;
  pointStampedMsg.point.y = marker.y;
  pointStampedMsg.point.z = marker.z;
  return pointStampedMsg;
}
} // namespace utilities

MarkerPublisher::MarkerPublisher(rclcpp::Node::SharedPtr &node, int markerId,
                                 std::string const &topicName)
    : markerId(markerId) {
  publisher =
      node->create_publisher<geometry_msgs::msg::PointStamped>(topicName, 100);
}

MarkerPublisher::~MarkerPublisher() {}

void MarkerPublisher::publish(rclcpp::Time const &time,
                              LabeledMarker const &marker,
                              rclcpp::Logger logger) {
  if (marker.occluded) {
    RCLCPP_DEBUG(logger,
                 "Marker %d is occluded but publishing last known position",
                 markerId);
  }

  geometry_msgs::msg::PointStamped msg = utilities::getRosPoint(marker);
  msg.header.stamp = time;
  msg.header.frame_id = "optitrack"; // Default frame, could be configurable

  publisher->publish(msg);
}

MarkerPublishDispatcher::MarkerPublishDispatcher(
    rclcpp::Node::SharedPtr &node, std::string const &topicBaseName)
    : node(node), topicBaseName(topicBaseName) {}

void MarkerPublishDispatcher::publish(rclcpp::Time const &time,
                                      std::vector<LabeledMarker> const &markers,
                                      rclcpp::Logger logger) {
  // Clear seen markers for this frame
  seenMarkerIds.clear();

  for (auto const &marker : markers) {
    int const id = marker.id;
    seenMarkerIds.insert(id);
    auto const iter = markerPublisherMap.find(id);

    if (iter == markerPublisherMap.end()) {
      std::stringstream topicName;
      topicName << topicBaseName << "/marker_" << id;
      markerPublisherMap[id] =
          MarkerPublisherPtr(new MarkerPublisher(node, id, topicName.str()));
      RCLCPP_INFO(logger, "Created new publisher for marker %d on topic %s", id,
                  topicName.str().c_str());
    }

    markerPublisherMap[id]->publish(time, marker, logger);
  }

  // Remove stale publishers after publishing
  removeStalePublishers(logger);
}

void MarkerPublishDispatcher::removeStalePublishers(rclcpp::Logger logger) {
  for (auto it = markerPublisherMap.begin(); it != markerPublisherMap.end();) {
    int id = it->first;
    if (seenMarkerIds.find(id) == seenMarkerIds.end()) {
      // Marker not seen this frame, increment missed count
      missedFrameCount[id]++;
      if (missedFrameCount[id] >= STALE_GRACE_FRAMES) {
        RCLCPP_INFO(logger, "Removing stale publisher for marker %d (missed %d frames)", 
                    id, missedFrameCount[id]);
        missedFrameCount.erase(id);
        it = markerPublisherMap.erase(it);
        continue;
      }
    } else {
      // Marker seen, reset missed count
      missedFrameCount[id] = 0;
    }
    ++it;
  }
}

} // namespace mocap_optitrack
