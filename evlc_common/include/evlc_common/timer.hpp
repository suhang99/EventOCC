#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace evlc {

class ScopedTimer {
 public:
  ScopedTimer(std::string label = "") : start_(clock_t::now()), label_(label) {}
  ~ScopedTimer() {
    auto now = clock_t::now();
    auto diff = now - start_;
    std::cout << "timer " << label_ << " :"
              << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << "ms"
              << std::endl;
  }

 private:
  using clock_t = std::chrono::high_resolution_clock;

  std::chrono::time_point<clock_t> start_;
  std::string label_;
};

class Profiler {
 public:
  using Ptr = std::shared_ptr<Profiler>;

  Profiler() : root(std::make_unique<Node>("All", nullptr)) {}

  void start() {
    root->start = Clock::now();
  }

  void stop() {
    auto end = Clock::now();
    auto duration = std::chrono::duration_cast<Duration>(end - root->start);
    root->totalDuration += duration;
    root->maxDuration = std::max(root->maxDuration, duration);
    root->runs++;
    root->start = TimePoint();  // Reset start time
  }

  void start(const std::string& name, const std::string& group = "All") {
    // Find or create the node for this snippet, specifying its group.
    Node* node = findOrCreateNode(name, group);
    node->start = Clock::now();
  }

  void stop(const std::string& name) {
    // Find the node by name; it must exist if stop is called correctly.
    Node* node = findNode(name, root.get());
    if (node && node->start != TimePoint()) {
      auto end = Clock::now();
      auto duration = std::chrono::duration_cast<Duration>(end - node->start);
      node->totalDuration += duration;
      node->maxDuration = std::max(node->maxDuration, duration);
      node->runs++;
      node->start = TimePoint();  // Reset start time
    }
  }

  void outputStatistics() const {
    std::cout << std::left << std::setw(30) << "Name" << std::setw(12) << "Total" << std::setw(12)
              << "Mean" << std::setw(12) << "Max" << std::setw(10) << "Run" << std::setw(12)
              << "Usage" << std::endl;
    std::cout << std::string(90, '-') << std::endl;
    outputNodeStatistics(root.get(), 0, root->totalDuration);
    std::cout << std::endl;
  }

  void reset() { root = std::make_unique<Node>("All", nullptr); }


 private:
  struct Node {
    std::string name;
    Node* parent;
    std::chrono::microseconds totalDuration{0};
    std::chrono::microseconds maxDuration{0};
    size_t runs = 0;
    std::map<std::string, std::unique_ptr<Node>> children;
    std::chrono::time_point<std::chrono::steady_clock> start;

    Node(std::string name, Node* parent) : name(std::move(name)), parent(parent) {}
  };

  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Duration = std::chrono::microseconds;
  std::unique_ptr<Node> root;

  Node* findOrCreateNode(const std::string& name, const std::string& group) {
    Node* groupNode = findNode(group, root.get());
    if (!groupNode) {
      // If the group doesn't exist, create it under the root.
      groupNode = new Node(group, root.get());
      root->children[group] = std::unique_ptr<Node>(groupNode);
    }
    auto& child = groupNode->children[name];
    if (!child) {
      count++;
      child = std::make_unique<Node>(name, groupNode);
    }
    return child.get();
  }

  int count = 0;

  Node* findNode(const std::string& name, Node* current) const {
    if (current->name == name) return current;
    for (auto& child : current->children) {
      Node* found = findNode(name, child.second.get());
      if (found) return found;
    }
    return nullptr;
  }

  void outputNodeStatistics(const Node* node, int depth, const Duration& parentDuration) const {
    if (node->runs > 0 || node == root.get()) {
      auto totalDuration =
          std::chrono::duration_cast<std::chrono::microseconds>(node->totalDuration).count();
      auto maxDuration =
          std::chrono::duration_cast<std::chrono::microseconds>(node->maxDuration).count();
      auto avgDuration = node->runs > 0 ? totalDuration / node->runs : 0;
      double percentOfGroup =
          parentDuration.count() > 0 ? 100.0 * totalDuration / parentDuration.count() : 0.0;

      std::cout << std::string(depth * 2, ' ') << std::left << std::setw(30 - depth * 2)
                << node->name;
      if (totalDuration > 1e6) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << totalDuration / 1e6
                  << "  s   ";
      } else if (totalDuration > 1e3) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << totalDuration / 1e3
                  << " ms   ";
      } else {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << totalDuration / 1.0
                  << " us   ";
      }
      if (avgDuration > 1e6) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << avgDuration / 1e6
                  << "  s   ";
      } else if (totalDuration > 1e3) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << avgDuration / 1e3
                  << " ms   ";
      } else {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << avgDuration / 1.0
                  << " us   ";
      }
      if (maxDuration > 1e6) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << maxDuration / 1e6
                  << "  s   ";
      } else if (totalDuration > 1e3) {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << maxDuration / 1e3
                  << " ms   ";
      } else {
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << maxDuration / 1.0
                  << " us   ";
      }

      std::cout << std::setw(10) << node->runs << std::fixed << std::setprecision(2)
                << percentOfGroup << "%" << std::endl;
    }

    for (const auto& child : node->children) {
      // Pass the current node's total duration as the parent's total duration
      // for percentage calculation
      outputNodeStatistics(child.second.get(), depth + 1, node->totalDuration);
    }
  }
};
}  // namespace evlc