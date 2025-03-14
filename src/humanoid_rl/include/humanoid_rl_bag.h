#ifndef HUMANOID_RL_BAG_H
#define HUMANOID_RL_BAG_H

#include <map>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>


class HumanoidRLBag {
public:
    HumanoidRLBag(const std::string &bag_file, const std::string &topic_name, double playback_rate = 1.0);
    ~HumanoidRLBag();

    bool LoadBag();
    const std::map<std::string, double> &GetFrameJointStates(size_t frame_index) const;

    const std::vector<std::map<std::string, double> > &GetPosFrames() const;
    size_t GetFrameNum() const;
    bool IsEmpty() const;
    void Clear();
    double GetPlaybackRate() const;
    double GetDuration() const;

private:
    std::string bag_file_;
    std::string topic_name_;
    std::vector<std::map<std::string, double> > pos_frames_;
    std::vector<std::map<std::string, double> > vel_frames_;
    std::vector<std::map<std::string, double> > tau_frames_;
    double duration_;
    double playback_rate_;
};
#endif // HUMANOID_RL_BAG_H