#include "humanoid_rl_bag.h"
#include "log4z.h"

HumanoidRLBag::HumanoidRLBag(const std::string &bag_file, const std::string &topic_name, double playback_rate)
    : bag_file_(bag_file), topic_name_(topic_name), playback_rate_(playback_rate)
{
    try
    {
        if(!bag_file_.empty() && !topic_name_.empty())
        {
            LoadBag();
            LOGFMTD("Load bag file %s and topic %s success", bag_file_.c_str(), topic_name_.c_str());
        } else
        {
            LOGFMTD("Bag file or topic name is empty");
        }
    } catch(const std::exception &e)
    {
        LOGFMTD("Load bag file %s and topic %s failed: %s", bag_file_.c_str(), topic_name_.c_str(), e.what());
    }
}

HumanoidRLBag::~HumanoidRLBag() { LOGD("HumanoidRLBag object has been destroyed!"); }

bool HumanoidRLBag::LoadBag()
{
    try
    {
        pos_frames_.clear();
        vel_frames_.clear();
        tau_frames_.clear();
        duration_ = 0.0;

        rosbag::Bag bag;
        bag.open(bag_file_, rosbag::bagmode::Read);

        // create view for topic
        std::vector<std::string> topics = { topic_name_ };
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // 获取起始和结束时间
        ros::Time start_time;
        ros::Time end_time;

        // 检查 view 是否有消息
        if(view.size() > 0)
        {
            start_time = view.getBeginTime();
            end_time = view.getEndTime();
            duration_ = (end_time - start_time).toSec();
        }

        // iterate over messages
        for(const rosbag::MessageInstance &msg : view)
        {
            sensor_msgs::JointStateConstPtr joint_state_msg = msg.instantiate<sensor_msgs::JointState>();
            if(joint_state_msg)
            {
                // create frame
                std::map<std::string, double> frame;
                for(size_t i = 0; i < joint_state_msg->name.size(); ++i)
                {
                    frame[joint_state_msg->name[i]] = joint_state_msg->position[i];
                }
                pos_frames_.push_back(frame);
            } else
            {
                LOGFMTD("Message is not a JointState message");
                continue;
            }
        }
        bag.close();
        LOGFMTI("Loaded %zu frames from bag file %s, duration: %.2f seconds", pos_frames_.size(), bag_file_.c_str(),
                duration_);

        return true;
    } catch(const std::exception &e)
    {
        LOGFMTD("Load bag file %s failed: %s", bag_file_.c_str(), e.what());
        return false;
    }
}

const std::map<std::string, double> &HumanoidRLBag::GetFrameJointStates(size_t frame_index) const
{
    static std::map<std::string, double> empty_frame;
    if(frame_index >= pos_frames_.size())
    {
        LOGFMTD("Frame index %zu out of range", frame_index);
        return empty_frame;
    }
    return pos_frames_[frame_index];
}

const std::vector<std::map<std::string, double> > &HumanoidRLBag::GetPosFrames() const { return pos_frames_; }

double HumanoidRLBag::GetPlaybackRate() const { return playback_rate_; }

double HumanoidRLBag::GetDuration() const { return duration_; }

size_t HumanoidRLBag::GetFrameNum() const { return pos_frames_.size(); }

bool HumanoidRLBag::IsEmpty() const { return pos_frames_.empty(); }

void HumanoidRLBag::Clear()
{
    pos_frames_.clear();
    vel_frames_.clear();
    tau_frames_.clear();
    duration_ = 0.0;
}