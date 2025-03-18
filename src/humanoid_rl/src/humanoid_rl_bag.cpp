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
        // vel_frames_.clear();
        // tau_frames_.clear();
        duration_ = 0.0;

        rosbag::Bag bag;
        bag.open(bag_file_, rosbag::bagmode::Read);

        // create view for topic
        std::vector<std::string> topics = { topic_name_ };
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        // get bag start and end time
        ros::Time start_time;
        ros::Time end_time;

        // check if view has messages
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
                    // LOGFMTD("frame[%s] = %f", joint_state_msg->name[i].c_str(), joint_state_msg->position[i]);
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

        if(!pos_frames_.empty() && playback_rate_ > 0.0)
        {

            int transition_frames = 200; // playback_rate_ seconds * 100Hz

            // caculate how many frames we should have for smooth looping
            int total_frames = static_cast<int>(pos_frames_.size());
            int target_frames = static_cast<int>(playback_rate_);

            // find the number of frames needed to make total divisible by playback rate

            int remainder = total_frames % target_frames;
            int alignment_frames = remainder == 0 ? 0 : target_frames - remainder;
            int add_alignment_frames = total_frames + alignment_frames;

            // add enough frames to make total divisible by playback rate
            LOGFMTI("Adding %d frames (%d transition frames + %d alignment frames) to smoothly return to zero",
                    add_alignment_frames, transition_frames, alignment_frames);


            if(alignment_frames > 0)
            {
                for(int i = 0; i < alignment_frames; ++i)
                {
                    pos_frames_.push_back(pos_frames_.back());
                }
            }

            // get last frame data
            const auto &start_frame = pos_frames_.back();
            LOGFMTW("After alignment, pos_frames_.size(): %zu, add_alignment_frames: %d", pos_frames_.size(),
                    add_alignment_frames);

            // create and add interpolated frames
            for(int i = 1; i <= transition_frames; ++i)
            {
                double factor = static_cast<double>(i) / static_cast<double>(transition_frames);

                // interpolate between last frame and zero frame
                std::map<std::string, double> interp_frame;

                for(const auto &joint : start_frame)
                {
                    std::string joint_name = joint.first;
                    double startValue = joint.second;
                    double endValue = 0.0; // must be 0.0

                    // linear interpolation
                    double interpolatedValue = startValue * (1.0 - factor) + endValue * factor;
                    interp_frame[joint_name] = interpolatedValue;
                    LOGFMTD("Interpolated frame %d: %s = %f, startValue: %f, endValue: %f, factor: %f", i,
                            joint_name.c_str(), interpolatedValue, startValue, endValue, factor);
                }

                // add interpolated frame to pos_frames_
                pos_frames_.push_back(interp_frame);
                LOGFMTD("Interpolated frame %d: %zu", i, pos_frames_.size());
            }
        }


        if(!ResampleFrames())
        {
            LOGFMTD("Failed to resample frames!");
            return false;
        }

        return true;
    } catch(const std::exception &e)
    {
        LOGFMTD("Load bag file %s failed: %s", bag_file_.c_str(), e.what());
        return false;
    }
}

bool HumanoidRLBag::ResampleFrames()
{
    if(pos_frames_.empty() || playback_rate_ <= 0.0)
    {
        LOGFMTD("Cannot resample frames: bag not loaded or playback rate is not set");
        return false;
    }


    ori_pos_frames_.clear();
    ori_pos_frames_ = pos_frames_;
    pos_frames_.clear();


    // every playback_rate_ frames, add one frame to ori_pos_frames_
    for(size_t i = 0; i < ori_pos_frames_.size(); i += playback_rate_)
    {
        pos_frames_.push_back(ori_pos_frames_[i]);
    }

    LOGFMTI("Resampled %zu frames from %zu frames with playback rate %f", pos_frames_.size(), ori_pos_frames_.size(),
            playback_rate_);

    return true;
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
    ori_pos_frames_.clear();
    // vel_frames_.clear(); // TODO: add vel_frames_
    // tau_frames_.clear(); // TODO: add tau_frames_
    duration_ = 0.0;
}