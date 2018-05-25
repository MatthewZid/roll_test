/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#include <roll_test/rviz_rosbag_player.h>

namespace rviz_rosbag
{

ros::AdvertiseOptions createAdvertiseOptions(const rosbag::ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    ros::M_string::const_iterator header_iter = c->header->find("latching");
    opts.latch = (header_iter != c->header->end() and header_iter->second == "1");
    return opts;
}


ros::AdvertiseOptions createAdvertiseOptions(rosbag::MessageInstance const& m, uint32_t queue_size, const std::string& prefix) {
    return ros::AdvertiseOptions(prefix + m.getTopic(), queue_size, m.getMD5Sum(), m.getDataType(), m.getMessageDefinition());
}

// PlayerOptions

PlayerOptions::PlayerOptions() :
    prefix(""),
    quiet(false),
    start_paused(false),
    at_once(false),
    bag_time(false),
    bag_time_frequency(0.0),
    time_scale(1.0),
    queue_size(0),
    advertise_sleep(0.2),
    try_future(false),
    has_time(false),
    loop(false),
    time(0.0f),
    has_duration(false),
    duration(0.0f),
    keep_alive(false),
    wait_for_subscribers(false),
    rate_control_topic(""),
    rate_control_max_delay(1.0f),
    skip_empty(ros::DURATION_MAX)
{
}

void PlayerOptions::check() {
    if (bags.size() == 0)
        throw ros::Exception("You must specify at least one bag file to play from");
    if (has_duration and duration <= 0.0)
        throw ros::Exception("Invalid duration, must be > 0.0");
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options),
    paused_(false),
    // If we were given a list of topics to pause on, then go into that mode
    // by default (it can be toggled later via 't' from the keyboard).
    pause_for_topics_(options_.pause_topics.size() > 0),
    pause_change_requested_(false),
    requested_pause_state_(false)
{
  ros::NodeHandle private_node_handle("~");
  pause_service_ = private_node_handle.advertiseService("pause_playback", &Player::pauseCallback, this);
}

Player::~Player() {
    for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        bag->close();
}

void Player::publish() {
    options_.check();

    // Open all the bag files
    for(std::string const& filename : options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());

        try
        {
            boost::shared_ptr<rosbag::Bag> bag(boost::make_shared<rosbag::Bag>());
            bag->open(filename, rosbag::bagmode::Read);
            bags_.push_back(bag);
        }
        catch (rosbag::BagUnindexedException ex) {
            std::cerr << "Bag file " << filename << " is unindexed.  Run rosbag reindex." << std::endl;
            return;
        }
    }

    if (!node_handle_.ok())
      return;

    if (!options_.prefix.empty())
    {
      ROS_INFO_STREAM("Using prefix '" << options_.prefix << "'' for topics ");
    }

    if (!options_.quiet)
      puts("");
    
    // Publish all messages in the bags
    rosbag::View full_view;
    for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        full_view.addQuery(*bag);

    ros::Time initial_time = full_view.getBeginTime();

    initial_time += ros::Duration(options_.time);

    ros::Time finish_time = ros::TIME_MAX;
    if (options_.has_duration)
    {
      finish_time = initial_time + ros::Duration(options_.duration);
    }

    rosbag::View view;
    rosbag::TopicQuery topics(options_.topics);

    if (options_.topics.empty())
    {
      for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        view.addQuery(*bag, initial_time, finish_time);
    } else {
      for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        view.addQuery(*bag, topics, initial_time, finish_time);
    }

    if (view.size() == 0)
    {
      std::cerr << "No messages to play on specified topics.  Exiting." << std::endl;
      //ros::shutdown();
      return;
    }

    // Advertise all of our messages
    for(const rosbag::ConnectionInfo* c : view.getConnections())
    {
        ros::M_string::const_iterator header_iter = c->header->find("callerid");
        std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

        std::string callerid_topic = callerid + c->topic;

        std::map<std::string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
        if (pub_iter == publishers_.end()) {

            ros::AdvertiseOptions opts = createAdvertiseOptions(c, options_.queue_size, options_.prefix);

            ros::Publisher pub = node_handle_.advertise(opts);
            publishers_.insert(publishers_.begin(), std::pair<std::string, ros::Publisher>(callerid_topic, pub));

            pub_iter = publishers_.find(callerid_topic);
        }
    }

    if (options_.rate_control_topic != "")
    {
        std::cout << "Creating rate control topic subscriber..." << std::flush;

        boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
        ros::SubscribeOptions ops;
        ops.topic = options_.rate_control_topic;
        ops.queue_size = 10;
        ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
        ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
        ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
            const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                boost::bind(&Player::updateRateTopicTime, this, _1));

        rate_control_sub_ = node_handle_.subscribe(ops);

        std::cout << " done." << std::endl;
    }


    std::cout << "Waiting " << options_.advertise_sleep.toSec() << " seconds after advertising topics..." << std::flush;
    options_.advertise_sleep.sleep();
    std::cout << " done." << std::endl;

    std::cout << std::endl << "Hit space to toggle paused, or 's' to step." << std::endl;

    paused_ = options_.start_paused;

    if (options_.wait_for_subscribers)
    {
        waitForSubscribers();
    }

    while (true) {
        // Set up our time_translator and publishers

        time_translator_.setTimeScale(options_.time_scale);

        start_time_ = view.begin()->getTime();
        time_translator_.setRealStartTime(start_time_);
        bag_length_ = view.getEndTime() - view.getBeginTime();

        // Set the last rate control to now, so the program doesn't start delayed.
        last_rate_control_ = start_time_;

        time_publisher_.setTime(start_time_);

        ros::WallTime now_wt = ros::WallTime::now();
        time_translator_.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));


        time_publisher_.setTimeScale(options_.time_scale);
        if (options_.bag_time)
            time_publisher_.setPublishFrequency(options_.bag_time_frequency);
        else
            time_publisher_.setPublishFrequency(-1.0);

        paused_time_ = now_wt;

        // Call do-publish for each message
        for(rosbag::MessageInstance m : view) {
            if (!node_handle_.ok())
                break;

            doPublish(m);
        }

        if (options_.keep_alive)
            while (node_handle_.ok())
                doKeepAlive();

        if (!node_handle_.ok()) {
            std::cout << std::endl;
            break;
        }
        if (!options_.loop) {
            std::cout << std::endl << "Done." << std::endl;
            break;
        }
    }

    //ros::shutdown();
}

void Player::updateRateTopicTime(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event)
{
}

void Player::printTime()
{
}

bool Player::pauseCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
}

void Player::processPause(const bool paused, ros::WallTime &horizon)
{
}

void Player::waitForSubscribers() const
{
}

void Player::doPublish(rosbag::MessageInstance const& m)
{
}

void Player::doKeepAlive()
{
}

TimePublisher::TimePublisher() : time_scale_(1.0)
{
  setPublishFrequency(-1.0);
  time_pub_ = node_handle_.advertise<rosgraph_msgs::Clock>("clock",1);
}

void TimePublisher::setPublishFrequency(double publish_frequency)
{
}

void TimePublisher::setTimeScale(double time_scale)
{
}

void TimePublisher::setHorizon(const ros::Time& horizon)
{
}

void TimePublisher::setWCHorizon(const ros::WallTime& horizon)
{
}

void TimePublisher::setTime(const ros::Time& time)
{
}

ros::Time const& TimePublisher::getTime() const
{
}

void TimePublisher::runClock(const ros::WallDuration& duration)
{
}

void TimePublisher::stepClock()
{
}

void TimePublisher::runStalledClock(const ros::WallDuration& duration)
{
}

bool TimePublisher::horizonReached()
{
}

}