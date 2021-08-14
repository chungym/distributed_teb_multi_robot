/*********************************************************************
*
* License
*
* Author: Yiu Ming Chung
* 
*********************************************************************/
#ifndef ROBOT_SCAN_MATCHER_BASE_H_
#define ROBOT_SCAN_MATCHER_BASE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <tf2_ros/transform_listener.h>

namespace robot_scan_matcher_base 
{
  class BaseMatcher
  {
    public:
      virtual void initialize(ros::NodeHandle nh, tf2_ros::Buffer* tf, std::string global_frame, std::string local_frame,
                      std::string laserScan_topic, double costmap_SizeInMetersX, double costmap_SizeInMetersY,
                      std::string obst_topic) = 0;

      virtual ~BaseMatcher(){stopWorker();}


     /**
     * @brief This method performs the actual work
     */
    virtual void compute() = 0;


     /**
      * @brief Instantiate a worker that repeatedly carry out robot-scan matching.
      * The worker is implemented as a timer event that is invoked at a specific \c rate.
      * By specifying the argument \c spin_thread the timer event is invoked in a separate
      * thread and callback queue or otherwise it is called from the global callback queue (of the
      * node in which the plugin is used).
      * @param rate The rate that specifies how often the matching should be done.
      * @param spin_thread if \c true,the timer is invoked in a separate thread, otherwise in the default callback queue)
     */
    void startWorker(ros::Rate rate, bool spin_thread = false)
    {
      //setup
      

      //
      
      if (spin_thread_)
      {
        {
          boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
          need_to_terminate_ = true;
        }
        spin_thread_->join();
        delete spin_thread_;
      }
      
      if (spin_thread)
      {
        ROS_DEBUG_NAMED("robot_scan_matcher", "Spinning up a thread for the robot_scan_matcher plugin");
        need_to_terminate_ = false;
        spin_thread_ = new boost::thread(boost::bind(&BaseMatcher::spinThread, this));
        nh_.setCallbackQueue(&callback_queue_);
      }
      else
      {
        spin_thread_ = NULL;
        nh_.setCallbackQueue(ros::getGlobalCallbackQueue());
      }
      
      worker_timer_ = nh_.createTimer(rate, &BaseMatcher::workerCallback, this);
    }
    
    /**
     * @brief Stop the worker
     */
    void stopWorker()
    {
      worker_timer_.stop();
      if (spin_thread_)
      {
        {
          boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
          need_to_terminate_ = true;
        }
        spin_thread_->join();
        delete spin_thread_;
      }
    }

protected:
  
    /**
     * @brief Protected constructor that should be called by subclasses
     */
    BaseMatcher() : nh_("~robot_scan_matcher"), spin_thread_(NULL), need_to_terminate_(false) {}
    
    /**
     * @brief Blocking method that checks for new timer events (active if startWorker() is called with spin_thread enabled) 
     */
    void spinThread()
    {
      while (nh_.ok())
      {
        {
          boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
          if (need_to_terminate_)
            break;
        }
        callback_queue_.callAvailable(ros::WallDuration(0.1f));
      }
    }
    
    /**
     * @brief The callback of the worker that performs the actual work
     */
    void workerCallback(const ros::TimerEvent&)
    {

      compute();
    }


protected:
  ros::Timer worker_timer_;


private:

  ros::NodeHandle nh_;
  boost::thread* spin_thread_;
  ros::CallbackQueue callback_queue_;
  boost::mutex terminate_mutex_;
  bool need_to_terminate_;

  };
};
#endif
