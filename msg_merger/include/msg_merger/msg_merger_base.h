/*
* BSD 3-Clause License
*
* Copyright (c) 2021, chungym
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MSG_MERGER_BASE_H_
#define MSG_MERGER_BASE_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

namespace msg_merger_base
{
  class BaseMerger
  {
    public:
      virtual void initialize(ros::NodeHandle nh) = 0;

      virtual ~BaseMerger(){stopWorker();}


     /**
     * @brief This method performs the actual work
     */
    virtual void compute() = 0;
    virtual void compute2() = 0;

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
        ROS_DEBUG_NAMED("msg_merger", "Spinning up a thread for the msg_merger plugin");
        need_to_terminate_ = false;
        spin_thread_ = new boost::thread(boost::bind(&BaseMerger::spinThread, this));
        nh_.setCallbackQueue(&callback_queue_);
      }
      else
      {
        spin_thread_ = NULL;
        nh_.setCallbackQueue(ros::getGlobalCallbackQueue());
      }
      
      worker_timer_ = nh_.createTimer(1, &BaseMerger::workerCallback, this);
      worker_timer2_ = nh_.createTimer(rate, &BaseMerger::workerCallback2, this);
    }
    
    /**
     * @brief Stop the worker
     */
    void stopWorker()
    {
      worker_timer_.stop();
      worker_timer2_.stop();
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
    BaseMerger() : nh_("~msg_merger"), spin_thread_(NULL), need_to_terminate_(false) {}
    
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

    /**
     * @brief The callback of the worker that performs the actual work
     */
    void workerCallback2(const ros::TimerEvent&)
    {

      compute2();
    }


protected:
  ros::Timer worker_timer_;
  ros::Timer worker_timer2_;


private:

  ros::NodeHandle nh_;
  boost::thread* spin_thread_;
  ros::CallbackQueue callback_queue_;
  boost::mutex terminate_mutex_;
  bool need_to_terminate_;

  };
};
#endif
