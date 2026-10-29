#include <gtest/gtest.h>
#include <atomic>
#include <map>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
class Timers:public tf2_ros::CreateTimerInterface {
public:
  tf2_ros::TimerHandle createTimer(rclcpp::Clock::SharedPtr,const tf2::Duration&,tf2_ros::TimerCallbackType cb)override{
    std::lock_guard<std::mutex>lock(mutex);const auto id=next++;callbacks.emplace(id,std::move(cb));return id;
  }
  void remove(const tf2_ros::TimerHandle&id)override{std::lock_guard<std::mutex>lock(mutex);callbacks.erase(id);}
  void cancel(const tf2_ros::TimerHandle&id)override{remove(id);}
  void reset(const tf2_ros::TimerHandle&)override{}
  size_t size(){std::lock_guard<std::mutex>lock(mutex);return callbacks.size();}
  void fire(){std::map<tf2_ros::TimerHandle,tf2_ros::TimerCallbackType> copy;{std::lock_guard<std::mutex>lock(mutex);copy=callbacks;}for(const auto&entry:copy)entry.second(entry.first);}
private:std::mutex mutex;uint64_t next=10000;std::map<tf2_ros::TimerHandle,tf2_ros::TimerCallbackType>callbacks;
};
class BufferCompat:public testing::Test {
protected:
  std::shared_ptr<rclcpp::Clock>clock=std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer{clock};std::shared_ptr<Timers>timers=std::make_shared<Timers>();
  static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{buffer.setCreateTimerInterface(timers);}
  void transform(int64_t stamp){geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base";t.header.stamp=rclcpp::Time(stamp);t.transform.rotation.w=1;ASSERT_TRUE(buffer.setTransform(t,"fixture"));}
};
TEST_F(BufferCompat, ImmediateHasNoTimer){transform(1000000000);int calls=0;
  auto future=buffer.waitForTransform("map","base",rclcpp::Time(1000000000),rclcpp::Duration::from_seconds(1),[&](auto){++calls;});
  EXPECT_EQ(future.get().child_frame_id,"base");EXPECT_EQ(calls,1);EXPECT_EQ(timers->size(),0u);
}
TEST_F(BufferCompat, TimeoutCompletesExactlyOnce){int calls=0;
  auto future=buffer.waitForTransform("map","base",rclcpp::Time(1000000000),rclcpp::Duration::from_seconds(1),[&](auto){++calls;});
  timers->fire();EXPECT_THROW(future.get(),tf2::TimeoutException);transform(1000000000);timers->fire();EXPECT_EQ(calls,1);EXPECT_EQ(timers->size(),0u);
}
TEST_F(BufferCompat, CancelRemovesTimerByRequestValue){int calls=0;
  auto future=buffer.waitForTransform("map","base",rclcpp::Time(1000000000),rclcpp::Duration::from_seconds(1),[&](auto){++calls;});
  buffer.cancel(future);EXPECT_EQ(timers->size(),0u);transform(1000000000);timers->fire();EXPECT_EQ(calls,0);
}
TEST_F(BufferCompat, ConcurrentRegistrationAndTransforms){
  std::atomic<bool>running{true};std::atomic<int64_t>stamp{1000000000};std::atomic<size_t>completed{0},failed{0};
  std::thread producer([&]{while(running){const auto next=stamp.fetch_add(100000);transform(next);std::this_thread::sleep_for(std::chrono::microseconds(30));}});
  constexpr size_t count=4000;
  for(size_t i=0;i<count;++i){buffer.waitForTransform("map","base",rclcpp::Time(stamp.load()+1000000),rclcpp::Duration::from_seconds(1),[&](auto f){try{f.get();}catch(...){++failed;}++completed;});}
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while(completed<count&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  running=false;producer.join();EXPECT_EQ(completed,count);EXPECT_EQ(failed,0u);EXPECT_EQ(timers->size(),0u);
}
