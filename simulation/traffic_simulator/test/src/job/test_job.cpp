#include <gtest/gtest.h>

#include <traffic_simulator/job/job.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Job, onUpdate)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKNOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);
  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(was_cleanup_func_called);
}

TEST(Job, get_type)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKNOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);

  EXPECT_TRUE(job.type == type);

  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(job.type == type);
}

TEST(Job, get_exclusive)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKNOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);

  EXPECT_TRUE(job.exclusive == is_exclusive);

  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(job.exclusive == is_exclusive);
}

TEST(Job, get_event)
{
  bool was_cleanup_func_called = false;
  auto update_func = [](const double) { return true; };
  auto cleanup_func = [&was_cleanup_func_called]() { was_cleanup_func_called = true; };
  const auto type = traffic_simulator::job::Type::UNKNOWN;
  const auto event = traffic_simulator::job::Event::POST_UPDATE;
  const bool is_exclusive = true;

  auto job = traffic_simulator::job::Job(update_func, cleanup_func, type, is_exclusive, event);

  EXPECT_TRUE(job.event == event);

  const double step_time = 0.0;
  job.onUpdate(step_time);

  EXPECT_TRUE(job.event == event);
}