#include "behaviortree_cpp/bt_factory.h"

#include <atomic>
#include <future>
#include <thread>

#include <gtest/gtest.h>

using namespace BT;

class FastAction : public BT::ThreadedAction
{
public:
  // Any TreeNode with ports must have a constructor with this signature
  FastAction(const std::string& name, const BT::NodeConfig& config)
    : ThreadedAction(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return BT::NodeStatus::SUCCESS;
  }
};

TEST(WakeUp, BasicTest)
{
  static const char* xml_text = R"(

<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <FastAction/>
    </BehaviorTree>
</root> )";

  BehaviorTreeFactory factory;
  factory.registerNodeType<FastAction>("FastAction");

  Tree tree = factory.createTreeFromText(xml_text);

  using namespace std::chrono;

  auto t1 = steady_clock::now();
  tree.tickOnce();
  tree.sleep(milliseconds(200));
  auto t2 = steady_clock::now();

  auto dT = duration_cast<milliseconds>(t2 - t1).count();
  std::cout << "Woke up after msec: " << dT << std::endl;

  // steady_clock is monotonic and immune to NTP/DST jumps.
  // 100 ms gives headroom for scheduler jitter on loaded CI runners;
  // the wake-up should fire well under that threshold.
  ASSERT_LT(dT, 100);
}

// Issue #686: haltTree() called from another thread must stop
// tickWhileRunning(), instead of letting it tick (and restart) the tree again.
TEST(WakeUp, HaltTreeStopsTickWhileRunning_Issue686)
{
  static std::atomic_int starts;
  static std::atomic_bool give_up;
  starts = 0;
  give_up = false;

  class RunForever : public StatefulActionNode
  {
  public:
    RunForever(const std::string& name, const NodeConfig& config)
      : StatefulActionNode(name, config)
    {}
    static PortsList providedPorts()
    {
      return {};
    }
    NodeStatus onStart() override
    {
      starts++;
      return NodeStatus::RUNNING;
    }
    NodeStatus onRunning() override
    {
      // escape hatch, so that a failing test does not hang forever
      return give_up ? NodeStatus::SUCCESS : NodeStatus::RUNNING;
    }
    void onHalted() override
    {}
  };

  BehaviorTreeFactory factory;
  factory.registerNodeType<RunForever>("RunForever");

  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <RunForever/>
      </BehaviorTree>
    </root>)");

  auto result = std::async(std::launch::async, [&] {
    return tree.tickWhileRunning(std::chrono::milliseconds(10000));
  });

  while(starts == 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  tree.haltTree();

  // haltTree must also interrupt the 10 seconds sleep
  const bool returned =
      result.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
  give_up = true;
  tree.emitWakeUpSignal();

  ASSERT_TRUE(returned);
  ASSERT_EQ(result.get(), NodeStatus::IDLE);
  ASSERT_EQ(starts, 1);
}

// The first tick can still be inside onStart() when the other thread observes
// startup. Halting must wait for that tick rather than have its IDLE status
// overwritten by the tick's eventual RUNNING result.
TEST(WakeUp, HaltWaitsForInFlightTick)
{
  using namespace std::chrono_literals;
  for(const bool first_tick : { true, false })
  {
    SCOPED_TRACE(first_tick ? "initial tick" : "running tick");
    std::promise<void> entered;
    std::promise<void> release;
    auto release_future = release.get_future();
    std::atomic<int> ticks{ 0 };
    std::atomic_bool stop{ false };
    BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BlockingTick", [&](TreeNode&) {
      const int tick = ++ticks;
      if(tick == (first_tick ? 1 : 2))
      {
        entered.set_value();
        release_future.wait();
      }
      return stop ? NodeStatus::SUCCESS : NodeStatus::RUNNING;
    });
    auto tree = factory.createTreeFromText(R"(
      <root BTCPP_format="4"><BehaviorTree><BlockingTick/></BehaviorTree></root>)");
    if(!first_tick)
    {
      EXPECT_EQ(tree.tickExactlyOnce(), NodeStatus::RUNNING);
    }
    auto ticking =
        std::async(std::launch::async, [&] { return tree.tickWhileRunning(10s); });
    entered.get_future().wait();
    std::promise<void> halt_started;
    auto halting = std::async(std::launch::async, [&] {
      halt_started.set_value();
      tree.haltTree();
    });
    halt_started.get_future().wait();
    EXPECT_EQ(halting.wait_for(50ms), std::future_status::timeout);
    release.set_value();
    halting.get();

    const bool returned = ticking.wait_for(2s) == std::future_status::ready;
    stop = true;
    tree.emitWakeUpSignal();
    EXPECT_TRUE(returned);
    EXPECT_EQ(ticking.get(), NodeStatus::IDLE);
    EXPECT_EQ(tree.rootNode()->status(), NodeStatus::IDLE);
    EXPECT_EQ(ticks, first_tick ? 1 : 2);
  }
}

TEST(WakeUp, HaltFromTickCallbackDoesNotRestartTree)
{
  BehaviorTreeFactory factory;
  Tree* current_tree = nullptr;
  int ticks = 0;
  factory.registerSimpleCondition("HaltDuringTick", [&](TreeNode&) {
    if(++ticks == 1)
    {
      current_tree->haltTree();
      return NodeStatus::RUNNING;
    }
    return NodeStatus::SUCCESS;
  });
  auto tree = factory.createTreeFromText(R"(
    <root BTCPP_format="4"><BehaviorTree><HaltDuringTick/></BehaviorTree></root>)");
  current_tree = &tree;
  EXPECT_EQ(tree.tickWhileRunning(), NodeStatus::IDLE);
  EXPECT_EQ(tree.rootNode()->status(), NodeStatus::IDLE);
  EXPECT_EQ(ticks, 1);
  // A later explicit tick starts a new execution, independent of the old halt.
  EXPECT_EQ(tree.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_EQ(ticks, 2);
}
