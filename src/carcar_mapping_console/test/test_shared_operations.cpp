#include <gtest/gtest.h>
#include "carcar_mapping_console/operator_sessions.hpp"
#include "carcar_mapping_console/task_requests.hpp"
using namespace carcar_mapping_console;
using nlohmann::json;
TEST(SharedOperations, DevicesShareTaskIdentityWithoutDisclosingControlTokens){
  OperatorSessions sessions;
  ASSERT_TRUE(sessions.connect("secret-a","public-a"));
  ASSERT_FALSE(sessions.connect("secret-b","public-b"));
  EXPECT_TRUE(sessions.connected("secret-b"));EXPECT_FALSE(sessions.owns("secret-b"));
  EXPECT_EQ(sessions.status().dump().find("secret"),std::string::npos);
  EXPECT_FALSE(sessions.disconnect("secret-b"));EXPECT_TRUE(sessions.owns("secret-a"));
  sessions.connect("secret-b","public-b");sessions.claim("secret-b");
  EXPECT_FALSE(sessions.owns("secret-a"));EXPECT_TRUE(sessions.owns("secret-b"));
  EXPECT_FALSE(sessions.disconnect("secret-a"));EXPECT_TRUE(sessions.owns("secret-b"));
}
TEST(SharedOperations, DifferentDevicesContinueOnceAndReceiveOwnResults){
  TaskRequests ledger;ledger.reset("task");const json epoch=ledger.epoch();
  const auto first=ledger.submit("pc","start",json::object(),1,epoch);
  const auto second=ledger.submit("phone","resume",json::object(),1.05,epoch);
  EXPECT_TRUE(first.queued);EXPECT_FALSE(second.forward);EXPECT_EQ(second.canonical,"pc");
  EXPECT_TRUE(ledger.ready(1.1).empty());const auto requests=ledger.ready(1.2);
  ASSERT_EQ(requests.size(),1u);EXPECT_EQ(requests[0]["request_id"],"pc");
  ledger.acknowledge("pc","accepted","已开始");
  const auto status=ledger.status();ASSERT_EQ(status["requests"].size(),2u);
  for(const auto&r:status["requests"])EXPECT_EQ(r["request_result"],"accepted");
  EXPECT_TRUE(ledger.ready(2).empty());
}
TEST(SharedOperations, StopWinsEitherArrivalOrderAndOldEpochCannotRestart){
  for(const bool stop_first:{false,true}){
    TaskRequests ledger;ledger.reset("task");const json epoch=ledger.epoch();
    if(!stop_first)EXPECT_TRUE(ledger.submit("continue","start",{},1,epoch).queued);
    EXPECT_TRUE(ledger.submit("stop","pause",{},1.01,epoch).forward);
    if(stop_first)EXPECT_FALSE(ledger.submit("continue","start",{},1.02,epoch).error.empty());
    EXPECT_TRUE(ledger.ready(2).empty());
    EXPECT_FALSE(ledger.submit("late","start",{},3,epoch).error.empty());
    EXPECT_TRUE(ledger.submit("new","start",{},3,json(ledger.epoch())).queued);
    EXPECT_EQ(ledger.ready(4).size(),1u);
  }
}
TEST(SharedOperations, IdCollisionAndTaskSwitchDoNotReplayOldCommands){
  TaskRequests ledger;ledger.reset("one");const json epoch=ledger.epoch();
  ledger.submit("id","start",{},0,epoch);
  EXPECT_FALSE(ledger.submit("id","return",{},0,epoch).error.empty());
  ledger.reset("two");EXPECT_TRUE(ledger.ready(1).empty());
  EXPECT_FALSE(ledger.submit("stale","start",{},1,epoch).error.empty());
}
TEST(SharedOperations, RunningRecheckCanBeJoinedAfterItsInitialAcceptance){
  TaskRequests ledger;ledger.reset("task");const json epoch=ledger.epoch();
  EXPECT_TRUE(ledger.submit("pc","recheck",{},1,epoch).forward);
  ledger.acknowledge("pc","accepted","检查中");
  const auto joined=ledger.submit("phone","recheck",{},2,epoch,"pc");
  EXPECT_TRUE(joined.error.empty());EXPECT_FALSE(joined.forward);EXPECT_EQ(joined.canonical,"pc");
  EXPECT_EQ(ledger.status()["requests"][1]["request_result"],"accepted");
}
TEST(SharedOperations, BoundedLedgerDoesNotLeaveAliasesWithoutTheirCanonical){
  TaskRequests ledger;ledger.reset("task");const json epoch=ledger.epoch();
  for(int i=0;i<140;++i)ledger.submit("device-"+std::to_string(i),"start",{},1,epoch);
  const auto status=ledger.status();EXPECT_LE(status["requests"].size(),128u);
  const auto ready=ledger.ready(2);EXPECT_EQ(ready.size(),1u);
  ASSERT_FALSE(status["requests"].empty());const auto canonical=status["requests"][0]["canonical_request_id"];
  bool found=false;for(const auto&r:status["requests"])found|=r["request_id"]==canonical;EXPECT_TRUE(found);
}
TEST(SharedOperations, ExplicitManualStopCancelsQueuedAndAlreadySubmittedContinuation){
  TaskRequests ledger;ledger.reset("task");
  ledger.submit("queued","start",{},1,json(ledger.epoch()));
  EXPECT_FALSE(ledger.continuation_submitted());ledger.cancel_continuations("明确停车");
  EXPECT_TRUE(ledger.ready(2).empty());
  ledger.submit("sent","resume",{},3,json(ledger.epoch()));ledger.ready(4);
  EXPECT_TRUE(ledger.continuation_submitted());ledger.cancel_continuations("明确停车");
  EXPECT_FALSE(ledger.continuation_submitted());ledger.acknowledge("sent","accepted","迟到确认");
  const auto status=ledger.status();EXPECT_EQ(status["requests"][1]["request_result"],"cancelled");
}
