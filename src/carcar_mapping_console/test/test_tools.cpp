#include "carcar_mapping_console/profile_backup.hpp"
#include "carcar_mapping_console/task_process.hpp"
#include "carcar_mapping_console/serial_preflight.hpp"
#include <gtest/gtest.h>
#include <sys/wait.h>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sys/socket.h>
#include <poll.h>

using namespace carcar_mapping_console;
namespace fs = std::filesystem;
namespace {
class Scratch {
public:
  fs::path root = fs::temp_directory_path() / ("mapping-tools-" + nonce());
  Scratch() {fs::create_directory(root);}
  ~Scratch() {fs::remove_all(root);}
};
int run(std::vector<std::string> args, const fs::path & capture = {}, const std::string & fuser_mode = {}) {
  const auto child = fork();
  if (child < 0) return -1;
  if (!child) {
    if (!capture.empty()) {
      setenv("PATH", (capture.parent_path().string() + ":" + getenv("PATH")).c_str(), 1);
      setenv("MAPPING_CAPTURE_OUTPUT", capture.c_str(), 1);
      // 预检替身使用独立域，避免并发回放的地图被误认为真实入口冲突。
      setenv("ROS_DOMAIN_ID", "223", 1); setenv("ROS_LOCALHOST_ONLY", "1", 1);
      if (!fuser_mode.empty()) setenv("MAPPING_FUSER_MODE", fuser_mode.c_str(), 1);
      if (fuser_mode == "missing") setenv("PATH", capture.parent_path().c_str(), 1);
    }
    std::vector<char *> argv; for (auto & a : args) argv.push_back(a.data()); argv.push_back(nullptr);
    execvp(argv[0], argv.data()); _exit(127);
  }
  int status = 0; if (waitpid(child, &status, 0) < 0) return -1;
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}
TEST(MappingTools, NewPreparationPassesNonemptyLaunchArgumentsWithoutHardware) {
  Scratch scratch;
  fs::create_symlink(fs::read_symlink("/proc/self/exe"), scratch.root / "ros2");
  const auto capture = scratch.root / "args.json";
  ASSERT_EQ(run({PREPARE_EXE, "--mode", "new", "--map-root", (scratch.root / "maps with space").string()}, capture), 0);
  const auto args = json::parse(read_file(capture)).get<std::vector<std::string>>();
  EXPECT_NE(std::find(args.begin(), args.end(), "start_hardware:=false"), args.end());
  EXPECT_NE(std::find(args.begin(), args.end(), "mode:=new"), args.end());
  EXPECT_NE(std::find(args.begin(), args.end(), "map_root:=" + (scratch.root / "maps with space").string()), args.end());
  for (const auto & a : args) EXPECT_FALSE(a.size() >= 2 && a.substr(a.size() - 2) == ":=") << a;
}
TEST(MappingTools, HardwarePreflightRejectsOccupiedPortsAndProbeErrorsWithoutDevices) {
  Scratch scratch;
  fs::create_symlink(fs::read_symlink("/proc/self/exe"), scratch.root/"ros2");
  const auto base=scratch.root/"base port", lidar=scratch.root/"lidar port", capture=scratch.root/"args.json";
  {std::ofstream(lidar)<<"临时文件，不是设备";}
  const std::vector<std::string> args{PREPARE_EXE,"--hardware","--serial",base.string(),"--lidar",lidar.string(),"--map-root",(scratch.root/"maps").string()};
  {
    std::ofstream held(base); held<<"占用替身"; held.flush();
    EXPECT_NE(run(args,capture),0); EXPECT_FALSE(fs::exists(capture));
  }
  ASSERT_EQ(run(args,capture),0); // ros2是仅记录argv的替身，绝不启动硬件。
  const auto launched=json::parse(read_file(capture)).get<std::vector<std::string>>();
  EXPECT_NE(std::find(launched.begin(),launched.end(),"start_hardware:=true"),launched.end());
  fs::remove(capture);
  fs::create_symlink(fs::read_symlink("/proc/self/exe"),scratch.root/"fuser");
  EXPECT_NE(run(args,capture),0); EXPECT_FALSE(fs::exists(capture));
}
TEST(MappingTools, CandidateSlamConfigReachesLaunchWithoutHardwareAndRequiresNewMap) {
  Scratch scratch;
  fs::create_symlink(fs::read_symlink("/proc/self/exe"), scratch.root / "ros2");
  const auto config = scratch.root / "slam candidate.yaml", capture = scratch.root / "args.json";
  {std::ofstream(config) << "slam_toolbox:\n  ros__parameters:\n    max_laser_range: 6.0\n";}
  ASSERT_EQ(run({PREPARE_EXE, "--slam-params", config.string()}, capture), 0);
  const auto args = json::parse(read_file(capture)).get<std::vector<std::string>>();
  EXPECT_NE(std::find(args.begin(), args.end(), "slam_params_file:=" + fs::canonical(config).string()), args.end());
  EXPECT_NE(std::find(args.begin(), args.end(), "start_hardware:=false"), args.end());
  fs::remove(capture);
  EXPECT_NE(run({PREPARE_EXE, "--slam-params", config.string(), "--mode", "resume"}, capture), 0);
  EXPECT_FALSE(fs::exists(capture));
  EXPECT_NE(run({PREPARE_EXE, "--slam-params", (scratch.root / "missing.yaml").string()}, capture), 0);
  EXPECT_FALSE(fs::exists(capture));
}
TEST(MappingTools, SharedSerialPreflightRejectsBusyPortsAndAllProbeFailures) {
  Scratch scratch;
  const auto port = scratch.root / "base port", alias = scratch.root / "serial alias";
  {
    std::ofstream held(port); held << "临时文件，不是设备"; held.flush();
    fs::create_symlink(port, alias);
    EXPECT_THROW(require_unused_serial(alias), std::runtime_error);
  }
  EXPECT_NO_THROW(require_unused_serial(alias));
  EXPECT_THROW(require_unused_serial(scratch.root / "missing port"), std::runtime_error);

  const auto executable = fs::read_symlink("/proc/self/exe");
  fs::create_symlink(executable, scratch.root / "fuser");
  const auto capture = scratch.root / "probe-args.json";
  const std::vector<std::string> args{executable.string(), "--probe-serial", alias.string()};
  ASSERT_EQ(run(args, capture, "idle"), 0);
  const auto argv = json::parse(read_file(capture)).get<std::vector<std::string>>();
  ASSERT_EQ(argv.size(), 1u); EXPECT_EQ(argv.front(), fs::canonical(port).string());
  for (const auto & mode : {"busy", "error", "newline", "exit2", "signal"}) {
    SCOPED_TRACE(mode);
    EXPECT_NE(run(args, capture, mode), 0);
  }
  Scratch missing_tool;
  EXPECT_NE(run(args, missing_tool.root / "capture.json", "missing"), 0);
}
TEST(MappingTools, ProfileBackupPreservesBytesPermissionsAndEarlierBackup) {
  Scratch scratch; const auto source = scratch.root / "mapping profile.nmconnection";
  std::string content = "[connection]\nuuid=test\n[security]\npsk=offline-test\n";
  content.append(1, '\0'); content += "end";
  {std::ofstream file(source, std::ios::binary); file.write(content.data(), content.size());}
  const auto first = backup_profile(source), second = backup_profile(source);
  EXPECT_NE(first, second); EXPECT_EQ(first.parent_path(), source.parent_path());
  EXPECT_EQ(first.extension(), ".backup"); EXPECT_EQ(read_file(first), content); EXPECT_EQ(read_file(second), content);
  struct stat info{}; ASSERT_EQ(stat(first.c_str(), &info), 0); EXPECT_EQ(info.st_mode & 0777, 0600);
  fs::remove(source); EXPECT_EQ(read_file(first), content);
}
TEST(MappingTools, MissingOrSymlinkProfileIsRejectedWithoutCreatingBackup) {
  Scratch scratch;
  EXPECT_THROW(backup_profile(scratch.root / "missing"), std::runtime_error);
  {std::ofstream(scratch.root / "profile") << "unmodified";}
  fs::create_symlink(scratch.root / "profile", scratch.root / "alias");
  EXPECT_THROW(backup_profile(scratch.root / "alias"), std::runtime_error);
  EXPECT_THROW(backup_profile(scratch.root), std::runtime_error);
  EXPECT_EQ(read_file(scratch.root / "profile"), "unmodified");
  EXPECT_EQ(std::distance(fs::directory_iterator(scratch.root), fs::directory_iterator()), 2);
}
TEST(MappingTools, ChineseDiagnosticTailRemainsValidJsonAfterByteTruncation) {
  Scratch scratch; const auto log=scratch.root/"diagnostic.log";
  {std::ofstream file(log);for(int i=0;i<2001;++i)file<<"雷";file<<"x";}
  const auto tail=log_tail(log); EXPECT_LE(tail.size(),6000u);
  EXPECT_NE(tail.find("雷"),std::string::npos); EXPECT_EQ(tail.back(),'x');
  EXPECT_NO_THROW(json::parse(json{{"log",tail}}.dump()));
}
TEST(MappingTools, BrowserReconnectCancelsRetryAndIgnoresOldSocketCallbacks) {
  const std::string script = R"JS(
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const timers=new Map(),listeners={},sockets=[],elements={};let nextTimer=1;
const element=id=>elements[id]??(elements[id]={id,style:{},getContext:()=>({setTransform(){},fillRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){},fillText(){}}),classList:{toggle(){},add(){}},clientWidth:600,clientHeight:400});
class WS {static OPEN=1;static CLOSED=3;constructor(){this.readyState=0;this.sent=[];sockets.push(this);}close(){this.readyState=3;}send(value){this.sent.push(JSON.parse(value));}}
const doc={hidden:false,getElementById:element,querySelectorAll:()=>[],addEventListener:(name,fn)=>listeners[name]=fn};
const context={document:doc,window:{devicePixelRatio:1,addEventListener(){}},location:{host:'localhost:8080'},WebSocket:WS,performance:{now:()=>0},setInterval(){},setTimeout:(fn,ms)=>{const id=nextTimer++;if(ms===1000)timers.set(id,fn);return id;},clearTimeout:id=>timers.delete(id),AbortController,fetch:()=>new Promise(()=>{}),requestAnimationFrame(){},console};
vm.createContext(context);vm.runInContext(fs.readFileSync(process.argv[1],'utf8'),context);
const old=sockets[0];old.readyState=3;old.onclose();assert.equal(timers.size,1);
const pending=[...timers.values()][0];
doc.hidden=true;listeners.visibilitychange();assert.equal(timers.size,0);
doc.hidden=false;listeners.visibilitychange();assert.equal(sockets.length,2);
const healthy=sockets[1];healthy.readyState=1;healthy.onmessage({data:JSON.stringify({session:'current'})});
pending();assert.equal(sockets.length,2);assert.equal(vm.runInContext('session',context),'current');
old.onclose();old.onerror();old.onmessage({data:JSON.stringify({session:'obsolete'})});
assert.equal(vm.runInContext('session',context),'current');assert.equal(timers.size,0);
doc.hidden=true;listeners.visibilitychange();assert.equal(vm.runInContext('session',context),'');
assert.equal(healthy.readyState,3);healthy.onclose();assert.equal(timers.size,0);
vm.runInContext('connect()',context);assert.equal(sockets.length,2);
doc.hidden=false;listeners.visibilitychange();assert.equal(sockets.length,3);
const failed=sockets[2];failed.readyState=3;failed.onclose();assert.equal(timers.size,1);
const retry=[...timers.values()][0];timers.clear();retry();assert.equal(sockets.length,4);
assert.equal(vm.runInContext('session',context),'');
const current=sockets[3];current.readyState=1;current.bufferedAmount=0;current.onmessage({data:JSON.stringify({session:'joystick'})});
vm.runInContext("state={phase:'CURRENT',control:{armed:true},control_age:0};challenge='fresh';",context);
current.onmessage({data:JSON.stringify({armed:true,challenge:'fresh',health:''})});
const stick=element('joystick');stick.getBoundingClientRect=()=>({left:0,top:0,width:210,height:210});stick.setPointerCapture=()=>{};
const event={pointerId:7,clientX:180,clientY:30,preventDefault(){}};
stick.onpointerdown(event);let command=current.sent.at(-1);assert.equal(command.action,'drive');assert(command.x>0);assert(command.yaw<0);assert(Math.hypot(command.x,command.yaw)<=1.00001);
stick.onpointerup(event);assert.equal(current.sent.at(-1).action,'release');assert.equal(vm.runInContext('pressed',context),'');
stick.onpointerdown(event);assert.equal(current.sent.at(-1).action,'drive');
stick.onpointercancel(event);assert.equal(current.sent.at(-1).action,'stop');assert.equal(vm.runInContext('activePointer',context),null);
)JS";
  EXPECT_EQ(run({NODE_EXE, "-e", script, APP_JS}), 0);
}
TEST(MappingTools, BrowserHeldJoystickIgnoresHttpDelayButStopsOnControlTimeout) {
  EXPECT_EQ(run({NODE_EXE,BROWSER_TEST,APP_JS}),0);
}
TEST(MappingTools, BrowserUdpTransportBoundsTrafficRejectsReorderAndLocksOnFallback) {
  EXPECT_EQ(run({NODE_EXE,RTC_BROWSER_TEST,APP_JS}),0);
}
TEST(MappingTools, BackgroundTaskCannotInheritAndKeepTheControllerSocketOpen) {
  Scratch scratch;int pair[2];ASSERT_EQ(socketpair(AF_UNIX,SOCK_STREAM,0,pair),0);
  const int server=fcntl(pair[0],F_DUPFD,100);close(pair[0]);ASSERT_GE(server,100);
  TaskProcess task;const auto log=scratch.root/"child.log";
  task.start({fs::read_symlink("/proc/self/exe").string(),"--probe-fd-closed",std::to_string(server)},log);
  close(server);pollfd peer{pair[1],POLLIN|POLLHUP,0};
  EXPECT_GT(poll(&peer,1,200),0)<<"网关关闭连接后，后台子进程仍持有socket，客户端收不到断开";
  // closefrom在exec前生效，EOF可能早于子进程main打印日志；分别等待两个可观察结果。
  const auto until=steady()+1;while(fs::file_size(log)==0&&steady()<until)usleep(5000);
  EXPECT_NE(read_file(log).find("fd_closed"),std::string::npos);
  close(pair[1]);task.stop();
}
TEST(MappingTools, FirstGatewayHttpResponseIsReadyWithoutWaitingForStateTimer) {
  EXPECT_EQ(run({NODE_EXE,STARTUP_TEST,GATEWAY_EXE}),0);
}
}  // namespace
int main(int argc, char ** argv) {
  if(argc==3&&std::string(argv[1])=="--probe-fd-closed") {
    const bool closed=fcntl(std::stoi(argv[2]),F_GETFD)==-1&&errno==EBADF;
    std::cout<<(closed?"fd_closed":"fd_open")<<std::endl;
    usleep(500000);return closed?0:1;
  }
  if (fs::path(argv[0]).filename() == "fuser") {
    const auto configured = getenv("MAPPING_FUSER_MODE");
    if (configured) if (const auto output = getenv("MAPPING_CAPTURE_OUTPUT")) {
      std::ofstream file(output); file << json(std::vector<std::string>(argv + 1, argv + argc));
    }
    const std::string mode = configured ? configured : "error";
    if (mode == "idle") return 1;
    if (mode == "busy") {std::cout << "12345\n"; return 0;}
    if (mode == "newline") {std::cout << '\n'; return 1;}
    if (mode == "exit2") return 2;
    if (mode == "signal") {raise(SIGTERM); return 2;}
    std::cerr << "离线替身：占用检查工具执行错误\n"; return 1;
  }
  if (argc == 3 && std::string(argv[1]) == "--probe-serial") {
    try {require_unused_serial(argv[2]); return 0;}
    catch (const std::exception & error) {std::cerr << error.what() << '\n'; return 1;}
  }
  // 仅供预检测试捕获 exec 的完整 argv；不会启动 ROS 节点或执行器。
  if (fs::path(argv[0]).filename() == "ros2") {
    const auto output = getenv("MAPPING_CAPTURE_OUTPUT"); if (!output) return 2;
    std::ofstream file(output); file << json(std::vector<std::string>(argv + 1, argv + argc)); return file ? 0 : 3;
  }
  testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS();
}
