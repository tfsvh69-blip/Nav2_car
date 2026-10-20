#define JETSON_PROXY_TEST
#include "../proxy.cpp"
#include <openssl/ssl.h>
#include <openssl/x509v3.h>
#include <functional>

using namespace proxy;
void require(bool value, const char* text) {
  if (!value) throw std::runtime_error(text);
  std::cout << "PASS " << text << '\n';
}
class FixtureServer {
  Fd socket_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
 public:
  int port;
  explicit FixtureServer(std::function<void(int)> handler) : socket_(listener(0)) {
    sockaddr_in addr{}; socklen_t size = sizeof(addr);
    getsockname(socket_.n, reinterpret_cast<sockaddr*>(&addr), &size);
    port = ntohs(addr.sin_port);
    thread_ = std::thread([this, handler] {
      std::vector<std::thread> clients;
      while (!stop_) {
        pollfd p{socket_.n, POLLIN, 0};
        if (poll(&p, 1, 20) <= 0) continue;
        int fd = accept4(socket_.n, nullptr, nullptr, SOCK_CLOEXEC);
        if (fd < 0) continue;
        clients.emplace_back([handler, fd] {
          Fd owned(fd);
          timeval timeout{2, 0};
          setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
          setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
          handler(fd);
        });
      }
      for (auto& client : clients) client.join();
    });
  }
  ~FixtureServer() { stop_ = true; thread_.join(); }
};
class TlsFixture {
  SSL_CTX* ctx_ = nullptr;
  std::unique_ptr<FixtureServer> server_;
  std::string directory_;
 public:
  std::string ca;
  std::atomic<int> response{401};
  int port() const { return server_->port; }
  TlsFixture() {
    char name[] = "/tmp/jetson-proxy-test-XXXXXX";
    char* dir = mkdtemp(name);
    if (!dir) throw std::runtime_error("mkdtemp");
    directory_ = dir; ca = directory_ + "/ca.pem";
    EVP_PKEY_CTX* key_context = EVP_PKEY_CTX_new_id(EVP_PKEY_RSA, nullptr);
    EVP_PKEY* key = nullptr;
    EVP_PKEY_keygen_init(key_context);
    EVP_PKEY_CTX_set_rsa_keygen_bits(key_context, 2048);
    EVP_PKEY_keygen(key_context, &key);
    EVP_PKEY_CTX_free(key_context);
    X509* cert = X509_new(); X509_set_version(cert, 2);
    ASN1_INTEGER_set(X509_get_serialNumber(cert), 1);
    X509_gmtime_adj(X509_get_notBefore(cert), -60);
    X509_gmtime_adj(X509_get_notAfter(cert), 3600);
    X509_set_pubkey(cert, key);
    X509_NAME* subject = X509_get_subject_name(cert);
    X509_NAME_add_entry_by_txt(subject, "CN", MBSTRING_ASC,
      reinterpret_cast<const unsigned char*>("127.0.0.1"), -1, -1, 0);
    X509_set_issuer_name(cert, subject);
    for (const auto& item : std::vector<std::pair<int,const char*>>{
         {NID_subject_alt_name, "IP:127.0.0.1"}, {NID_basic_constraints, "critical,CA:TRUE"}}) {
      X509_EXTENSION* extension = X509V3_EXT_conf_nid(nullptr, nullptr, item.first, const_cast<char*>(item.second));
      X509_add_ext(cert, extension, -1); X509_EXTENSION_free(extension);
    }
    X509_sign(cert, key, EVP_sha256());
    FILE* file = fopen(ca.c_str(), "w"); PEM_write_X509(file, cert); fclose(file);
    ctx_ = SSL_CTX_new(TLS_server_method());
    SSL_CTX_use_certificate(ctx_, cert); SSL_CTX_use_PrivateKey(ctx_, key);
    X509_free(cert); EVP_PKEY_free(key);
    server_ = std::make_unique<FixtureServer>([this](int fd) {
      SSL* ssl = SSL_new(ctx_); SSL_set_fd(ssl, fd);
      if (SSL_accept(ssl) == 1) {
        char data[8192];
        if (SSL_read(ssl, data, sizeof(data)) > 0) {
          std::string response_text = "HTTP/1.1 " + std::to_string(response.load()) +
            " Test\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
          SSL_write(ssl, response_text.data(), static_cast<int>(response_text.size()));
        }
      }
      SSL_free(ssl);
    });
  }
  ~TlsFixture() { server_.reset(); SSL_CTX_free(ctx_); unlink(ca.c_str()); rmdir(directory_.c_str()); }
};
std::string request(int port, const std::string& payload) {
  Fd fd = connect_to("127.0.0.1", port, 500);
  if (!fd || !send_all(fd.n, payload)) throw std::runtime_error("fixture request failed");
  std::string result;
  if (!read_headers(fd.n, result, 1500)) throw std::runtime_error("fixture response failed");
  return result;
}
int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);
  curl_global_init(CURL_GLOBAL_DEFAULT);
  if (argc == 2 && std::string(argv[1]) == "--long-idle") {
    int a[2], b[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, a) || socketpair(AF_UNIX, SOCK_STREAM, 0, b)) return 1;
    for (int fd : {a[0], a[1], b[0], b[1]}) nonblocking(fd);
    std::thread bridge([&] { relay(a[1], b[1]); close(a[1]); close(b[1]); });
    std::cout << "Waiting 185 seconds with a completely idle local tunnel" << std::endl;
    std::this_thread::sleep_for(Ms(185000));
    bool sent = send_all(a[0], "ping");
    char buffer[16]{};
    bool readable = wait_fd(b[0], POLLIN, Clock::now() + Ms(1000));
    auto n = readable ? recv(b[0], buffer, sizeof(buffer), 0) : -1;
    bool ok = sent && n == 4 && std::string(buffer, 4) == "ping";
    close(a[0]); close(b[0]); bridge.join();
    std::cout << (ok ? "PASS" : "FAIL") << " tunnel survives 185 seconds idle" << std::endl;
    curl_global_cleanup(); return ok ? 0 : 1;
  }
  try {
    TlsFixture tls;
    std::atomic<int> good_connects{0}, failed_connects{0};
    std::atomic<bool> reject_tunnel{false};
    FixtureServer good([&](int fd) {
      nonblocking(fd); std::string header;
      if (!read_headers(fd, header, 1000)) return;
      ++good_connects;
      if (reject_tunnel) { reply(fd, 502, "Failed"); return; }
      Fd remote = connect_to("127.0.0.1", tls.port(), 500);
      if (!remote) return;
      send_all(fd, "HTTP/1.1 200 Connection established\r\n\r\n");
      relay(fd, remote.n);
    });
    FixtureServer fake([&](int fd) {
      nonblocking(fd); std::string header;
      if (!read_headers(fd, header, 1000)) return;
      ++failed_connects;
      send_all(fd, "HTTP/1.1 200 Connection established\r\n\r\n");
      // 返回200但没有目标TLS连接：旧健康检查会将它误判为可用。
    });
    Upstream healthy{"healthy", "127.0.0.1", good.port};
    Upstream false_positive{"fake200", "127.0.0.1", fake.port};
    Config config;
    config.gateways = false; config.probe_timeout = 700; config.interval = 500;
    config.cooldown = 500; config.connect_timeout = 250;
    config.probe_url = "https://127.0.0.1:" + std::to_string(tls.port()) + "/models";
    config.ca_file = tls.ca;
    require(probe(healthy, config).ok, "verified TLS + 401 accepted through proxy");
    require(!probe(false_positive, config).ok, "CONNECT 200 without TLS rejected");
    auto untrusted = config; untrusted.ca_file.clear();
    require(!probe(healthy, untrusted).ok, "untrusted certificate rejected");
    auto wrong_host = config;
    wrong_host.probe_url = "https://localhost:" + std::to_string(tls.port()) + "/models";
    require(!probe(healthy, wrong_host).ok, "certificate hostname mismatch rejected");
    tls.response = 403;
    require(!probe(healthy, config).ok, "403 is not healthy");
    tls.response = 401;
    config.fixed = {false_positive, healthy};
    Pool pool(config); pool.refresh();
    require(pool.ready().size() == 1 && pool.ready()[0].key() == healthy.key(), "fake higher priority skipped");
    FixtureServer frontend([&](int fd) { nonblocking(fd); handle(fd, pool, config); });
    Config via = config; via.ca_file = tls.ca;
    require(probe({"front", "127.0.0.1", frontend.port}, via).ok, "complete HTTPS through frontend succeeds");
    pool.fail(healthy);
    auto start = Clock::now();
    std::string unavailable = request(frontend.port, "CONNECT target:443 HTTP/1.1\r\nHost: target:443\r\n\r\n");
    require(unavailable.find("503") != std::string::npos && Clock::now() - start < Ms(300), "all down fails quickly without synchronous probing");
    pool.refresh();
    require(pool.ready().empty(), "cooldown not bypassed by refresh");
    std::this_thread::sleep_for(Ms(550));
    pool.refresh();
    require(pool.ready().size() == 1, "automatic recovery after cooldown");
    // 先将两个代理都验为健康，再让优先代理在真正CONNECT时返回502。
    std::atomic<int> backup_connects{0};
    FixtureServer backup([&](int fd) {
      nonblocking(fd); std::string header;
      if (!read_headers(fd, header, 1000)) return;
      ++backup_connects;
      Fd remote = connect_to("127.0.0.1", tls.port(), 500);
      if (!remote) return;
      send_all(fd, "HTTP/1.1 200 Connection established\r\n\r\n"); relay(fd, remote.n);
    });
    Config failover = config;
    failover.fixed = {healthy, {"backup", "127.0.0.1", backup.port}};
    Pool two(failover); two.refresh();
    require(two.ready().size() == 2, "both candidates initially healthy");
    FixtureServer frontend2([&](int fd) { nonblocking(fd); handle(fd, two, failover); });
    reject_tunnel = true;
    require(probe({"front2", "127.0.0.1", frontend2.port}, config).ok, "CONNECT failure retries healthy backup before TLS payload");
    require(two.ready().size() == 1 && two.ready()[0].name == "backup", "failed candidate enters cooldown");
    reject_tunnel = false;
    std::this_thread::sleep_for(Ms(550)); two.refresh();
    require(two.ready().size() == 2 && two.ready()[0].name == "healthy", "preferred candidate restored after recovery");
    // 并发探测的网络等待不得持有请求选择锁。
    std::atomic<bool> stall_entered{false};
    std::atomic<bool> stall{false};
    FixtureServer slow([&](int fd) {
      nonblocking(fd); std::string header;
      if (!read_headers(fd, header, 1000)) return;
      if (stall) {
        stall_entered = true;
        std::this_thread::sleep_for(Ms(500)); return;
      }
      Fd remote = connect_to("127.0.0.1", tls.port(), 500);
      if (!remote) return;
      send_all(fd, "HTTP/1.1 200 Connection established\r\n\r\n"); relay(fd, remote.n);
    });
    Config concurrent = config;
    concurrent.fixed = {{"slow", "127.0.0.1", slow.port}, healthy};
    Pool concurrent_pool(concurrent); concurrent_pool.refresh();
    stall = true;
    auto refresh = std::async(std::launch::async, [&] { concurrent_pool.refresh(); });
    auto wait_limit = Clock::now() + Ms(1000);
    while (!stall_entered && Clock::now() < wait_limit) std::this_thread::sleep_for(Ms(1));
    auto before = Clock::now(); auto snapshot = concurrent_pool.ready();
    bool quick = Clock::now() - before < Ms(100);
    refresh.get();
    require(stall_entered && quick && !snapshot.empty(), "slow health probe does not block request selection");
    // 明文POST不允许因为上游断开而在下一候选重复提交。
    std::atomic<int> posts{0};
    FixtureServer post_sink([&](int fd) {
      nonblocking(fd); std::string header;
      if (!read_headers(fd, header, 1000)) return;
      if (header.rfind("CONNECT ", 0) == 0) {
        Fd remote = connect_to("127.0.0.1", tls.port(), 500);
        send_all(fd, "HTTP/1.1 200 Connection established\r\n\r\n"); relay(fd, remote.n);
      } else ++posts;
    });
    Config post_config = config;
    post_config.fixed = {{"post_sink", "127.0.0.1", post_sink.port}, {"backup", "127.0.0.1", backup.port}};
    Pool post_pool(post_config); post_pool.refresh();
    FixtureServer post_front([&](int fd) { nonblocking(fd); handle(fd, post_pool, post_config); });
    int backup_before = backup_connects.load();
    {
      Fd fd = connect_to("127.0.0.1", post_front.port, 500);
      send_all(fd.n, "POST http://fixture/submit HTTP/1.1\r\nHost: fixture\r\nContent-Length: 1\r\n\r\nx");
      shutdown(fd.n, SHUT_WR);
      wait_fd(fd.n, POLLIN, Clock::now() + Ms(1000));
    }
    require(posts == 1 && backup_connects == backup_before, "POST sent once after upstream disconnect");
    // 半关闭后仍允许对端回包。
    int a[2], b[2]; socketpair(AF_UNIX, SOCK_STREAM, 0, a); socketpair(AF_UNIX, SOCK_STREAM, 0, b);
    for (int fd : {a[0], a[1], b[0], b[1]}) nonblocking(fd);
    std::thread bridge([&] { relay(a[1], b[1]); close(a[1]); close(b[1]); });
    send_all(a[0], "hello"); shutdown(a[0], SHUT_WR);
    char data[16]{}; wait_fd(b[0], POLLIN, Clock::now()+Ms(500));
    auto count = recv(b[0], data, sizeof(data), 0);
    bool forwarded = count == 5 && std::string(data, 5) == "hello";
    send_all(b[0], "world"); shutdown(b[0], SHUT_WR);
    wait_fd(a[0], POLLIN, Clock::now()+Ms(500)); count = recv(a[0], data, sizeof(data), 0);
    bool returned = count == 5 && std::string(data, 5) == "world";
    close(a[0]); close(b[0]); bridge.join();
    require(forwarded && returned, "TCP half-close preserves response");
    require(no_proxy().find("127.0.0.1") != std::string::npos, "dynamic bypass keeps loopback");
    std::cout << "ALL REGRESSIONS PASSED\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL " << e.what() << '\n'; return 1;
  }
  curl_global_cleanup();
}
