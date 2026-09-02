# Jetson + VS Code Remote SSH + Codex 代理配置笔记

## 1. 当前已跑通的网络结构

```text
Windows 11 主机
IP：192.168.5.17
Digilink Mixed Port：7892
        │
        │ 局域网
        ▼
Jetson
IP：192.168.5.11
        │
        ▼
VS Code Remote SSH
        │
        ▼
Codex
```

关键点：

- Windows 与 Jetson 在同一局域网：`192.168.5.0/24`
- Digilink 开启「局域网代理」
- Digilink 混合端口：`7892`
- Jetson 通过 `192.168.5.17:7892` 使用电脑代理访问外网

---

## 2. Windows 端 Digilink

开启：

```text
局域网代理：开启
混合端口：7892
```

验证：

```powershell
netstat -ano | findstr :7892
```

正常应看到：

```text
0.0.0.0:7892   LISTENING
```

---

## 3. Jetson `.bashrc` 代理配置

代理配置要放在下面这段 **之前**：

```bash
case $- in
    *i*) ;;
      *) return;;
esac
```

加入：

```bash
# ============================================================
# Windows 主机代理
# ============================================================

export http_proxy="http://192.168.5.17:7892"
export https_proxy="http://192.168.5.17:7892"
export HTTP_PROXY="http://192.168.5.17:7892"
export HTTPS_PROXY="http://192.168.5.17:7892"

# 局域网通信不经过代理
export no_proxy="localhost,127.0.0.1,::1,192.168.5.0/24,192.168.5.11,192.168.5.17"
export NO_PROXY="$no_proxy"
```

然后：

```bash
source ~/.bashrc
```

验证：

```bash
env | grep -i proxy
curl -I https://www.google.com
```

---

## 4. VS Code Windows 本机设置

`User Settings (JSON)` 中保留：

```json
"http.proxy": "http://127.0.0.1:7892",
"http.proxyStrictSSL": false
```

这里的 `127.0.0.1` 指的是 **Windows 自己**，所以本机 VS Code 可以正常连接 Digilink。

---

## 5. VS Code Remote SSH 设置

连接 Jetson 后打开：

```text
Preferences: Open Remote Settings (JSON)
```

配置：

```json
{
    "http.useLocalProxyConfiguration": false,
    "http.proxy": "http://192.168.5.17:7892",
    "http.proxySupport": "override",
    "http.noProxy": [
        "localhost",
        "127.0.0.1",
        "::1",
        "192.168.5.11",
        "192.168.5.17"
    ]
}
```

### 为什么必须单独设置 Remote Proxy？

Windows 本机：

```text
127.0.0.1:7892
```

代表 Windows 上的 Digilink。

但 Remote SSH 后 Codex 实际运行在 Jetson：

```text
Jetson 的 127.0.0.1 ≠ Windows 的 127.0.0.1
```

如果 Codex 被注入：

```text
HTTP_PROXY=http://127.0.0.1:7892
HTTPS_PROXY=http://127.0.0.1:7892
```

它就会错误连接 Jetson 自己，从而出现：

```text
token_exchange_failed
error sending request for url
https://auth.openai.com/oauth/token
```

正确应为：

```text
HTTP_PROXY=http://192.168.5.17:7892
HTTPS_PROXY=http://192.168.5.17:7892
```

---

## 6. 故障检查命令

### Jetson 能否访问电脑代理

```bash
nc -vz 192.168.5.17 7892
```

### 指定代理测试

```bash
curl -I \
  -x http://192.168.5.17:7892 \
  https://www.google.com
```

### 查看 Codex 进程

```bash
pgrep -af codex
```

### 查看 Codex 实际代理变量

```bash
PID=$(pgrep -f '/codex .*app-server' | head -n1)

tr '\0' '\n' < /proc/$PID/environ \
  | grep -iE '^(http|https|all|no)_proxy='
```

正确结果应指向：

```text
192.168.5.17:7892
```

不能是：

```text
127.0.0.1:7892
```

---

# 7. 双系统切换到 Ubuntu 后

需要重新检查，**不能默认 Windows 的配置直接可用**。

原因有两个：

1. Ubuntu 中需要有自己的代理软件，并开放一个可供 Jetson 访问的 HTTP/Mixed 代理端口。
2. Ubuntu 获得的局域网 IP 可能与 Windows 不同。

Ubuntu 主机先执行：

```bash
ip -br addr
ip route
```

确认主机 IP，例如：

```text
192.168.5.20
```

再确认代理监听端口，例如：

```text
7892
```

那么 Jetson 和 VS Code Remote Settings 中需要改成：

```text
http://192.168.5.20:7892
```

### IP 会不会一定变化？

不一定。

双系统通常使用同一块物理网卡，MAC 地址通常相同，所以路由器 **可能** 继续分配同一个 IP，但 DHCP 并不保证永远不变。

因此推荐：

- 在路由器中给主机做 DHCP 静态租约 / 地址保留
- 给 Jetson 也做地址保留

例如长期固定：

```text
主机：192.168.5.17
Jetson：192.168.5.11
```

这样 Windows / Ubuntu 切换后，维护成本最低。

---

## 8. 最终记忆

```text
Windows 本机 VS Code：
127.0.0.1:7892

Jetson / Remote Codex：
192.168.5.17:7892
```

一句话：

> 本机程序访问本机代理用 `127.0.0.1`；Jetson 访问电脑代理必须使用电脑的局域网 IP。
