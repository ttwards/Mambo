#pragma once
#include <cstdint>
#include <functional>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono> // 需要包含 chrono
#include <libusb-1.0/libusb.h>

namespace ares
{

constexpr uint16_t EXEC_FRAME_HEAD = 0xCAFE;
constexpr uint16_t EXEC_REPLY_HEAD = 0xC0DE;
constexpr uint16_t SYNC_FRAME_HEAD = 0x5A5A;
constexpr uint16_t ERROR_FRAME_HEAD = 0xCADE;
constexpr uint16_t HEARTBEAT_ERROR_CODE = (1 << 8); // BIT(8)
constexpr uint8_t HEARTBEAT_REQUEST_ID = 0xFF;
constexpr std::chrono::milliseconds HEARTBEAT_INTERVAL{3}; // 10ms
constexpr std::chrono::milliseconds RECONNECT_DELAY{500};  // 自动重连尝试间隔

constexpr size_t USB_FS_MPS = 64;
constexpr uint16_t VID = 0x1209;
constexpr uint16_t PID = 0x0001;
constexpr uint8_t EP_IN = 0x81;
constexpr uint8_t EP_OUT = 0x01;
constexpr int USB_TIMEOUT_MS = 1000; // USB 操作超时时间

// 执行帧
struct ExecFrame {
	uint16_t head;
	uint16_t func_id;
	uint32_t arg1;
	uint32_t arg2;
	uint32_t arg3;
	uint8_t request_id;
	uint8_t reserved;
} __attribute__((packed));

// 执行返回帧
struct ExecReplyFrame {
	uint16_t head;
	uint16_t func_id;
	uint32_t value;
	uint8_t request_id;
	uint8_t reserved;
} __attribute__((packed));

// 数据帧
struct SyncFrame {
	uint16_t head;
	uint16_t data_id;
	uint8_t data[USB_FS_MPS - 4]; // 4 bytes: head + data_id
} __attribute__((packed));

// 错误帧
struct ErrorFrame {
	uint16_t head;
	uint8_t request_id;
	uint8_t reserved;
	uint16_t error_code;
} __attribute__((packed));

// --- 回调类型定义 ---
using SyncCallback = std::function<void(uint16_t data_id, const uint8_t *data, size_t len)>;
using ExecCallback = std::function<uint32_t(uint16_t func_id, uint32_t arg1, uint32_t arg2,
					    uint32_t arg3, uint8_t request_id)>;
// {{ edit_1: 添加 ExecReply 和 Error 回调类型 }}
using ExecReplyCallback = std::function<void(uint16_t func_id, uint32_t value, uint8_t request_id)>;
using ErrorCallback = std::function<void(uint8_t request_id_or_data_id, uint16_t error_code)>;

class Protocol
{
      public:
	Protocol();
	~Protocol();

	// 初始化并连接 USB 设备
	bool connect();
	// 断开连接
	void disconnect();
	// 检查连接状态
	bool is_connected() const;

	// 注册同步帧回调
	void register_sync_callback(SyncCallback cb);

	// 注册执行帧回调 (下位机使用)
	void register_exec_callback(ExecCallback cb);
	// {{ edit_2: 添加新的回调注册函数声明 }}
	void register_exec_reply_callback(ExecReplyCallback cb);
	void register_error_callback(ErrorCallback cb);

	// 处理接收到的数据 (现在设为私有，由内部读取线程调用)
	// void on_receive(const uint8_t* buf, size_t len);

	// 发送执行帧 (上位机使用)
	bool send_exec(uint16_t func_id, uint32_t arg1, uint32_t arg2, uint32_t arg3,
		       uint8_t request_id);

	// 发送同步帧 (下位机使用)
	bool send_sync(uint16_t data_id, const uint8_t *data, size_t len);

	// 获取同步帧数据缓冲区指针 (这个接口可能不再需要，取决于具体应用)
	// uint8_t* get_sync_data_buffer(uint16_t data_id);

	// 移除 set_send_function，使用内部 USB 发送
	// void set_send_function(std::function<void(const uint8_t*, size_t)> send_func);

      private:
	// 处理接收数据的内部方法
	void on_receive_internal(const uint8_t *buf, size_t len);
	// USB 底层写操作
	bool usb_write(const uint8_t *data, size_t len);
	// USB 读取线程函数
	void usb_read_loop();
	void heartbeat_loop(); // 新增：心跳线程函数

	// {{ edit_1: 添加缺失的成员变量声明 }}
	libusb_context *usb_ctx_ = nullptr;
	libusb_device_handle *dev_handle_ = nullptr;
	std::thread read_thread_;
	std::thread heartbeat_thread_;
	std::atomic<bool> running_{false}; // 控制线程运行状态
	std::atomic<bool> stop_{false};    // 停止 USB 读取线程

	// 回调函数
	SyncCallback sync_cb_ = nullptr;
	ExecCallback exec_cb_ = nullptr;
	ExecReplyCallback exec_reply_cb_ = nullptr;
	ErrorCallback error_cb_ = nullptr;

	// 辅助函数
	std::string get_current_timestamp(); // 获取当前时间戳字符串

	// 新增：用于同步 USB 读写的互斥锁
	std::mutex usb_mutex_;
};

} // namespace ares
