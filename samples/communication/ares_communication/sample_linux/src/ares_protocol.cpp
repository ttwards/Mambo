#include "ares_protocol.hpp"
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <vector>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <stdexcept>

namespace ares
{

// 辅助函数：获取当前时间戳
std::string Protocol::get_current_timestamp()
{
	auto now = std::chrono::system_clock::now();
	auto now_c = std::chrono::system_clock::to_time_t(now);
	auto now_ms =
		std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) %
		1000;

	std::stringstream ss;
	struct tm tm_buf;
	time_t tt = now_c;
#ifdef _WIN32
	struct tm ptm_s;
	localtime_s(&ptm_s, &tt);
	struct tm *ptm = &ptm_s;
#else
	struct tm *ptm = localtime_r(&tt, &tm_buf); // POSIX
#endif

	if (ptm) {
		ss << std::put_time(ptm, "%Y-%m-%d %H:%M:%S");
		ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
	} else {
		ss << "TimestampError";
	}
	return ss.str();
}

Protocol::Protocol() = default;

Protocol::~Protocol()
{
	disconnect();
}

bool Protocol::connect()
{
	if (dev_handle_) {
		return true;
	}
	if (stop_) {
		return false;
	}

	int r = libusb_init(&usb_ctx_);
	if (r < 0) {
		std::cerr << get_current_timestamp()
			  << " libusb_init Error: " << libusb_error_name(r) << std::endl;
		return false;
	}

	// libusb_set_option(usb_ctx_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);

	dev_handle_ = libusb_open_device_with_vid_pid(usb_ctx_, VID, PID);
	if (!dev_handle_) {
		std::cerr << get_current_timestamp()
			  << " Error finding/opening device VID=" << std::hex << VID
			  << " PID=" << PID << std::dec << std::endl;
		libusb_exit(usb_ctx_);
		usb_ctx_ = nullptr;
		return false;
	}

	libusb_detach_kernel_driver(dev_handle_, 0);

	r = libusb_claim_interface(dev_handle_, 0);
	if (r < 0) {
		std::cerr << get_current_timestamp()
			  << " libusb_claim_interface Error: " << libusb_error_name(r) << std::endl;
		libusb_close(dev_handle_);
		dev_handle_ = nullptr;
		libusb_exit(usb_ctx_);
		usb_ctx_ = nullptr;
		return false;
	}

	std::cout << get_current_timestamp() << " USB Device connected successfully." << std::endl;
	running_ = true;
	stop_ = false; // 允许线程运行
	read_thread_ = std::thread(&Protocol::usb_read_loop, this);
	heartbeat_thread_ = std::thread(&Protocol::heartbeat_loop, this);

	return true;
}

void Protocol::disconnect()
{
	if (stop_) { // 防止重复调用
		return;
	}
	stop_ = true; // 命令所有线程停止
	running_ = false;

	if (heartbeat_thread_.joinable()) {
		heartbeat_thread_.join();
	}
	if (read_thread_.joinable()) {
		read_thread_.join();
	}

	if (dev_handle_) {
		libusb_release_interface(dev_handle_, 0);
		libusb_close(dev_handle_);
		dev_handle_ = nullptr;
	}
	if (usb_ctx_) {
		libusb_exit(usb_ctx_);
		usb_ctx_ = nullptr;
	}
	std::cout << get_current_timestamp() << " USB Device disconnected." << std::endl;
}

bool Protocol::is_connected() const
{
	return dev_handle_ != nullptr && running_;
}

void Protocol::register_sync_callback(SyncCallback cb)
{
	sync_cb_ = std::move(cb);
}

void Protocol::register_exec_callback(ExecCallback cb)
{
	exec_cb_ = std::move(cb);
}

void Protocol::register_exec_reply_callback(ExecReplyCallback cb)
{
	exec_reply_cb_ = std::move(cb);
}

void Protocol::register_error_callback(ErrorCallback cb)
{
	error_cb_ = std::move(cb);
}

bool Protocol::usb_write(const uint8_t *data, size_t len)
{
	if (!is_connected())
		return false;

	int actual_length = 0;
	int r;
	{
		std::lock_guard<std::mutex> lock(usb_mutex_);
		if (!dev_handle_)
			return false;
		r = libusb_bulk_transfer(dev_handle_, EP_OUT, const_cast<uint8_t *>(data), len,
					 &actual_length, USB_TIMEOUT_MS);
	}

	if (r == 0 && actual_length == static_cast<int>(len)) {
		return true;
	} else {
		std::cerr << get_current_timestamp() << " USB Write Error: " << libusb_error_name(r)
			  << ", transferred: " << actual_length << "/" << len << std::endl;
		if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_IO || r == LIBUSB_ERROR_PIPE) {
			std::cerr << get_current_timestamp()
				  << " USB connection lost during write. Triggering reconnect."
				  << std::endl;
			running_ = false; // 触发重连
		}
		return false;
	}
}

void Protocol::usb_read_loop()
{
	uint8_t buffer[USB_FS_MPS];
	while (!stop_) {
		if (!running_) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		int actual_length = 0;
		int r = libusb_bulk_transfer(dev_handle_, EP_IN, buffer, sizeof(buffer),
					     &actual_length, 500);

		if (r == 0 && actual_length > 0) {
			on_receive_internal(buffer, actual_length);
		} else if (r != LIBUSB_ERROR_TIMEOUT && r != LIBUSB_TRANSFER_CANCELLED) {
			std::cerr << get_current_timestamp()
				  << " USB Read Error: " << libusb_error_name(r) << std::endl;
			if (r == LIBUSB_ERROR_NO_DEVICE || r == LIBUSB_ERROR_IO ||
			    r == LIBUSB_ERROR_PIPE) {
				std::cerr
					<< get_current_timestamp()
					<< " USB connection lost during read. Triggering reconnect."
					<< std::endl;
				running_ = false; // 信号连接丢失
			}
		}
	}
	std::cout << get_current_timestamp() << " USB read loop finished." << std::endl;
}

void Protocol::on_receive_internal(const uint8_t *buf, size_t len)
{
	if (len < 2) {
		std::cerr << get_current_timestamp() << " Error: Received runt frame. Len=" << len
			  << std::endl;
		return;
	}

	uint16_t head = ntohs(*(reinterpret_cast<const uint16_t *>(buf)));

	switch (head) {
	case SYNC_FRAME_HEAD: {
		if (len >= offsetof(SyncFrame, data)) {
			const auto *frame = reinterpret_cast<const SyncFrame *>(buf);
			if (sync_cb_) {
				try {
					sync_cb_(frame->data_id, frame->data,
						 len - offsetof(SyncFrame, data));
				} catch (const std::exception &e) {
					std::cerr << get_current_timestamp()
						  << " Exception in sync_cb_: " << e.what()
						  << std::endl;
				} catch (...) {
					std::cerr << get_current_timestamp()
						  << " Unknown exception in sync_cb_" << std::endl;
				}
			}
		} else {
			std::cerr << get_current_timestamp()
				  << " Error: Received truncated Sync frame. Len=" << len
				  << std::endl;
		}
		break;
	}
	case EXEC_FRAME_HEAD: {
		if (len == sizeof(ExecFrame)) {
			const auto *frame = reinterpret_cast<const ExecFrame *>(buf);
			if (frame->request_id != HEARTBEAT_REQUEST_ID && exec_cb_) {
				try {
					exec_cb_(frame->func_id, frame->arg1, frame->arg2,
						 frame->arg3, frame->request_id);
				} catch (const std::exception &e) {
					std::cerr << get_current_timestamp()
						  << " Exception in exec_cb_: " << e.what()
						  << std::endl;
				} catch (...) {
					std::cerr << get_current_timestamp()
						  << " Unknown exception in exec_cb_" << std::endl;
				}
			}
		} else {
			std::cerr << get_current_timestamp()
				  << " Error: Received Exec frame with incorrect size. Len=" << len
				  << std::endl;
		}
		break;
	}
	case EXEC_REPLY_HEAD: {
		if (len == sizeof(ExecReplyFrame)) {
			const auto *frame = reinterpret_cast<const ExecReplyFrame *>(buf);
			if (frame->request_id != HEARTBEAT_REQUEST_ID && exec_reply_cb_) {
				try {
					exec_reply_cb_(frame->func_id, frame->value,
						       frame->request_id);
				} catch (const std::exception &e) {
					std::cerr << get_current_timestamp()
						  << " Exception in exec_reply_cb_: " << e.what()
						  << std::endl;
				} catch (...) {
					std::cerr << get_current_timestamp()
						  << " Unknown exception in exec_reply_cb_"
						  << std::endl;
				}
			}
		} else {
			std::cerr << get_current_timestamp()
				  << " Error: Received Exec Reply frame with incorrect size. Len="
				  << len << std::endl;
		}
		break;
	}
	case ERROR_FRAME_HEAD: {
		if (len == sizeof(ErrorFrame)) {
			const auto *frame = reinterpret_cast<const ErrorFrame *>(buf);
			if (frame->request_id != HEARTBEAT_REQUEST_ID && error_cb_) {
				try {
					error_cb_(frame->request_id, frame->error_code);
				} catch (const std::exception &e) {
					std::cerr << get_current_timestamp()
						  << " Exception in error_cb_: " << e.what()
						  << std::endl;
				} catch (...) {
					std::cerr << get_current_timestamp()
						  << " Unknown exception in error_cb_" << std::endl;
				}
			}
		} else {
			std::cerr << get_current_timestamp()
				  << " Error: Received Error frame with incorrect size. Len=" << len
				  << std::endl;
		}
		break;
	}
	default:
		std::cerr << get_current_timestamp()
			  << " Error: Received frame with unknown head: 0x" << std::hex << head
			  << std::dec << std::endl;
		break;
	}
}

bool Protocol::send_exec(uint16_t func_id, uint32_t arg1, uint32_t arg2, uint32_t arg3,
			 uint8_t request_id)
{
	ExecFrame frame_to_send;
	frame_to_send.head = htons(EXEC_FRAME_HEAD);
	frame_to_send.func_id = func_id;
	frame_to_send.arg1 = arg1;
	frame_to_send.arg2 = arg2;
	frame_to_send.arg3 = arg3;
	frame_to_send.request_id = request_id;
	frame_to_send.reserved = 0;
	return usb_write(reinterpret_cast<uint8_t *>(&frame_to_send), sizeof(frame_to_send));
}

bool Protocol::send_sync(uint16_t data_id, const uint8_t *data, size_t len)
{
	constexpr size_t sync_header_size = offsetof(ares::SyncFrame, data);
	if (len > (USB_FS_MPS - sync_header_size)) {
		std::cerr << "Error: Sync data too large (" << len << " > "
			  << (USB_FS_MPS - sync_header_size) << ")" << std::endl;
		return false;
	}
	std::vector<uint8_t> buffer(sync_header_size + len);
	auto *frame = reinterpret_cast<SyncFrame *>(buffer.data());
	frame->head = htons(SYNC_FRAME_HEAD);
	frame->data_id = data_id;
	memcpy(frame->data, data, len);
	return usb_write(buffer.data(), buffer.size());
}

void Protocol::heartbeat_loop()
{
	while (!stop_) {
		if (running_) {
			// --- 已连接，发送心跳 ---
			ErrorFrame heartbeat_frame;
			heartbeat_frame.head = htons(ERROR_FRAME_HEAD);
			heartbeat_frame.request_id = HEARTBEAT_REQUEST_ID;
			heartbeat_frame.reserved = 0;
			heartbeat_frame.error_code = HEARTBEAT_ERROR_CODE;

			if (!usb_write(reinterpret_cast<uint8_t *>(&heartbeat_frame),
				       sizeof(heartbeat_frame))) {
				std::cerr << get_current_timestamp()
					  << " Failed to send heartbeat. Connection might be lost."
					  << std::endl;
			}
			std::this_thread::sleep_for(HEARTBEAT_INTERVAL);

		} else {
			// --- 已断开，尝试重连 ---
			if (dev_handle_) {
				libusb_release_interface(dev_handle_, 0);
				libusb_close(dev_handle_);
				dev_handle_ = nullptr;
			}

			std::cerr << get_current_timestamp()
				  << " Connection lost. Attempting to reconnect in "
				  << RECONNECT_DELAY.count() << " ms..." << std::endl;
			std::this_thread::sleep_for(RECONNECT_DELAY);

			if (stop_)
				break;

			dev_handle_ = libusb_open_device_with_vid_pid(usb_ctx_, VID, PID);
			if (!dev_handle_) {
				continue; // 失败，在下一次循环中重试
			}

			libusb_detach_kernel_driver(dev_handle_, 0);

			int r = libusb_claim_interface(dev_handle_, 0);
			if (r < 0) {
				std::cerr << get_current_timestamp()
					  << " Reconnect failed: libusb_claim_interface Error: "
					  << libusb_error_name(r) << std::endl;
				libusb_close(dev_handle_);
				dev_handle_ = nullptr;
				continue; // 失败，在下一次循环中重试
			}

			std::cout << get_current_timestamp()
				  << " USB Device reconnected successfully." << std::endl;
			running_ = true; // 恢复运行状态
		}
	}
	std::cout << get_current_timestamp() << " Heartbeat loop finished." << std::endl;
}

} // namespace ares
