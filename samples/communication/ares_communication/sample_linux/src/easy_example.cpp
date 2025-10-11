#include "ares_protocol.hpp"
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>

// 简单的同步帧回调，收到数据直接打印
void sync_handler(uint16_t data_id, const uint8_t *data, size_t len)
{
	printf("[同步帧] DataID=%u, 长度=%zu, 内容=", data_id, len);
	for (size_t i = 0; i < len; ++i) {
		printf("%02X ", data[i]);
	}
	printf("\n");
}

// 简单的执行帧回调，收到数据直接打印
uint32_t exec_handler(uint16_t func_id, uint32_t arg1, uint32_t arg2, uint32_t arg3,
		      uint8_t request_id)
{
	printf("[执行帧] FuncID=%u, Arg1=%u, Arg2=%u, Arg3=%u, ReqID=%u\n", func_id, arg1, arg2,
	       arg3, request_id);
	// 返回一个简单的结果
	return arg1 + arg2 + arg3;
}

int main()
{
	ares::Protocol proto;

	// 注册回调
	proto.register_sync_callback(sync_handler);
	proto.register_exec_callback(exec_handler);

	printf("正在连接设备...\n");
	if (!proto.connect()) {
		printf("连接失败！\n");
		return 1;
	}
	printf("设备已连接。\n");

	while (true) {
		printf("\n请选择操作：\n");
		printf("1. 发送同步帧(sync)\n");
		printf("2. 发送执行帧(exec)\n");
		printf("0. 退出\n");
		printf("输入选项: ");
		int op = 0;
		if (!(std::cin >> op)) {
			std::cin.clear();
			std::cin.ignore(1024, '\n');
			continue;
		}
		if (op == 0)
			break;
		if (op == 1) {
			// 发送同步帧
			uint16_t data_id;
			size_t len;
			printf("请输入DataID (uint16): ");
			std::cin >> data_id;
			printf("请输入数据长度 (1~64): ");
			std::cin >> len;
			if (len == 0 || len > 64) {
				printf("长度非法！\n");
				continue;
			}
			std::vector<uint8_t> buf(len);
			printf("请输入数据内容（以空格分隔，每个字节十进制或十六进制0x开头）：\n");
			for (size_t i = 0; i < len; ++i) {
				unsigned int val;
				std::cin >> std::hex >> val;
				buf[i] = static_cast<uint8_t>(val);
			}
			if (proto.send_sync(data_id, buf.data(), len)) {
				printf("同步帧已发送。\n");
			} else {
				printf("发送失败！\n");
			}
		} else if (op == 2) {
			// 发送执行帧
			uint16_t func_id;
			uint32_t arg1, arg2, arg3;
			uint8_t req_id;
			printf("请输入FuncID (uint16): ");
			std::cin >> func_id;
			printf("请输入Arg1 (uint32): ");
			std::cin >> arg1;
			printf("请输入Arg2 (uint32): ");
			std::cin >> arg2;
			printf("请输入Arg3 (uint32): ");
			std::cin >> arg3;
			printf("请输入RequestID (uint8): ");
			unsigned int tmp;
			std::cin >> tmp;
			req_id = static_cast<uint8_t>(tmp);
			if (proto.send_exec(func_id, arg1, arg2, arg3, req_id)) {
				printf("执行帧已发送。\n");
			} else {
				printf("发送失败！\n");
			}
		} else {
			printf("无效选项。\n");
		}
		// 清理输入缓冲区
		std::cin.ignore(1024, '\n');
	}

	printf("正在断开连接...\n");
	proto.disconnect();
	printf("程序结束。\n");
	return 0;
}
