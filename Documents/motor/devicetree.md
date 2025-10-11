# 电机驱动 Device Tree 配置
### DJI Motor Device Tree Properties
- `compatible`: 必须是"dji,motor"
  - `is_m3508`: 电机型号，可选
  - `is_m2006`: 电机型号，可选
  - `is_gm6020`: 电机型号，可选
- `id`: 电机编号
- `rx_id`: 电机接收ID
- `tx_id`: 电机发送ID
- `gear_ratio`: 减速比（需要乘100）
- `status` = "okay": 启用电机
- `can_channel`: CAN总线节点
- `controllers`: PID控制器节点，是从前到后的串联环，暂时不支持MIT模式
- `capabilities`: 指定PID环对应的控制对象，可选
  - "angle": 角度控制
  - "speed": 速度控制
```dts
\ {
	motor1 {
		compatible = "dji,motor";
        is_m3508;
		id = <1>;
    	rx_id = <0x201>;
        tx_id = <0x200>;
		gear_ratio = <1920>;
		status = "okay";
		can_channel = <&canbus1>;
		controllers = <&angle_pid_1 &speed_pid_1>;
		capabilities = "angle", "speed";
	};
};
```
#### 请注意，必须存在对应capabilities属性，否则电机驱动只能使用力矩控制
### 达妙电机 Device Tree Properties
- `compatible`: 必须是"dm,motor"
- `id`: 电机编号
- `tx_id`: 电机发送ID
- `rx_id`: 电机接收ID
- `can_channel`: CAN总线节点
- `gear_ratio`: 减速比（需要乘100）
- `controllers`: PID控制器节点，按顺序指定各控制环
- `capabilities`: 指定控制模式，可选：
  - "mit": MIT力矩控制
  - "pv": 位置-速度双环控制
  - "vo": 速度-电流双环控制
- `v_max`: 最大速度限制（需要乘10）
- `p_max`: 最大角度限制（需要乘10）
- `t_max`: 最大扭矩限制（需要乘10）
```dts
/ {
	motor1: motor1{
		compatible = "dm,motor";
		id = <24>;
		rx_id = <24>;
		tx_id = <86>;
		gear_ratio = <100>;
		status = "okay";
		can_channel = <&canbus1>;
		controllers = <&mit_pid_0>;
		capabilities = "mit";
		p_max = <125>;
		v_max = <450>;
		t_max = <100>;
	};
};
```

### PID Device Tree Properties
- `#controller-cells`: 必须是<0>
- `compatible`: 必须是"pid,single"
- `k_p`: 比例系数（乘100）
- `k_i`: 积分系数（乘10）
- `k_d`: 微分系数（乘10）
```dts
\ {
	pid {
		angle_pid_1: angle_pid_1 {
			#controller-cells = <0>;
			compatible = "pid,single";
			k_p = <700>;
			k_i = <3>;
			k_d = <800>;
		};
		speed_pid_1: speed_pid_1 {
			#controller-cells = <0>;
			compatible = "pid,single";
			k_p = <880>;
			k_i = <20>;
			k_d = <20>;
		};
	};
};
```
