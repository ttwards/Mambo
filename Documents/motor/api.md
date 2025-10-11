# 电机驱动API

```c
motor_driver_api {
    int motor_get(const struct device *dev, motor_status_t *status);
    int motor_set(const struct device *dev, motor_status_t *status);
    void motor_control(const struct device *dev, enum motor_cmd cmd);
};
```

```c
/**
 * @brief 电机工作模式
 *
 * MIT: MIT模式
 * PV: 位置-速度控制
 * VO: 速度控制
 * ML_TORQUE: 多环扭矩控制
 * ML_ANGLE: 多环角度控制
 * ML_SPEED: 多环速度控制
 */
enum motor_mode { MIT, PV, VO, ML_TORQUE, ML_ANGLE, ML_SPEED };
```

```c
/**
 * @brief 电机控制命令
 */
enum motor_cmd { ENABLE_MOTOR, DISABLE_MOTOR, SET_ZERO, CLEAR_PID, CLEAR_ERROR };
```

```c
motor_status_t {
    float angle;
    float rpm;
    float torque;
    float temperature; /* Cannot be set in target */
    int round_cnt;

    float speed_limit[2];
    float torque_limit[2];

    enum motor_mode mode;
};
```

## 电机状态获取

`motor_get(const struct device *dev, motor_status_t *status)`: 获取电机当前状态，参数为电机状态结构体。
`motor_set(const struct device *dev, motor_status_t *status)`: 设置电机目标状态，参数为电机状态结构体。
`motor_control(const struct device *dev, enum motor_cmd cmd)`: 控制电机，参数为控制命令。

## 示例

设置电机为多环-扭矩控制模式，并设置目标扭矩为10N*m。

```c
motor_status_t status;
status.mode = ML_TORQUE;
status.torque = 10;
motor_set(motor, &status);
```

获取电机当前的扭矩

```c
motor_status_t status;
motor_get(motor, &status);
LOG_INF("Motor torque: %.2f", status.torque);
```
