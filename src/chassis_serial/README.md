# chassis_serial_driver

通过串口控制全向轮底盘

## 使用指南

安装依赖 `sudo apt install ros-foxy-serial-driver`

更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与电控通讯的串口

启动串口模块 `ros2 launch chassis_serial_driver serial_driver.launch.py`

## 发送和接收

详情请参考 [packet.hpp](include/chassis_serial_driver/packet.hpp)


