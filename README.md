# Zephyr RTOS 下位机框架
## 该项目希望以设备树形式描述机器人
### 最终的效果将如example.dts所示
The structure is as shown below
![structure](https://mirrors.sustech.edu.cn/git/12411711/mambo/-/blob/main/structure.png "Structure")
## 如何在我的开发板上运行？（以Robomaster Developement Board C为例）
### Initialization

First zephyr SDKs are needed.
```shell
sudo apt update
sudo apt upgrade
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```
### You should have a virtual environment if Python tells you to do so

可以提前更换PIP源:
```shell
pip install --upgrade pip --index-url https://mirrors.sustech.edu.cn/pypi/web/simple
pip config set global.index-url https://mirrors.sustech.edu.cn/pypi/web/simple
```

```shell
pip install west
```

Then initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://mirrors.sustech.edu.cn/git/12411711/mambo --mr master my-workspace
# update Zephyr modules
cd my-workspace
west update
```

Then export a Zephyr CMake package. This allows CMake to automatically load boilerplate code required for building Zephyr applications.
```shell
west zephyr-export
pip install -r ./zephyr/scripts/requirements.txt
west sdk install
```

### Building and running

To build the application, run the following command:

```shell
cd mambo
west build -b robomaster_board_c samples\motor\dji_m3508_demo
```

`$BOARD` is the target board. Here you can use `robomaster_board_c`

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build -b $BOARD samples\motor\dji_m3508_demo
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```
默认为cmsis-dap, 如果使用stlink请加上`--runner stlink`, Jlink则为`--runner jlink`
If everything goes well, the LED on the board should be blinking

Then you can have your motors running!

另外，VSC_sample_configs下有.vscode的示例配置文件

详细的文档请参考`Documents`文件夹

```
好无聊逗逗梅总吧
    嘬嘬嘬𐃆 ˒˒ ͏                               
͏
͏                             ╱|、
                            (˚ˎ 。7 
                            |、˜ 〵 
                            じしˍ,_)ノ
```

