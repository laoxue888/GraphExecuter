
---

# 前言

配置开发环境

# Windows

```shell
conda create -n graph_executer python=3.12
conda activate graph_executer

cd graph_executer
pip install -r .\requirements.txt

git clone https://github.com/laoxue888/NodeGraphQt.git
cd NodeGraphQt
pip install -e .

pip install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu124
```

# Linux

❇️配置开发环境

```shell
sudo apt update
apt install python3-pip -y
apt-get install portaudio19-dev -y
apt-get install x11-xserver-utils
apt install libxcb* -y

# 使用清华源下载
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple --break-system-packages


pip install jpeg4py -i https://pypi.tuna.tsinghua.edu.cn/simple --break-system-packages
sudo apt-get install -y libturbojpeg
```

❇️运行

```shell
cd ros2_docker_ws
source install/setup.bash

source install/setup.bash
cd src/robot_controller_graph
python3 main.py

```

$$  $$