
---

# 前言

配置开发环境

# Windows

```shell
conda create -n graph_executer python=3.12
conda activate graph_executer

cd graph_executer
pip install -r requirements_linux.txt
# pip install -r requirements_windows.txt

git clone https://github.com/laoxue888/NodeGraphQt.git
cd NodeGraphQt
conda activate graph_executer
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
conda activate graph_executer
python3 main.py
```

$$ P\left(x,y|z,u\right)=\frac{P\left(z,u|x,y\right)P\left(x,y\right)}{P\left(z,u\right)}\propto\sqrt{\frac{\left(z,u|x,y\right)}{\mathbb{R}^{\prime}/z}}\sum_{\widetilde{v}=z}\left(\begin{array}{c}{{}}\\ {{}}\\ {{}}\\ {{}}\\ {{}}\\ {{}}&{{}}\end{array}\right)\ . $$
