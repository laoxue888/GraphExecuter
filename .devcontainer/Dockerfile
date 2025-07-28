	# 创建容器所需要的docker镜像，这里是ESP-IDF的镜像
	FROM docker.1ms.run/ubuntu:22.04
	# 执行命令时，不需要交互，直接运行
	ARG DEBIAN_FRONTEND=nointeractive

	RUN apt-get update \
	  && apt install -y -q \
	  cmake \
	  git \
	  python3 \
	  python3-pip \
	  wget

	ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

	CMD ["/bin/bash"]
