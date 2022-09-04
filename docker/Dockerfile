########## Pull ##########
FROM ros:noetic
########## Non-interactive ##########
ENV DEBIAN_FRONTEND=noninteractive
########## Common tool ##########
RUN apt-get update && \
	apt-get install -y \
		vim \
		wget \
		unzip \
		git \
        python-tk
########## ROS setup ##########
RUN mkdir -p ~/catkin_ws/src && \
	cd ~/catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make" && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=~/catkin_ws" >> ~/.bashrc
## cmk
RUN echo 'function cmk(){ \n\
		lastpwd=$OLDPWD \n\
		cpath=$(pwd) \n\
		cd ~/catkin_ws \n\
		catkin_make $@ \n\
		cd $cpath \n\
		OLDPWD=$lastpwd \n\
        }' >> ~/.bashrc
########## Cache busting ##########
ARG CACHEBUST=1
########## msg_conversion ##########
# RUN apt-get update && \
#     apt-get install -y \
# 		libpcl-dev \
# 		ros-noetic-pcl-conversions \
# 		ros-noetic-image-geometry \
# 		ros-noetic-cv-bridge \
# 		ros-noetic-rviz && \
# 	cd ~/catkin_ws/src && \
# 	git clone https://github.com/ozakiryota/msg_conversion.git && \
# 	cd ~/catkin_ws/ && \
# 	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
########## Initial position ##########
WORKDIR /root/catkin_ws/src/msg_conversion