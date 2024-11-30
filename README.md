##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%# ROS2
#=== link include with ROS.jgp picture above ===#
	https://docs.ros.org/en/galactic/index.html
	https://roboticsbackend.com/category/ros2/

#list of robot pakages: 
	https://moveit.ros.org/robots/
#link of book:
	https://vn1lib.org/s/ROS 
# link yootube about moveit
	https://www.youtube.com/channel/UCofPudSKrNzT3vccJD6VDyA/videos

#ROS melodic:
	http://wiki.ros.org/ROS/Tutorials
#Tutorials:
	http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
#moveit:
	http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%# Ubuntu Command
- Enable rviz to display on X server:
	export GAZEBO_IP=127.0.0.1
	export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0	
	
	export DISPLAY=127.0.0.1:0.0
	export LIBGL_ALWAYS_INDIRECT=0
	
##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
- to reboot WSL, open powershell with admin and run this command:
	Get-Service LxssManager | Restart-Service

- power command help me successfully upgrade WSL 20.04 from WSL 18.04
	sudo apt-get purge snapd
##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
lsb_release -a
sudo apt update
sudo apt list --upgradable
sudo apt upgrade
sudo apt --purge autoremove
sudo apt install update-manager-core
sudo do-release-upgrade
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
Động học Robot
	Nghiên cứu bài toán động học thuận và động học ngược
	Nghiên cứu thông số, cấu hình robot Motoman UP6
	Xây dựng và tính toán động học thuận bằng phương pháp DenavitHartenberg (DH)
	Tính toán động học ngược bằng phương pháp đại số
	Tính toán môi trường làm việc thích hợp dựa trên các cơ cấu của robot
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
Điều khiển và hoạch định quỹ đạo
	- Tìm hiểu các phương pháp hoạch định quỹ đạo
	- Xây dựng giải thuật hoạch định quỹ đạo cho robot
	- Lập trình chuyển giải thuật sang chương trình trên máy tính
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
Nghiên cứu bộ thông số kết cấu cơ khí và thông số servo motor:
	- Nắm được thông số về kết cấu cơ khí bao gồm kích thước các khâu và khối lượng của khâu
	- Nắm được thông số của servo motor Yaskawa Sigma2 bao gồm thông số cuộn dây, dòng điện và điện áp định mức, công suất motor
	+ Tìm kiếm tài liệu của robot thông qua Internet để tìm ra thông số về cơ cấu các khâu robot UP6
	+ Tìm hiểu thông số motor thông qua nhãn chứa thông số motor và tiến hành đo đạc để tìm ra bảng thông số thích hợp.
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#	
Nghiên cứu Driver servo motor HIWIN D1 và điều khiển servo Yaskawa sử dụng diver HIWIN
	- Kết hợp điều khiển thành công Driver HIWIN và Yaskawa motor
	+ Đọc và nghiên cứu tài liệu từ nhà sản xuất HIWIN và cài đặt thông số để Driver có thể điều khiển motor chính xác
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
Tìm hiểu lý thuyết Robot bao gồm động học robot, hoạch định quỹ đạo cho robot
	- Nắm vững lý thuyết robot bao gồm động học robot, hoạch định quỹ đạo cho robot
	+ Nghiên cứu tài liệu về lý thuyết robot thông qua Thầy hướng dẫn và các tài liệu nguồn từ Internet
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#	
Xây dựng giải thuật điều khiển và thi công Board mạch điều khiển dựa trên ARM® Cortex®-M4-based STM32F4 MCU ,Raspberry Pi 3 ARM Cortex-A53 CPU
	- Nhúng thành công giải thuật điều khiển Robot UP6 lên ARM® Cortex®-M4- based STM32F4 MCU, Raspberry Pi 3 ARM Cortex-A53 CPU
	+ Nghiên cứu và tìm hiểu kiến trúc ARM® Cortex®-M4-based STM32F4 MCU, Raspberry Pi 3 ARM Cortex-A53 CPU và sử dụng ngôn ngữ lập trình C ,C# đểnhúng giải thuật điều khiển robot lên Board điều khiển
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#	
Xây dựng phần mềm giao diện người máy để có thể tương tác điều khiển và lập trình cho robot sử dụng ngôn ngữ C# và Java cài đặt trên PC vàAndroid device
	- Xây dựng hoàn thiện giao diện người máy để ngừoi dùng có thể tương tác lập trình điều khiển robot thông qua PC ,Android device
	+ Vận dụng ngôn ngữ C# và Java để xây dựng phần mềm giao diện tương tác người máy để điều khiển Robot
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#

1. gioi thieu ve Robot(Delta)
2. Mo tả Hoạt động chung của Robot Delta
3. bài toán động học thuận của Robot Delta
4. bài toán động học ngược của Robot Delta
5. bài toán động học của Robot Delta
6. giới thiệu về Software Robot Operating System(ROS)
	node
	topic
	msg
	pub
	sub
	...
7.  giới thiệu về MoveIt(Rviz)
	mô phỏng robot
8. Unified Robot Description Format (URDF)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
https://www.udemy.com/course/mastering-rtos-hands-on-with-freertos-arduino-and-stm32fx/learn/lecture/25694576#overview 
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
DRV8825 x3
NEMA 23 Stepper Motor

ff = 346.4     // base
rf = 200       // Base arm
re = 465       // endeffector arm
ee = 86.5      // endeffector 

UART
https://www.youtube.com/watch?v=c1_kVj1KJAg 

UART DMA
https://www.youtube.com/watch?v=S7C-NrHRtDY

{"theta1":"400","theta2":"400","theta3":"400","gripper":"1"}<LF>
{"theta1":"100","theta2":"200","theta3":"300","gripper":"1"}<LF>
{"theta1":"0","theta2":"0","theta3":"0","gripper":"1"}<LF>

A B C D 

list of points: [A, B, C, D, E, F, ...]

Current point go to each point

PA3 -> D14
PA2 -> D15

check the GPU memory with tegrastats
sudo tegrastats
