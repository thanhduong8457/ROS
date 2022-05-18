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

With z = -375 we have rmax = 1
With z = -376 we have rmax = 2
With z = -377 we have rmax = 3
With z = -378 we have rmax = 5
With z = -379 we have rmax = 6
With z = -380 we have rmax = 8
With z = -381 we have rmax = 9
With z = -382 we have rmax = 10
With z = -383 we have rmax = 12
With z = -384 we have rmax = 13
With z = -385 we have rmax = 15
With z = -386 we have rmax = 16
With z = -387 we have rmax = 18
With z = -388 we have rmax = 19
With z = -389 we have rmax = 21
With z = -390 we have rmax = 22
With z = -391 we have rmax = 24
With z = -392 we have rmax = 25
With z = -393 we have rmax = 27
With z = -394 we have rmax = 29
With z = -395 we have rmax = 30
With z = -396 we have rmax = 32
With z = -397 we have rmax = 33
With z = -398 we have rmax = 35
With z = -399 we have rmax = 37
With z = -400 we have rmax = 38
With z = -401 we have rmax = 40
With z = -402 we have rmax = 42
With z = -403 we have rmax = 44
With z = -404 we have rmax = 45
With z = -405 we have rmax = 47
With z = -406 we have rmax = 49
With z = -407 we have rmax = 51
With z = -408 we have rmax = 52
With z = -409 we have rmax = 54
With z = -410 we have rmax = 56
With z = -411 we have rmax = 58
With z = -412 we have rmax = 60
With z = -413 we have rmax = 62
With z = -414 we have rmax = 64
With z = -415 we have rmax = 66
With z = -416 we have rmax = 68
With z = -417 we have rmax = 70
With z = -418 we have rmax = 72
With z = -419 we have rmax = 74
With z = -420 we have rmax = 76
With z = -421 we have rmax = 78
With z = -422 we have rmax = 80
With z = -423 we have rmax = 82
With z = -424 we have rmax = 85
With z = -425 we have rmax = 87
With z = -426 we have rmax = 89
With z = -427 we have rmax = 91
With z = -428 we have rmax = 94
With z = -429 we have rmax = 96
With z = -430 we have rmax = 99
With z = -431 we have rmax = 101
With z = -432 we have rmax = 103
With z = -433 we have rmax = 106
With z = -434 we have rmax = 109
With z = -435 we have rmax = 111
With z = -436 we have rmax = 114
With z = -437 we have rmax = 117
With z = -438 we have rmax = 119
With z = -439 we have rmax = 122
With z = -440 we have rmax = 125
With z = -441 we have rmax = 128
With z = -442 we have rmax = 131
With z = -443 we have rmax = 134
With z = -444 we have rmax = 137
With z = -445 we have rmax = 141
With z = -446 we have rmax = 144
With z = -447 we have rmax = 147
With z = -448 we have rmax = 151
With z = -449 we have rmax = 155
With z = -450 we have rmax = 158
With z = -451 we have rmax = 162
With z = -452 we have rmax = 166
With z = -453 we have rmax = 171
With z = -454 we have rmax = 175
With z = -455 we have rmax = 180
With z = -456 we have rmax = 184
With z = -457 we have rmax = 190
With z = -458 we have rmax = 195
With z = -459 we have rmax = 201
With z = -460 we have rmax = 208
With z = -461 we have rmax = 215
With z = -462 we have rmax = 223
With z = -463 we have rmax = 232
With z = -464 we have rmax = 245
With z = -465 we have rmax = 372
With z = -466 we have rmax = 371
With z = -467 we have rmax = 371
With z = -468 we have rmax = 370
With z = -469 we have rmax = 369
With z = -470 we have rmax = 368
With z = -471 we have rmax = 367
With z = -472 we have rmax = 367
With z = -473 we have rmax = 366
With z = -474 we have rmax = 365
With z = -475 we have rmax = 364
With z = -476 we have rmax = 364
With z = -477 we have rmax = 363
With z = -478 we have rmax = 362
With z = -479 we have rmax = 361
With z = -480 we have rmax = 360

