# Disturbance Rejection Model Predictive Control for Quadrotor in PX4 SITL simulation

> 这是一个基于模型预测控制（MPC）框架的四旋翼轨迹跟踪控制项目，同时进行了抗干扰，针对PX4 SITL仿真，但是能够很容易移植到真实四旋翼平台，尤其当飞控是PX4时。本工程在Ubuntu20.04+ROS Noetic上进行了测试。
## 参考代码
代码参考了开源工程https://github.com/uzh-rpg/rpg_mpc?tab=readme-ov-file
## 安装依赖
git

MPC tool：使用ACADO toolkit，求解器为qpoases，安装参考：https://acado.github.io/install_linux.html

Eigen：矩阵运算需要用到，要求版本在3.4及以上，安装参考：https://eigen.tuxfamily.org/index.php?title=Main_Page

PX4固件：最好安装1.14.4版本：https://github.com/PX4/PX4-Autopilot/tree/v1.14.4

仿真所需要的依赖详见PX4官方文档：https://docs.px4.io/v1.14/en/sim_gazebo_classic/
## 代码结构
代码采取从底层到顶层的结构：

`mpc_controller`: 生成ACADO底层求解代码

`mpc_wrapper`：调用ACADO的函数，封装成底层wrapper，应该不用动（应该没错误吧），相关类为MpcWrapper

`mpc_ros_application`：针对基于ROS的实现创建的中间层，相关类MpcRosApplication为父类，不能创建实例，该类中创建了MpcWrapper的实例

`mpc_tracking`：实际应用层，大部分修改在这里进行，相关类TrackingMpc继承自MpcRosApplication。要想创建新的基于ROS的代码直接添加新的.cpp文件，创建子类继承MpcRosApplication就行，其他的不用动

`px4_interface`：PX4接口层，参考官方代码https://docs.px4.io/v1.14/en/ros/ ，相关类OffboardMode，若是创建了新的实际应用层的类需要在OffboardMode类中修改

`tracking_node`: main函数入口
## 运行步骤

1. 克隆项目仓库到工作空间下：
   ```
   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
   git clone https://github.com/lw-seu/quadrotor_mpc.git
   cd ~/catkin_ws
2. 编译：
   ```
   catkin build quadrotor_mpc
3. 启动PX4节点：
   ```
   roslaunch px4 mavros_posix_sitl.launch
   ```
   若成功启动，则能看到Gazebo打开且正确加载了无人机，终端显示Ready to takeoff
   
   PS:首次启动PX4节点，要打开地面站`QGroundControl`修改参数`THR_MDL_FAC`的值（0.8-1.0应该都可以）来使得推力映射更接近真实（参考官方文档 https://docs.px4.io/v1.14/en/config_mc/pid_tuning_guide_multicopter.html 中Thrust Curve部分），后面若修改了最大推力约束最好再调一下这个参数
 
   <img width="583" height="107" alt="5215d4c4bb79865c448269a138b15031" src="https://github.com/user-attachments/assets/5ef3dc1a-75d9-41d6-b94c-fb1b0f847bce" />

5. 打开另一个终端，启动控制算法节点：
   ```
   rosrun quadrotor_mpc tracking_mpc
   ```
   无人机沿双扭线飞行三圈，表示成功运行

   
