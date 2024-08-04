# Joystick

![image](https://github.com/user-attachments/assets/44add8b1-2e7e-4485-8759-adcd089ce3e5)

## 使用keybord控制搖桿位置(ROS架構下,node之間溝通)
#### 2024.07.19
#### 程式位置:
##### 1. Keyboard_control_ros1.py  
node name: **keyboard_control**, publish topic: **/amr_control_vel**

/home/emma/Documents/joystick_ws/src/my_publisher/src

操控鍵盤

###### 改變方向
以s為中心,點擊前w、後x、左a、右f、右前r、左前q、右後v、左後z

###### 改變速度
u 搖桿加速, j 搖桿減速


##### 2. car_simulator.py 
node name: **joystick_simulator** , subscribe topic: **/amr_control_vel**

/home/emma/Documents/joystick_ws/src/my_publisher/src

可顯示搖桿移動狀況

#### 套件安裝：
1. pygame: `pip install pygame`

#### 建立 ROS Nodes
1. 建立工作空間
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    ```
2. 設定環境指標
    ```
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
3. 建立package套件包 (name: my_publisher)
    ```
    cd ~/catkin_ws/src
    catkin_create_pkg my_publisher std_msgs rospy
    ```
4. 在my_publisher套件內建立一個如下talker.py的腳本檔案：
    ```
    cd ~/catkin_ws/src/my_publisher/src
    touch XXX.py
    ```    
5. 打開 XXX.py文件並編輯
    ```
    nano XXX.py
    ```
7. 儲存並退出檔案
   Save
    ```
    Ctrl + O
    ```
    Exit
    ```
    Ctrl + X
    ```
9. 使檔案具有執行權限
    ```
    chmod +x XXX.py
    ```
10. 修改CMakeLists.txt檔案包含腳本 (node ==> src/XXX.py)
    ```
    cd ~/catkin_ws/src/my_publisher
    echo 'catkin_install_python(PROGRAMS src/XXX.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})' >> CMakeLists.txt
    ```
11. 編譯工作空間
    ```
    cd ~/catkin_ws
    catkin_make
    ```

#### 執行
Terminal A: 
```
roscore
```
Terminal B:
```
cd Documents/joystick_ws
source devel/setup.zsh
rosrun my_publisher Keyboard_control_ros1.py
```
Terminal C:
```
cd Documents/joystick_ws
source devel/setup.zsh
rosrun my_publisher car_simulator.py
```
若有更改程式,記得在source之前先
```
catkin_make
```
