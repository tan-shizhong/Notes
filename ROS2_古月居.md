# ROS2
---

- @author: ShiZhong Tan

- @date: 2022-08-16

---

<!-- @import "[TOC]" {cmd="toc" depthFrom=2 depthTo=6 orderedList=true} -->

<!-- code_chunk_output -->

1. [基本概念](#基本概念)
    1. [工作空间（Workspace）](#工作空间workspace)
    2. [节点（Node）](#节点node)
        1. [面向过程实现方式](#面向过程实现方式)
        2. [面向对象实现方式](#面向对象实现方式)
    3. [话题（Topic）](#话题topic)
        1. [话题发布者流程](#话题发布者流程)
        2. [话题订阅者流程](#话题订阅者流程)
        3. [常用话题操作](#常用话题操作)
    4. [服务（service）](#服务service)
        1. [客户端\服务器模型](#客户端服务器模型)
        2. [客户端](#客户端)
        3. [服务端](#服务端)
        4. [服务命令行操作](#服务命令行操作)
    5. [通信接口（Interface）](#通信接口interface)
        1. [接口类别](#接口类别)
        2. [服务接口](#服务接口)
            1. [接口定义](#接口定义)
            2. [接口调用](#接口调用)
        3. [话题接口](#话题接口)
            1. [接口定义](#接口定义-1)
            2. [接口调用](#接口调用-1)
        4. [接口命令行操作](#接口命令行操作)
    6. [动作（Action）](#动作action)
        1. [接口定义](#接口定义-2)
        2. [通信模型](#通信模型)
            1. [服务端模型](#服务端模型)
            2. [客户端模型](#客户端模型)
        3. [动作命令行操作](#动作命令行操作)
    7. [参数（Parameter）](#参数parameter)
        1. [通信模型](#通信模型-1)
        2. [全局字典](#全局字典)
        3. [可动态监控](#可动态监控)
        4. [声明、创建、修改一个参数的值](#声明-创建-修改一个参数的值)
        5. [命令行操作](#命令行操作)
    8. [分布式通信（Distributed Communicatiion）](#分布式通信distributed-communicatiion)
    9. [DDS（Data Distribution Service）](#ddsdata-distribution-service)
        1. [常用通信模型](#常用通信模型)
        2. [质量服务策略QoS](#质量服务策略qos)
        3. [代码](#代码)
        4. [命令行操作](#命令行操作-1)
2. [机器人配置及仿真](#机器人配置及仿真)
    1. [Launch：多节点启动与配置脚本](#launch多节点启动与配置脚本)
        1. [命令行参数配置](#命令行参数配置)
        2. [资源重映射](#资源重映射)
        3. [ROS参数设置](#ros参数设置)
        4. [加载参数文件](#加载参数文件)
        5. [Launch互相调用](#launch互相调用)
        6. [功能包编译配置](#功能包编译配置)
    2. [TF：机器人坐标系管理](#tf机器人坐标系管理)
        1. [生成坐标关系](#生成坐标关系)
        2. [静态TF广播](#静态tf广播)
        3. [TF监听](#tf监听)
        4. [海龟跟随实例](#海龟跟随实例)
            1. [坐标系动态广播](#坐标系动态广播)
            2. [海龟跟随](#海龟跟随)
    3. [URDF：机器人建模方法（统一机器人描述格式）](#urdf机器人建模方法统一机器人描述格式)
        1. [连杆Link](#连杆link)
        2. [关节Joint](#关节joint)
        3. [完整机器人模型](#完整机器人模型)
        4. [创建机器人模型](#创建机器人模型)
    4. [Gazebo：三维物理仿真平台](#gazebo三维物理仿真平台)
        1. [XACRO机器人模型优化](#xacro机器人模型优化)
        2. [XACRO文件语法](#xacro文件语法)
            1. [常量定义](#常量定义)
            2. [数字计算](#数字计算)
            3. [宏定义](#宏定义)
            4. [文件包含](#文件包含)
        3. [机器人仿真模型配置](#机器人仿真模型配置)
        4. [构建仿真环境](#构建仿真环境)
            1. [机器人模型代码](#机器人模型代码)
    5. [Rviz：三维可视化显示平台](#rviz三维可视化显示平台)
        1. [彩色相机仿真与可视化](#彩色相机仿真与可视化)
        2. [图像数据可视化](#图像数据可视化)
        3. [三维相机仿真与可视化](#三维相机仿真与可视化)
        4. [激光雷达仿真与可视化](#激光雷达仿真与可视化)
        5. [点云数据可视化](#点云数据可视化)
        6. [Gazebo与Rviz区别](#gazebo与rviz区别)
    6. [RQT：模块化可视化工具](#rqt模块化可视化工具)

<!-- /code_chunk_output -->

## 基本概念

### 工作空间（Workspace）
- src，代码空间，未来编写的代码、脚本，都需要人为的放置到这里；
- build，编译空间，保存编译过程中产生的中间文件；
- install，安装空间，放置编译得到的可执行文件和脚本；
- log，日志空间，编译和运行过程中，保存各种警告、错误、信息等日志。

### 节点（Node）

![/image](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.3_%E8%8A%82%E7%82%B9/image-20220526231417594.png)

1. 初始化编程语言接口
2. 初始化节点（需要署名，每个节点名字唯一）
3. 实现节点的功能
4. 销毁节点节约内存
   
**运行节点**

`ros2 run learning_node node_helloworld`

所有的代码更改后要编写后需要设置功能包的编译选项，让系统知道Python程序的入口，一般选择在同路径创建一个`setup.py`文件；
并重新编译

#### 面向过程实现方式

```python
# Hello World 程序示意
#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import time

def main(args=None):                             # ROS2节点主入口main函数
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = Node("node_helloworld")               # 创建ROS2节点对象并进行初始化
    
    while rclpy.ok():                            # ROS2系统是否正常运行
        node.get_logger().info("Hello World")    # ROS2日志输出
        time.sleep(0.5)                          # 休眠控制循环时间
    
    node.destroy_node()                          # 销毁节点对象    
    rclpy.shutdown()       
```

Python完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：
```python
    entry_points={
        'console_scripts': [
         'node_helloworld       = learning_node.node_helloworld:main',
        ],
    }
```

#### 面向对象实现方式

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
import time

"""
创建一个HelloWorld节点, 初始化时输出“hello world”日志
"""
class HelloWorldNode(Node):
    def __init__(self, name):
        super().__init__(name)                     # ROS2节点父类初始化
        while rclpy.ok():                          # ROS2系统是否正常运行
            self.get_logger().info("Hello World")  # ROS2日志输出
            time.sleep(0.5)                        # 休眠控制循环时间

def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = HelloWorldNode("node_helloworld_class") # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口
```

### 话题（Topic）

发布/订阅者模型
![发布订阅者模型](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.4_%E8%AF%9D%E9%A2%98/image8.gif)

- 基于DDS的发布/订阅模型：话题数据传输的特性是从一个节点到另外一个节点，发送数据的对象称之为发布者，接收数据的对象称之为订阅者，每一个话题都需要有一个名字，传输的数据也需要有固定的数据类型。
- 多对多通信：发布者和订阅者的数量并不是唯一的，可以称之为是多对多的通信模型。
- 异步通信：指发布者发出数据后，并不知道订阅者什么时候可以收到。
- 消息接口：发布者和订阅者就得统一数据的描述格式，在ROS中，话题通信数据的描述格式称之为消息，对应编程语言中数据结构的概念。

#### 话题发布者流程

- 编程接口初始化
- 创建节点并初始化
- 创建发布者对象
- 创建并填充话题消息
- 发布话题消息
- 销毁节点并关闭接口
  
使用`create_publisher（消息类型、话题名、队列长度）`来创建一个发布者

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-发布“Hello World”话题
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from std_msgs.msg import String                  # 字符串消息类型

"""
创建一个发布者节点
"""
class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.pub = self.create_publisher(String, "chatter", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        msg = String()                                            # 创建一个String类型的消息对象
        msg.data = 'Hello World'                                  # 填充消息对象中的消息数据
        self.pub.publish(msg)                                     # 发布话题消息
        self.get_logger().info('Publishing: "%s"' % msg.data)     # 输出日志信息，提示已经完成话题发布

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = PublisherNode("topic_helloworld_pub")     # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

#### 话题订阅者流程

- 编程接口初始化
- 创建节点并初始化
- 创建订阅者对象
- 回调函数处理话题数据
- 销毁节点并关闭接口

使用`create_subscription（消息类型、话题名、订阅者回调函数、队列长度）`来创建一个发布者

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅“Hello World”话题消息
"""

import rclpy                      # ROS2 Python接口库
from rclpy.node   import Node     # ROS2 节点类
from std_msgs.msg import String   # ROS2标准定义的String消息


class SubscriberNode(Node):

    def __init__(self, name):
        super().__init__(name)                             # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            String, "chatter", self.listener_callback, 10) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）

    def listener_callback(self, msg):                      # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('I heard: "%s"' % msg.data) # 输出日志信息，提示订阅收到的话题消息

def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = SubscriberNode("topic_helloworld_sub")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口  
```

#### 常用话题操作

命令行中操作代码

```
$ ros2 topic list                # 查看话题列表
$ ros2 topic info <topic_name>   # 查看话题信息
$ ros2 topic hz <topic_name>     # 查看话题发布频率
$ ros2 topic bw <topic_name>     # 查看话题传输带宽
$ ros2 topic echo <topic_name>   # 查看话题数据
$ ros2 topic pub <topic_name> <msg_type> <msg_data>   # 发布话题消息

$ rqt_graph  # 将发布者和接受者关系用图的形式表现出来
```

### 服务（service）

在需要数据的时候，发一个查询的请求，然后尽快得到此时目标对应的数据。

这样的通信模型和话题单向传输有所不同，变成了发送一个请求，反馈一个应答的形式；这种通信机制在ROS中称为服务，Service。

#### 客户端\服务器模型

从服务的实现机制上来看，这种发送请求反馈应答的形式叫做客户端/服务器模型，简称为CS模型；客户端在需要某些数据的时候，针对某个具体的服务，发送请求信息，服务器端收到请求之后，就会进行处理并反馈应答信息。

![服务器模型](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.5_%E6%9C%8D%E5%8A%A1/image8.gif)

#### 客户端

创建客户端

- 编程接口初始化
- 创建节点并初始化
- 创建客户端对象
- 创建并发送请求数据
- 等待服务器端应答数据
- 销毁节点并关闭接口

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-发送两个加数，请求加法器计算
"""

import sys

import rclpy                                                                      # ROS2 Python接口库
from rclpy.node   import Node                                                     # ROS2 节点类
from learning_interface.srv import AddTwoInts                                     # 自定义的服务接口

class adderClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                    # ROS2节点父类初始化
        self.client = self.create_client(AddTwoInts, 'add_two_ints')              # 创建服务客户端对象（服务接口类型，服务名）
        while not self.client.wait_for_service(timeout_sec=1.0):                  # 循环等待服务器端成功启动
            self.get_logger().info('service not available, waiting again...') 
        self.request = AddTwoInts.Request()                                       # 创建服务请求的数据对象
                    
    def send_request(self):                                                       # 创建一个发送服务请求的函数
        self.request.a = int(sys.argv[1])
        self.request.b = int(sys.argv[2])
        self.future = self.client.call_async(self.request)                        # 异步方式发送服务请求

def main(args=None):
    rclpy.init(args=args)                                                         # ROS2 Python接口初始化
    node = adderClient("service_adder_client")                                    # 创建ROS2节点对象并进行初始化
    node.send_request()                                                           # 发送服务请求
    
    while rclpy.ok():                                                             # ROS2系统正常运行
        rclpy.spin_once(node)                                                     # 循环执行一次节点

        if node.future.done():                                                    # 数据是否处理完成
            try:
                response = node.future.result()                                   # 接收服务器端的反馈数据
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(                                           # 将收到的反馈信息打印输出
                    'Result of add_two_ints: for %d + %d = %d' % 
                    (node.request.a, node.request.b, response.sum))
            break
            
    node.destroy_node()                                                           # 销毁节点对象
    rclpy.shutdown()                                                              # 关闭ROS2 Python接口

```

#### 服务端

创建服务端

- 编程接口初始化
- 创建节点并初始化
- 创建服务器端对象
- 通过回调函数处进行服务
- 向客户端反馈应答结果
- 销毁节点并关闭接口

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-提供加法器的服务器处理功能
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from learning_interface.srv import AddTwoInts    # 自定义的服务接口

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)                                                             # ROS2节点父类初始化
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)    # 创建服务器对象（接口类型、服务名、服务器回调函数）

    def adder_callback(self, request, response):                                           # 创建回调函数，执行收到请求后对数据的处理
        response.sum = request.a + request.b                                               # 完成加法求和计算，将结果放到反馈的数据中
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))   # 输出日志信息，提示已经完成加法求和计算
        return response                                                                    # 反馈应答信息

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = adderServer("service_adder_server")       # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

```

#### 服务命令行操作

```
$ ros2 service list                  # 查看服务列表
$ ros2 service type <service_name>   # 查看服务数据类型
$ ros2 service call <service_name> <service_type> <service_data>   # 发送服务请求
```


### 通信接口（Interface）

软件开发中，接口的使用就更多了，比如我们在编写程序时，使用的函数和函数的输入输出也称之为接口，每一次调用函数的时候，就像是把主程序和调用函数通过这个接口连接到一起，系统才能正常工作。

![ROS通信接口](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.6_%E9%80%9A%E4%BF%A1%E6%8E%A5%E5%8F%A3/image-20220528001346095.png)

ROS通信接口的主要目的就是传输数据，那就得让大家高效的建立连接，并且准确包装和解析传输的数据内容，话题、服务等机制也就诞生了，他们传输的数据，都要符合通信接口的标准定义。

#### 接口类别

ROS有三种常用的通信机制，分别是**话题、服务、动作**，通过每一种通信种定义的接口，各种节点才能有机的联系到一起。

![接口类别](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.6_%E9%80%9A%E4%BF%A1%E6%8E%A5%E5%8F%A3/image-20220528001633925.png)

- 话题通信接口的定义使用的是.msg文件，由于是单向传输，只需要描述传输的每一帧数据是什么就行，比如在这个定义里，会传输两个32位的整型数，x、y，我们可以用来传输二维坐标的数值。

- 服务通信接口的定义使用的是.srv文件，包含请求和应答两部分定义，通过中间的“---”区分，比如之前我们学习的加法求和功能，请求数据是两个64位整型数a和b，应答是求和的结果sum。

- 动作是另外一种通信机制，用来描述机器人的一个运动过程，使用.action文件定义，比如我们让小海龟转90度，一边转一边周期反馈当前的状态，此时接口的定义分成了三个部分，分别是动作的目标，比如是开始运动，运动的结果，最终旋转的90度是否完成，还有一个周期反馈，比如每隔1s反馈一下当前转到第10度、20度还是30度了，让我们知道运动的进度。

#### 服务接口

![服务接口](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.6_%E9%80%9A%E4%BF%A1%E6%8E%A5%E5%8F%A3/image-20220528002304850.png)

##### 接口定义

使用`GetObjectPosition.srv`定义服务通信的接口

```
bool get      # 获取目标位置的指令
---
int32 x       # 目标的X坐标
int32 y       # 目标的Y坐标
```

定义中有两个部分，上边是获取目标位置的指令，get为true的话，就表示我们需要一次位置，服务端就会反馈这个x、y坐标了。

完成定义后，还需要在功能包的CMakeLists.txt中配置编译选项，让编译器在编译过程中，根据接口定义，自动生成不同语言的代码：

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectPosition.srv"
)
```

```
 <build_depend>rosidl_default_generators</build_depend>
 <exec_depend>rosidl_default_runtime</exec_depend>
 <member_of_group>rosidl_interface_packages</member_of_group>
```

##### 接口调用

客户端接口调用
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-请求目标识别，等待目标位置应答
"""

import rclpy                                            # ROS2 Python接口库
from rclpy.node   import Node                           # ROS2 节点类
from learning_interface.srv import GetObjectPosition    # 自定义的服务接口

class objectClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                                  # ROS2节点父类初始化
        self.client = self.create_client(GetObjectPosition, 'get_target_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = GetObjectPosition.Request()
                    
    def send_request(self):
        self.request.get = True
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)                             # ROS2 Python接口初始化
    node = objectClient("service_object_client")       # 创建ROS2节点对象并进行初始化
    node.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(node)

        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(
                    'Result of object position:\n x: %d y: %d' %
                    (response.x, response.y))
            break
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

```

服务端接口调用
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2服务示例-提供目标识别服务
"""

import rclpy                                           # ROS2 Python接口库
from rclpy.node import Node                            # ROS2 节点类
from sensor_msgs.msg import Image                      # 图像消息类型
import numpy as np                                     # Python数值计算库
from cv_bridge import CvBridge                         # ROS与OpenCV图像转换类
import cv2                                             # Opencv图像处理库
from learning_interface.srv import GetObjectPosition   # 自定义的服务接口

lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                          # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)             # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()                                     # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

        self.srv = self.create_service(GetObjectPosition,               # 创建服务器对象（接口类型、服务名、服务器回调函数）
                                       'get_target_position',
                                       self.object_position_callback)    
        self.objectX = 0
        self.objectY = 0                              

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)             # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)        # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)          # 图像中轮廓检测

        for cnt in contours:                                         # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)                     # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)       # 将苹果的轮廓勾勒出来
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5,
                       (0, 255, 0), -1)                              # 将苹果的图像中心点画出来
            
            self.objectX = int(x+w/2)
            self.objectY = int(y+h/2)

        cv2.imshow("object", image)                                  # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(50)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')               # 输出日志信息，提示已进入回调函数
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')            # 将ROS的图像消息转化成OpenCV图像
        self.object_detect(image)                                      # 苹果检测

    def object_position_callback(self, request, response):            # 创建回调函数，执行收到请求后对数据的处理
        if request.get == True:
            response.x = self.objectX                                 # 目标物体的XY坐标
            response.y = self.objectY
            self.get_logger().info('Object position\nx: %d y: %d' %
                                   (response.x, response.y))          # 输出日志信息，提示已经反馈
        else:
            response.x = 0
            response.y = 0
            self.get_logger().info('Invalid command')                 # 输出日志信息，提示已经反馈
        return response


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImageSubscriber("service_object_server")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

```

#### 话题接口

话题通信接口的定义也是类似的，继续从之前的机器视觉案例中来衍生，我们想把服务换成话题，周期发布目标识别的位置，不管有没有人需要。
![话题接口定义](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.6_%E9%80%9A%E4%BF%A1%E6%8E%A5%E5%8F%A3/image-20220528003434007.png)

现在我们会运行三个节点：

- 第一个节点，将驱动相机并发布图像话题，此时的话题数据使用的是ROS中标准定义的Image图像消息；
- 第二个节点，会运行视觉识别功能，识别目标的位置，这个位置我们希望封装成话题消息，发布出去，谁需要使用谁就来订阅；
- 第三个节点，订阅位置话题，打印到终端中。

##### 接口定义
使用`ObjectPosition.msg`定义了服务通信的接口
```
int32 x      # 表示目标的X坐标
int32 y      # 表示目标的Y坐标
```

话题消息的内容是一个位置，我们使用x、y坐标值进行描述。

完成定义后，还需要在功能包的CMakeLists.txt中配置编译选项，让编译器在编译过程中，根据接口定义，自动生成不同语言的代码：

```

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectPosition.msg"
)
```

##### 接口调用

发布者接口调用
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2接口示例-发布目标位置
"""

import rclpy                                       # ROS2 Python接口库
from rclpy.node import Node                        # ROS2 节点类
from sensor_msgs.msg import Image                  # 图像消息类型
from cv_bridge import CvBridge                     # ROS与OpenCV图像转换类
import cv2                                         # Opencv图像处理库
import numpy as np                                 # Python数值计算库
from learning_interface.msg import ObjectPosition  # 自定义的目标位置消息

lower_red = np.array([0, 90, 128])                 # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])              # 红色的HSV阈值上限

"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):

    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.pub = self.create_publisher(
            ObjectPosition, "object_position", 10)              # 创建发布者对象（消息类型、话题名、队列长度）
        self.cv_bridge = CvBridge()                             # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

        self.objectX = 0
        self.objectY = 0   

    def object_detect(self, image):      
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)        # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)   # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)     # 图像中轮廓检测
        for cnt in contours:                                    # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)                # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高

            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)  # 将苹果的轮廓勾勒出来
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5,      # 将苹果的图像中心点画出来
                       (0, 255, 0), -1)   

            self.objectX = int(x+w/2)
            self.objectY = int(y+h/2)

        cv2.imshow("object", image)                             # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(50)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # 输出日志信息，提示已进入回调函数
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # 将ROS的图像消息转化成OpenCV图像
        position = ObjectPosition()
        self.object_detect(image)                               # 苹果检测
        position.x, position.y = int(self.objectX), int(self.objectY)
        self.pub.publish(position)                              # 发布目标位置

def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = ImageSubscriber("topic_webcam_sub")              # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口
```

订阅者接口调用
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2接口示例-订阅目标位置
"""

import rclpy                                       # ROS2 Python接口库
from rclpy.node   import Node                      # ROS2 节点类
from std_msgs.msg import String                    # 字符串消息类型
from learning_interface.msg import ObjectPosition  # 自定义的目标位置消息

"""
创建一个订阅者节点
"""
class SubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)                                                # ROS2节点父类初始化
        self.sub = self.create_subscription(\
            ObjectPosition, "/object_position", self.listener_callback, 10)   # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度

    def listener_callback(self, msg):                                         # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('Target Position: "(%d, %d)"' % (msg.x, msg.y))# 输出日志信息，提示订阅收到的话题消息


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = SubscriberNode("interface_position_sub")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

#### 接口命令行操作
```
$ ros2 interface list                    # 查看系统接口列表
$ ros2 interface show <interface_name>   # 查看某个接口的详细定义
$ ros2 interface package <package_name>  # 查看某个功能包中的接口定义
```

### 动作（Action）

一种ROS通信机制也会被常常用到——那就是动作。从这个名字上就可以很好理解这个概念的含义，这种通信机制的目的就是便于对机器人某一完整行为的流程进行管理

![动作模型](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528005012082.png)
再发送指令后，需要得到动作完成的反馈。

机器人动作使用的也是**客户端/服务器模型**

![动作服务器模型](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image8.gif)

客户端发送一个运动的目标，想让机器人动起来，服务器端收到之后，就开始控制机器人运动，一边运动，一边反馈当前的状态，如果是一个导航动作，这个反馈可能是当前所处的坐标，如果是机械臂抓取，这个反馈可能又是机械臂的实时姿态。当运动执行结束后，服务器再反馈一个动作结束的信息。整个通信过程就此结束。

动作也是一种**同步通信**机制，之前我们也介绍过，动作过程中的数据通信接口，使用`.action`文件进行定义。

#### 接口定义

通过`MoveCircle.action`进行自定义：
```
bool enable     # 定义动作的目标，表示动作开始的指令
---
bool finish     # 定义动作的结果，表示是否成功执行
---
int32 state     # 定义动作的反馈，表示当前执行到的位置
```

动作接口定义包含三个部分：
- 第一块是动作的目标，enable为true时，表示开始运动；
- 第二块是动作的执行结果，finish为true，表示动作执行完成；
- 第三块是动作的周期反馈，表示当前机器人旋转到的角度。

完成定义后，还需要在功能包的CMakeLists.txt中配置编译选项，让编译器在编译过程中，根据接口定义，自动生成不同语言的代码：

```
...
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveCircle.action"
)
...

```

#### 通信模型

通信模型就是这样，客户端发送给一个动作目标，服务器控制机器人开始运动，并周期反馈，结束后反馈结束信息。

![通信模型](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.7_%E5%8A%A8%E4%BD%9C/image-20220528010217043.png)

##### 服务端模型
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2动作示例-负责执行圆周运动动作的服务端
"""

import time

import rclpy                                      # ROS2 Python接口库
from rclpy.node   import Node                     # ROS2 节点类
from rclpy.action import ActionServer             # ROS2 动作服务器类
from learning_interface.action import MoveCircle  # 自定义的圆周运动接口

class MoveCircleActionServer(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化
        self._action_server = ActionServer(      # 创建动作服务器（接口类型、动作名、回调函数）
            self,
            MoveCircle,
            'move_circle',
            self.execute_callback)

    def execute_callback(self, goal_handle):            # 执行收到动作目标之后的处理函数
        self.get_logger().info('Moving circle...')
        feedback_msg = MoveCircle.Feedback()            # 创建一个动作反馈信息的消息

        for i in range(0, 360, 30):                     # 从0到360度，执行圆周运动，并周期反馈信息
            feedback_msg.state = i                      # 创建反馈信息，表示当前执行到的角度
            self.get_logger().info('Publishing feedback: %d' % feedback_msg.state)
            goal_handle.publish_feedback(feedback_msg)  # 发布反馈信息
            time.sleep(0.5)

        goal_handle.succeed()                           # 动作执行成功
        result = MoveCircle.Result()                    # 创建结果消息
        result.finish = True                            
        return result                                   # 反馈最终动作执行的结果

def main(args=None):                                    # ROS2节点主入口main函数
    rclpy.init(args=args)                               # ROS2 Python接口初始化
    node = MoveCircleActionServer("action_move_server") # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                    # 循环等待ROS2退出
    node.destroy_node()                                 # 销毁节点对象
    rclpy.shutdown()                                    # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'action_move_server    = learning_action.action_move_server:main',
        ],
    },
```

##### 客户端模型

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2动作示例-请求执行圆周运动动作的客户端
"""

import rclpy                                      # ROS2 Python接口库
from rclpy.node   import Node                     # ROS2 节点类
from rclpy.action import ActionClient             # ROS2 动作客户端类

from learning_interface.action import MoveCircle  # 自定义的圆周运动接口

class MoveCircleActionClient(Node):
    def __init__(self, name):
        super().__init__(name)                   # ROS2节点父类初始化
        self._action_client = ActionClient(      # 创建动作客户端（接口类型、动作名）
            self, MoveCircle, 'move_circle') 

    def send_goal(self, enable):                 # 创建一个发送动作目标的函数
        goal_msg = MoveCircle.Goal()             # 创建一个动作目标的消息
        goal_msg.enable = enable                 # 设置动作目标为使能，希望机器人开始运动

        self._action_client.wait_for_server()    # 等待动作的服务器端启动
        self._send_goal_future = self._action_client.send_goal_async(   # 异步方式发送动作的目标
            goal_msg,                                                   # 动作目标
            feedback_callback=self.feedback_callback)                   # 处理周期反馈消息的回调函数

        self._send_goal_future.add_done_callback(self.goal_response_callback) # 设置一个服务器收到目标之后反馈时的回调函数

    def goal_response_callback(self, future):           # 创建一个服务器收到目标之后反馈时的回调函数
        goal_handle = future.result()                   # 接收动作的结果
        if not goal_handle.accepted:                    # 如果动作被拒绝执行
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')                            # 动作被顺利执行

        self._get_result_future = goal_handle.get_result_async()              # 异步获取动作最终执行的结果反馈
        self._get_result_future.add_done_callback(self.get_result_callback)   # 设置一个收到最终结果的回调函数 

    def get_result_callback(self, future):                                    # 创建一个收到最终结果的回调函数
        result = future.result().result                                       # 读取动作执行的结果
        self.get_logger().info('Result: {%d}' % result.finish)                # 日志输出执行结果

    def feedback_callback(self, feedback_msg):                                # 创建处理周期反馈消息的回调函数
        feedback = feedback_msg.feedback                                      # 读取反馈的数据
        self.get_logger().info('Received feedback: {%d}' % feedback.state) 

def main(args=None):                                       # ROS2节点主入口main函数
    rclpy.init(args=args)                                  # ROS2 Python接口初始化
    node = MoveCircleActionClient("action_move_client")    # 创建ROS2节点对象并进行初始化
    node.send_goal(True)                                   # 发送动作目标
    rclpy.spin(node)                                       # 循环等待ROS2退出
    node.destroy_node()                                    # 销毁节点对象
    rclpy.shutdown()                                       # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
    entry_points={
        'console_scripts': [
         'action_move_client    = learning_action.action_move_client:main',
         'action_move_server    = learning_action.action_move_server:main',
        ],
    },
```


#### 动作命令行操作
```
$ ros2 action list                  # 查看服务列表
$ ros2 action info <action_name>    # 查看服务数据类型
$ ros2 action send_goal <action_name> <action_type> <action_data>   # 发送服务请求
```

### 参数（Parameter）

类似C++编程中的全局变量，可以便于在多个程序中共享某些数据，参数是ROS机器人系统中的全局字典，可以运行多个节点中共享数据。

#### 通信模型

![通信模型](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.8_%E5%8F%82%E6%95%B0/image-20220528013439656.png)

在NodeA相机驱动节点中，就需要考虑很多问题，相机连接到哪个usb端口，使用的图像分辨率是多少，曝光度和编码格式分别是什么，这些都可以通过参数设置，我们可以通过配置文件或者程序进行设置。

NodeB节点中也是一样，图像识别使用的阈值是多少，整个图像面积很大，那个部分是我们关注的核心区域，识别过程是否需要美颜等等，就像我们使用美颜相机一样，我们可以通过滑动条或者输入框设置很多参数，不同参数设置后，都会改变执行功能的一些效果。

这就是参数的作用。

#### 全局字典

在ROS系统中，参数是以全局字典的形态存在的，什么叫字典？就像真实的字典一样，由名称和数值组成，也叫做键和值，合成键值。或者我们也可以理解为，就像编程中的参数一样，有一个参数名 ，然后跟一个等号，后边就是参数值了，在使用的时候，访问这个参数名即可。

#### 可动态监控

在ROS2中，参数的特性非常丰富，比如某一个节点共享了一个参数，其他节点都可以访问，如果某一个节点对参数进行了修改，其他节点也有办法立刻知道，从而获取最新的数值。这在参数的高级编程中，大家都可能会用到。

#### 声明、创建、修改一个参数的值

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2参数示例-创建、读取、修改参数
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类

class ParameterNode(Node):
    def __init__(self, name):
        super().__init__(name)                                    # ROS2节点父类初始化
        self.timer = self.create_timer(2, self.timer_callback)    # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.declare_parameter('robot_name', 'mbot')              # 创建一个参数，并设置参数的默认值

    def timer_callback(self):                                      # 创建定时器周期执行的回调函数
        robot_name_param = self.get_parameter('robot_name').get_parameter_value().string_value   # 从ROS2系统中读取参数的值

        self.get_logger().info('Hello %s!' % robot_name_param)     # 输出日志信息，打印读取到的参数值

        new_name_param = rclpy.parameter.Parameter('robot_name',   # 重新将参数值设置为指定值
                            rclpy.Parameter.Type.STRING, 'mbot')
        all_new_parameters = [new_name_param]
        self.set_parameters(all_new_parameters)                    # 将重新创建的参数列表发送给ROS2系统

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ParameterNode("param_declare")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```
entry_points={
    'console_scripts': [
     'param_declare          = learning_parameter.param_declare:main',
    ],
},
```

#### 命令行操作

参数文件保存与加载
```
$ ros2 param dump turtlesim >> turtlesim.yaml  # 将某个节点的参数保存到参数文件中
$ ros2 param load turtlesim turtlesim.yaml     # 一次性加载某一个文件中的所有参数
```

参数查询与修改
```
$ ros2 param describe turtlesim background_b   # 查看某个参数的描述信息
$ ros2 param get turtlesim background_b        # 查询某个参数的值
$ ros2 param set turtlesim background_b 10     # 修改某个参数的值
```

### 分布式通信（Distributed Communicatiion）

在ROS系统中，机器人功能是由各种节点组成的，这些节点可能位于不同的计算机中，这种结构可以将原本资源消耗较多的任务，分配到不同的平台上，减轻计算压力，这就是分布式通信框架的典型应用之一。

![分布式通信](https://book.guyuehome.com/ROS2/2.%E6%A0%B8%E5%BF%83%E6%A6%82%E5%BF%B5/image/2.9_%E5%88%86%E5%B8%83%E5%BC%8F%E9%80%9A%E4%BF%A1/image-20220528014805888.png)

分布式网络分组

在`bashsrc`中更改
```
$ export ROS_DOMAIN_ID=<your_domain_id>
```

### DDS（Data Distribution Service）

#### 常用通信模型

 1.点对点模型，许多客户端连接到一个服务端，每次通信时，通信双方必须建立一条连接。当通信节点增多时，连接数也会增多。而且每个客户端都需要知道服务器的具体地址和所提供的服务，一旦服务器地址发生变化，所有客户端都会受到影响。

 2.Broker模型，针对点对点模型进行了优化，由Broker集中处理所有人的请求，并进一步找到真正能响应该服务的角色。这样客户端就不用关心服务器的具体地址了。不过问题也很明显，Broker作为核心，它的处理速度会影响所有节点的效率，当系统规模增长到一定程度，Broker就会成为整个系统的性能瓶颈。更麻烦是，如果Broker发生异常，可能导致整个系统都无法正常运转。之前的ROS1系统，使用的就是类似这样的架构。

3.广播模型，所有节点都可以在通道上广播消息，并且节点都可以收到消息。这个模型解决了服务器地址的问题，而且通信双方也不用单独建立连接，但是广播通道上的消息太多了，所有节点都必须关心每条消息，其实很多是和自己没有关系的。

4.就是以数据为中心的DDS模型了，这种模型与广播模型有些类似，所有节点都可以在DataBus上发布和订阅消息。但它的先进之处在于，通信中包含了很多并行的通路，每个节点可以只关心自己感兴趣的消息，忽略不感兴趣的消息，有点像是一个旋转火锅，各种好吃的都在这个DataBus传送，我们只需要拿自己想吃的就行，其他的和我们没有关系。

DDS的全称是Data Distribution Service，也就是**数据分发服务**，2004年由对象管理组织OMG发布和维护，是一套专门为实时系统设计的数据分发/订阅标准，最早应用于美国海军， 解决舰船复杂网络环境中大量软件升级的兼容性问题，现在已经成为强制标准。

DDS强调**以数据为中心**，可以提供丰富的服务质量策略，以保障数据进行实时、高效、灵活地分发，可满足各种分布式实时通信应用需求。

#### 质量服务策略QoS


- DEADLINE策略，表示通信数据必须要在每次截止时间内完成一次通信；
- HISTORY策略，表示针对历史数据的一个缓存大小；
- RELIABILITY策略，表示数据通信的模式，配置成BEST_EFFORT，就是尽力传输模式，网络情况不好的时候，也要保证数据流畅，此时可能会导致数据丢失，配置成RELIABLE，就是可信赖模式，可以在通信中尽量保证图像的完整性，我们可以根据应用功能场景选择合适的通信模式；
- DURABILITY策略，可以配置针对晚加入的节点，也保证有一定的历史数据发送过去，可以让新节点快速适应系统。

#### 代码

发布端代码

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 QoS示例-发布“Hello World”话题
"""

import rclpy                     # ROS2 Python接口库
from rclpy.node import Node      # ROS2 节点类
from std_msgs.msg import String  # 字符串消息类型
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # ROS2 QoS类

"""
创建一个发布者节点
"""
class PublisherNode(Node):

    def __init__(self, name):
        super().__init__(name)        # ROS2节点父类初始化

        qos_profile = QoSProfile(     # 创建一个QoS原则
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(String, "chatter", qos_profile) # 创建发布者对象（消息类型、话题名、QoS原则）
        self.timer = self.create_timer(0.5, self.timer_callback)         # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def timer_callback(self):                                # 创建定时器周期执行的回调函数
        msg = String()                                       # 创建一个String类型的消息对象
        msg.data = 'Hello World'                             # 填充消息对象中的消息数据
        self.pub.publish(msg)                                # 发布话题消息
        self.get_logger().info('Publishing: "%s"' % msg.data)# 输出日志信息，提示已经完成话题发布

def main(args=None):                           # ROS2节点主入口main函数
    rclpy.init(args=args)                      # ROS2 Python接口初始化
    node = PublisherNode("qos_helloworld_pub") # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                           # 循环等待ROS2退出
    node.destroy_node()                        # 销毁节点对象
    rclpy.shutdown()                           # 关闭ROS2 Python接口
```

订阅端代码

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 QoS示例-订阅“Hello World”话题消息
"""

import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from std_msgs.msg import String                  # ROS2标准定义的String消息
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # ROS2 QoS类

"""
创建一个订阅者节点
"""
class SubscriberNode(Node):

    def __init__(self, name):
        super().__init__(name)         # ROS2节点父类初始化

        qos_profile = QoSProfile(      # 创建一个QoS原则
            # reliability=QoSReliabilityPolicy.BEST_EFFORT,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(\
            String, "chatter", self.listener_callback, qos_profile) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、QoS原则）

    def listener_callback(self, msg):                      # 创建回调函数，执行收到话题消息后对数据的处理
        self.get_logger().info('I heard: "%s"' % msg.data) # 输出日志信息，提示订阅收到的话题消息

def main(args=None):                               # ROS2节点主入口main函数
    rclpy.init(args=args)                          # ROS2 Python接口初始化
    node = SubscriberNode("qos_helloworld_sub")    # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                               # 循环等待ROS2退出
    node.destroy_node()                            # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口
```

#### 命令行操作

```
$ ros2 topic pub /chatter std_msgs/msg/Int32 "data: 42" --qos-reliability best_effort 
$ ros2 topic echo /chatter --qos-reliability reliable
$ ros2 topic echo /chatter --qos-reliability best_effort
$ ros2 topic info /chatter --verbose
```

## 机器人配置及仿真

### Launch：多节点启动与配置脚本

使用Launch直接启动多个脚本配置文件，Launch的核心目的是启动节点，我们在命令行中输入的各种参数，在Launch文件中，通过类似这样的很多代码模版，也可以进行配置，甚至还可以使用Python原有的编程功能，大大丰富了启动过程中的多样化配置。

launch文件中出现的**argument**和**parameter**，虽都译为“参数”，但含义不同： 
- argument：仅限launch文件内部使用，方便在launch中调用某些数值； 
- parameter：ROS系统的参数，方便在节点见使用某些数值。

在命令行中启动`Launch`文件

```
ros2 launch learning_launch simple.launch.py
```

```python
from launch import LaunchDescription           # launch文件的描述类
from launch_ros.actions import Node            # 节点启动的描述类

def generate_launch_description():             # 自动生成launch文件的函数
    return LaunchDescription([                 # 返回launch文件的描述信息
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_helloworld_pub', # 节点的可执行文件
        ),
        Node(                                  # 配置一个节点的启动
            package='learning_topic',          # 节点所在的功能包
            executable='topic_helloworld_sub', # 节点的可执行文件名
        ),
    ])
```

#### 命令行参数配置

将命令行后的参数通过Launch传入节点

```python
import os

from ament_index_python.packages import get_package_share_directory # 查询功能包路径的方法

from launch import LaunchDescription    # launch文件的描述类
from launch_ros.actions import Node     # 节点启动的描述类


def generate_launch_description():      # 自动生成launch文件的函数
   rviz_config = os.path.join(          # 找到配置文件的完整路径
      get_package_share_directory('learning_launch'),
      'rviz',
      'turtle_rviz.rviz'
      )

   return LaunchDescription([           # 返回launch文件的描述信息
      Node(                             # 配置一个节点的启动
         package='rviz2',               # 节点所在的功能包
         executable='rviz2',            # 节点的可执行文件名
         name='rviz2',                  # 对节点重新命名
         arguments=['-d', rviz_config]  # 加载命令行参数
      )
   ])
```

#### 资源重映射

为了提高软件的复用性，ROS提供了资源重映射的机制，当使用别人的代码时，可以将通信话题的名称映射为一个新的值。

```python
from launch import LaunchDescription      # launch文件的描述类
from launch_ros.actions import Node       # 节点启动的描述类

def generate_launch_description():        # 自动生成launch文件的函数
    return LaunchDescription([            # 返回launch文件的描述信息
        Node(                             # 配置一个节点的启动
            package='turtlesim',          # 节点所在的功能包
            namespace='turtlesim1',       # 节点所在的命名空间
            executable='turtlesim_node',  # 节点的可执行文件名
            name='sim'                    # 对节点重新命名
        ),
        Node(                             # 配置一个节点的启动
            package='turtlesim',          # 节点所在的功能包
            namespace='turtlesim2',       # 节点所在的命名空间
            executable='turtlesim_node',  # 节点的可执行文件名
            name='sim'                    # 对节点重新命名
        ),
        Node(                             # 配置一个节点的启动
            package='turtlesim',          # 节点所在的功能包
            executable='mimic',           # 节点的可执行文件名
            name='mimic',                 # 对节点重新命名
            remappings=[                  # 资源重映射列表
                ('/input/pose', '/turtlesim1/turtle1/pose'),         # 将/input/pose话题名修改为/turtlesim1/turtle1/pose
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),  # 将/output/cmd_vel话题名修改为/turtlesim2/turtle1/cmd_vel
            ]
        )
    ])
```

#### ROS参数设置

```python
from launch import LaunchDescription                   # launch文件的描述类
from launch.actions import DeclareLaunchArgument       # 声明launch文件内使用的Argument类
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node                    # 节点启动的描述类


def generate_launch_description():                     # 自动生成launch文件的函数
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')     # 创建一个Launch文件内参数（arg）background_r
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')    # 创建一个Launch文件内参数（arg）background_g
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')   # 创建一个Launch文件内参数（arg）background_b
   )

   return LaunchDescription([                                      # 返回launch文件的描述信息
      background_r_launch_arg,                                     # 调用以上创建的参数（arg）
      background_g_launch_arg,
      background_b_launch_arg,
      Node(                                                        # 配置一个节点的启动
         package='turtlesim',
         executable='turtlesim_node',                              # 节点所在的功能包
         name='sim',                                               # 对节点重新命名
         parameters=[{                                             # ROS参数列表
            'background_r': LaunchConfiguration('background_r'),   # 创建参数background_r
            'background_g': LaunchConfiguration('background_g'),   # 创建参数background_g
            'background_b': LaunchConfiguration('background_b'),   # 创建参数background_b
         }]
      ),
   ])
```

#### 加载参数文件

```python
mport os

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法

from launch import LaunchDescription   # launch文件的描述类
from launch_ros.actions import Node    # 节点启动的描述类


def generate_launch_description():     # 自动生成launch文件的函数
   config = os.path.join(              # 找到参数文件的完整路径
      get_package_share_directory('learning_launch'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([          # 返回launch文件的描述信息
      Node(                            # 配置一个节点的启动
         package='turtlesim',          # 节点所在的功能包
         executable='turtlesim_node',  # 节点的可执行文件名
         namespace='turtlesim2',       # 节点所在的命名空间
         name='sim',                   # 对节点重新命名
         parameters=[config]           # 加载参数文件
      )
   ])
```

#### Launch互相调用

```python
import os

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法

from launch import LaunchDescription                 # launch文件的描述类
from launch.actions import IncludeLaunchDescription  # 节点启动的描述类
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction               # launch文件中的执行动作
from launch_ros.actions import PushRosNamespace      # ROS命名空间配置

def generate_launch_description():                   # 自动生成launch文件的函数
   parameter_yaml = IncludeLaunchDescription(        # 包含指定路径下的另外一个launch文件
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('learning_launch'), 'launch'),
         '/parameters_nonamespace.launch.py'])
      )

   parameter_yaml_with_namespace = GroupAction(      # 对指定launch文件中启动的功能加上命名空间
      actions=[
         PushRosNamespace('turtlesim2'),
         parameter_yaml]
      )

   return LaunchDescription([                        # 返回launch文件的描述信息
      parameter_yaml_with_namespace
   ])
```

#### 功能包编译配置

需要将用到的功能包以及相应文件夹加入，否则在编译时对应文件夹不会在编译时添加进去，容易产生找不到文件报错。

```python
    ...

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.*'))),
    ],

    ...
```


### TF：机器人坐标系管理

机器人中的坐标系
![各种坐标系](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.2_TF/image-20220528142046330.png)

比如在机械臂形态的机器人中，机器人安装的位置叫做基坐标系Base Frame，机器人安装位置在外部环境下的参考系叫做世界坐标系World Frame，机器人末端夹爪的位置叫做工具坐标系，外部被操作物体的位置叫做工件坐标系，在机械臂抓取外部物体的过程中，这些坐标系之间的关系也在跟随变化。

#### 生成坐标关系

```
$ ros2 run tf2_tools view_frames 
```

默认在当前终端路径下生成了一个frames.pdf文件，打开之后，就可以看到系统中各个坐标系的关系了。

![坐标关系](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.2_TF/image-20220528142507581.png)

通过`$ ros2 run tf2_ros tf2_echo turtle2 turtle1`查询坐标系变换信息

#### 静态TF广播

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-广播静态的坐标变换
"""

import rclpy                                                                 # ROS2 Python接口库
from rclpy.node import Node                                                  # ROS2 节点类
from geometry_msgs.msg import TransformStamped                               # 坐标变换消息
import tf_transformations                                                    # TF坐标变换库
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster  # TF静态坐标系广播器类

class StaticTFBroadcaster(Node):
    def __init__(self, name):
        super().__init__(name)                                                  # ROS2节点父类初始化
        self.tf_broadcaster = StaticTransformBroadcaster(self)                  # 创建一个TF广播器对象

        static_transformStamped = TransformStamped()                            # 创建一个坐标变换的消息对象
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()  # 设置坐标变换消息的时间戳
        static_transformStamped.header.frame_id = 'world'                       # 设置一个坐标变换的源坐标系
        static_transformStamped.child_frame_id  = 'house'                       # 设置一个坐标变换的目标坐标系
        static_transformStamped.transform.translation.x = 10.0                  # 设置坐标变换中的X、Y、Z向的平移
        static_transformStamped.transform.translation.y = 5.0                    
        static_transformStamped.transform.translation.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)          # 将欧拉角转换为四元数（roll, pitch, yaw）
        static_transformStamped.transform.rotation.x = quat[0]                  # 设置坐标变换中的X、Y、Z向的旋转（四元数）
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(static_transformStamped)              # 广播静态坐标变换，广播后两个坐标系的位置关系保持不变

def main(args=None):
    rclpy.init(args=args)                                # ROS2 Python接口初始化
    node = StaticTFBroadcaster("static_tf_broadcaster")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                     # 循环等待ROS2退出
    node.destroy_node()                                  # 销毁节点对象
    rclpy.shutdown()
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```python
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = learning_tf.static_tf_broadcaster:main',
        ],
    },
```

#### TF监听

查询两个坐标系之间的关系

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-监听某两个坐标系之间的变换
"""

import rclpy                                              # ROS2 Python接口库
from rclpy.node import Node                               # ROS2 节点类
import tf_transformations                                 # TF坐标变换库
from tf2_ros import TransformException                    # TF左边变换的异常类
from tf2_ros.buffer import Buffer                         # 存储坐标变换信息的缓冲类
from tf2_ros.transform_listener import TransformListener  # 监听坐标变换的监听器类

class TFListener(Node):

    def __init__(self, name):
        super().__init__(name)                                      # ROS2节点父类初始化

        self.declare_parameter('source_frame', 'world')             # 创建一个源坐标系名的参数
        self.source_frame = self.get_parameter(                     # 优先使用外部设置的参数值，否则用默认值
            'source_frame').get_parameter_value().string_value

        self.declare_parameter('target_frame', 'house')             # 创建一个目标坐标系名的参数
        self.target_frame = self.get_parameter(                     # 优先使用外部设置的参数值，否则用默认值
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()                                   # 创建保存坐标变换信息的缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建坐标变换的监听器

        self.timer = self.create_timer(1.0, self.on_timer)          # 创建一个固定周期的定时器，处理坐标信息

    def on_timer(self):
        try:
            now = rclpy.time.Time()                                 # 获取ROS系统的当前时间
            trans = self.tf_buffer.lookup_transform(                # 监听当前时刻源坐标系到目标坐标系的坐标变换
                self.target_frame,
                self.source_frame,
                now)
        except TransformException as ex:                            # 如果坐标变换获取失败，进入异常报告
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return

        pos  = trans.transform.translation                          # 获取位置信息
        quat = trans.transform.rotation                             # 获取姿态信息（四元数）
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
          % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

def main(args=None):
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = TFListener("tf_listener")            # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口
```

完成代码的编写后需要设置功能包的编译选项，让系统知道Python程序的入口，打开功能包的setup.py文件，加入如下入口点的配置：

```python
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = learning_tf.static_tf_broadcaster:main',
            'tf_listener = learning_tf.tf_listener:main',
        ],
    },
```

#### 海龟跟随实例

在两只海龟的仿真器中，我们可以定义三个坐标系，比如仿真器的全局参考系叫做world，turtle1和turtle2坐标系在两只海龟的中心点，这样，turtle1和world坐标系的相对位置，就可以表示海龟1的位置，海龟2也同理。


    小海龟仿真器
    海龟1的坐标系广播
    海龟2的坐标系广播
    海龟跟随控制

其中，两个坐标系的广播复用了turtle_tf_broadcaster节点，通过传入的参数名修改维护的坐标系名称。

启动`Launch`文件，其中主要包括四个节点

- 小海龟仿真器
- 海龟1的坐标系广播
- 海龟2的坐标系广播
- 海龟跟随控制

其中，两个坐标系的广播复用了turtle_tf_broadcaster节点，通过传入的参数名修改维护的坐标系名称。

`learning_tf/launch/turtle_following_demo.launch.py`

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf',
            executable='turtle_tf_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='learning_tf',
            executable='turtle_tf_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf',
            executable='turtle_following',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ), 
    ])
```

##### 坐标系动态广播

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-广播动态的坐标变换
"""

import rclpy                                       # ROS2 Python接口库
from rclpy.node import Node                        # ROS2 节点类
from geometry_msgs.msg import TransformStamped     # 坐标变换消息
import tf_transformations                          # TF坐标变换库
from tf2_ros import TransformBroadcaster           # TF坐标变换广播器
from turtlesim.msg import Pose                     # turtlesim小海龟位置消息

class TurtleTFBroadcaster(Node):

    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化

        self.declare_parameter('turtlename', 'turtle')        # 创建一个海龟名称的参数
        self.turtlename = self.get_parameter(                 # 优先使用外部设置的参数值，否则用默认值
            'turtlename').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)      # 创建一个TF坐标变换的广播对象并初始化

        self.subscription = self.create_subscription(         # 创建一个订阅者，订阅海龟的位置消息
            Pose,
            f'/{self.turtlename}/pose',                       # 使用参数中获取到的海龟名称
            self.turtle_pose_callback, 1)

    def turtle_pose_callback(self, msg):                              # 创建一个处理海龟位置消息的回调函数，将位置消息转变成坐标变换
        transform = TransformStamped()                                # 创建一个坐标变换的消息对象

        transform.header.stamp = self.get_clock().now().to_msg()      # 设置坐标变换消息的时间戳
        transform.header.frame_id = 'world'                           # 设置一个坐标变换的源坐标系
        transform.child_frame_id = self.turtlename                    # 设置一个坐标变换的目标坐标系
        transform.transform.translation.x = msg.x                     # 设置坐标变换中的X、Y、Z向的平移
        transform.transform.translation.y = msg.y
        transform.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta) # 将欧拉角转换为四元数（roll, pitch, yaw）
        transform.transform.rotation.x = q[0]                         # 设置坐标变换中的X、Y、Z向的旋转（四元数）
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(transform)     # 广播坐标变换，海龟位置变化后，将及时更新坐标变换信息

def main(args=None):
    rclpy.init(args=args)                                # ROS2 Python接口初始化
    node = TurtleTFBroadcaster("turtle_tf_broadcaster")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                     # 循环等待ROS2退出
    node.destroy_node()                                  # 销毁节点对象
    rclpy.shutdown()                                     # 关闭ROS2 Python接口
```

##### 海龟跟随

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2 TF示例-通过坐标变化实现海龟跟随功能
"""

import math
import rclpy                                              # ROS2 Python接口库
from rclpy.node import Node                               # ROS2 节点类
import tf_transformations                                 # TF坐标变换库
from tf2_ros import TransformException                    # TF左边变换的异常类
from tf2_ros.buffer import Buffer                         # 存储坐标变换信息的缓冲类
from tf2_ros.transform_listener import TransformListener  # 监听坐标变换的监听器类
from geometry_msgs.msg import Twist                       # ROS2 速度控制消息
from turtlesim.srv import Spawn                           # 海龟生成的服务接口
class TurtleFollowing(Node):

    def __init__(self, name):
        super().__init__(name)                                      # ROS2节点父类初始化

        self.declare_parameter('source_frame', 'turtle1')           # 创建一个源坐标系名的参数
        self.source_frame = self.get_parameter(                     # 优先使用外部设置的参数值，否则用默认值
            'source_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()                                   # 创建保存坐标变换信息的缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建坐标变换的监听器

        self.spawner = self.create_client(Spawn, 'spawn')           # 创建一个请求产生海龟的客户端
        self.turtle_spawning_service_ready = False                  # 是否已经请求海龟生成服务的标志位
        self.turtle_spawned = False                                 # 海龟是否产生成功的标志位

        self.publisher = self.create_publisher(Twist, 'turtle2/cmd_vel', 1) # 创建跟随运动海龟的速度话题

        self.timer = self.create_timer(1.0, self.on_timer)         # 创建一个固定周期的定时器，控制跟随海龟的运动

    def on_timer(self):
        from_frame_rel = self.source_frame                         # 源坐标系
        to_frame_rel   = 'turtle2'                                 # 目标坐标系

        if self.turtle_spawning_service_ready:                     # 如果已经请求海龟生成服务
            if self.turtle_spawned:                                # 如果跟随海龟已经生成
                try:
                    now = rclpy.time.Time()                        # 获取ROS系统的当前时间
                    trans = self.tf_buffer.lookup_transform(       # 监听当前时刻源坐标系到目标坐标系的坐标变换
                        to_frame_rel,
                        from_frame_rel,
                        now)
                except TransformException as ex:                   # 如果坐标变换获取失败，进入异常报告
                    self.get_logger().info(
                        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                    return

                msg = Twist()                                      # 创建速度控制消息
                scale_rotation_rate = 1.0                          # 根据海龟角度，计算角速度
                msg.angular.z = scale_rotation_rate * math.atan2(
                    trans.transform.translation.y,
                    trans.transform.translation.x)

                scale_forward_speed = 0.5                          # 根据海龟距离，计算线速度
                msg.linear.x = scale_forward_speed * math.sqrt(
                    trans.transform.translation.x ** 2 +
                    trans.transform.translation.y ** 2)

                self.publisher.publish(msg)                        # 发布速度指令，海龟跟随运动
            else:                                                  # 如果跟随海龟没有生成
                if self.result.done():                             # 查看海龟是否生成
                    self.get_logger().info(
                        f'Successfully spawned {self.result.result().name}')
                    self.turtle_spawned = True                     
                else:                                              # 依然没有生成跟随海龟
                    self.get_logger().info('Spawn is not finished')
        else:                                                      # 如果没有请求海龟生成服务
            if self.spawner.service_is_ready():                    # 如果海龟生成服务器已经准备就绪
                request = Spawn.Request()                          # 创建一个请求的数据
                request.name = 'turtle2'                           # 设置请求数据的内容，包括海龟名、xy位置、姿态
                request.x = float(4)
                request.y = float(2)
                request.theta = float(0)

                self.result = self.spawner.call_async(request)     # 发送服务请求
                self.turtle_spawning_service_ready = True          # 设置标志位，表示已经发送请求
            else:
                self.get_logger().info('Service is not ready')     # 海龟生成服务器还没准备就绪的提示


def main(args=None):
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = TurtleFollowing("turtle_following")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口
```

### URDF：机器人建模方法（统一机器人描述格式）

机器人一般是由**硬件结构、驱动系统、传感器系统、控制系统**四大部分组成

- 硬件结构就是底盘、外壳、电机等实打实可以看到的设备；
- 驱动系统就是可以驱使这些设备正常使用的装置，比如电机的驱动器，电源管理系统等；
- 传感系统包括电机上的编码器、板载的IMU、安装的摄像头、雷达等等，便于机器人感知自己的状态和外部的环境；
- 控制系统就是我们开发过程的主要载体了，一般是树莓派、电脑等计算平台，以及里边的操作系统和应用软件。

机器人建模的过程，其实就是按照类似的思路，通过建模语言，把机器人每一个部分都描述清楚，再组合起来的过程。

URDF模型文件使用的是`XML`格式

![URTF格式](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144424329.png)

在建模中，大臂和小臂就类似机器人的这些独立的刚体部分，称为**连杆Link**，手肘就类似于机器人电机驱动部分，称为**关节joint**。

所以在URDF建模过程中，关键任务就是通过这里的`<link>`和`<joint>`，理清楚每一个连杆和关节的描述信息。

#### 连杆Link

`<link>`标签用来描述机器人某个刚体部分的外观和物理属性，外观包括尺寸、颜色、形状，物理属性包括质量、惯性矩阵、碰撞参数等。

![连杆](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144534685.png)

该连杆的描述如下

![连杆描述](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144549092.png)

其中link标签中的name表示该连杆的名称，我们可以自定义，未来joint连接link的时候，会使用到这个名称。

link里边的`<visual>`部分用来描述机器人的外观，比如：

- `<geometry>`表示几何形状，里边使用<mesh>调用了一个在三维软件中提前设计好的蓝色外观，就是这个stl文件，看上去和真实机器人是一致的
- `<origin>`表示坐标系相对初始位置的偏移，分别是x、y、z方向上的平移，和roll、pitch、raw旋转，不需要偏移的话，就全为0。

第二个部分`<collision>`，描述碰撞参数，里边的内容似乎和`<visual>`一样，也有`<geometry>`和`<origin>`，看似相同，其实区别还是比较大的。

- `<visual>`部分重在描述机器人看上去的状态，也就是视觉效果；
- `<collision>`部分则是描述机器人运动过程中的状态，比如机器人与外界如何接触算作碰撞。

在这个机器人模型中，蓝色部分是通过`<visual>`来描述的，在实际控制过程中，这样复杂的外观在计算碰撞检测时，要求的算力较高，为了简化计算，我们将碰撞检测用的模型简化为了绿色框的圆柱体，也就是`<collision>`里边`<geometry>`描述的形状。`<origin>`坐标系偏移也是类似，可以描述刚体质心的偏移。

如果是移动机器人的话，link也可以用来描述小车的车体、轮子等部分。

#### 关节Joint

机器人模型中的刚体最终要通过关节joint连接之后，才能产生相对运动。

URDF中共有六种关节类型

![关节类型](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144655899.png)


1.continuous，描述旋转运动，可以围绕某一个轴无限旋转，比如小车的轮子，就属于这种类型。
2.revolute，也是旋转关节，和continuous类型的区别在于不能无限旋转，而是带有角度限制，比如机械臂的两个连杆，就属于这种运动。
3.prismatic，是滑动关节，可以沿某一个轴平移，也带有位置的极限，一般直线电机就是这种运动方式。
4.fixed，固定关节，是唯一一种不允许运动的关节，不过使用还是比较频繁的，比如相机这个连杆，安装在机器人上，相对位置是不会变化的，此时使用的连接方式就是Fixed。
5.Floating是浮动关节。
6.planar是平面关节，这两种使用相对较少。

![关节](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144722751.png)

在URDF模型中，每一个Joint都使用这样一段xml内容描述，比如关节的名字叫什么，运动类型是哪一种。

![URDF关节形式](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144729633-16537204524521.png)

- parent标签：描述父连杆；
- child标签：描述子连杆，子连杆会相对父连杆发生运动；
- origin：表示两个连杆坐标系之间的关系，也就是图中红色的向量，可以理解为这两个连杆该如何安装到一起；
- axis表示关节运动轴的单位向量，比如z等于1，就表示这个旋转运动是围绕z轴的正方向进行的；
- limit就表示运动的一些限制了，比如最小位置，最大位置，和最大速度等。


**ROS中关于平移的默认单位是m，旋转是弧度（不是度），线速度是m/s，角速度是rad/s。**

#### 完整机器人模型

最终所有的link和joint标签完成了对机器人每个部分的描述和组合，全都放在一个robot标签中，就形成了完整的机器人模型。

![完整机器人模型](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144900705.png)

![xml语言描述](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.3_URDF/image-20220528144824234.png)

#### 创建机器人模型

机器人的模型放置在learning_urdf功能包中，功能包中包含的文件夹如下：

- urdf：存放机器人模型的URDF或xacro文件
- meshes：放置URDF中引用的模型渲染文件
- launch：保存相关启动文件
- rviz：保存rviz的配置文件

可以在模型文件的路径下，使用`urdf_to_graphviz`这个小工具来分析下。

```
$ urdf_to_graphviz mbot_base.urdf  # 在模型文件夹下运行
```

xml语言描述
```xml
<?xml version="1.0" ?>
<robot name="mbot">

    <link name="base_link">
        <visual>
            <origin xyz=" 0 0 0" rpy="0 0 0" />
            <geometry>
                <cylinder length="0.16" radius="0.20"/>
            </geometry>
            <material name="yellow">
                <color rgba="1 0.4 0 1"/>
            </material>
        </visual>
    </link>

    <joint name="left_wheel_joint" type="continuous">
        <origin xyz="0 0.19 -0.05" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="left_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.06" length = "0.025"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 0.9"/>
            </material>
        </visual>
    </link>

    <joint name="right_wheel_joint" type="continuous">
        <origin xyz="0 -0.19 -0.05" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="right_wheel_link">
        <visual>
            <origin xyz="0 0 0" rpy="1.5707 0 0" />
            <geometry>
                <cylinder radius="0.06" length = "0.025"/>
            </geometry>
            <material name="white">
                <color rgba="1 1 1 0.9"/>
            </material>
        </visual>
    </link>

    <joint name="front_caster_joint" type="continuous">
        <origin xyz="0.18 0 -0.095" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="front_caster_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="front_caster_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <sphere radius="0.015" />
            </geometry>
            <material name="black">
                <color rgba="0 0 0 0.95"/>
            </material>
        </visual>
    </link>

    <joint name="back_caster_joint" type="continuous">
        <origin xyz="-0.18 0 -0.095" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="back_caster_link"/>
        <axis xyz="0 1 0"/>
    </joint>

    <link name="back_caster_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry>
                <sphere radius="0.015" />
            </geometry>
            <material name="black">
                <color rgba="0 0 0 0.95"/>
            </material>
        </visual>
    </link>

</robot>
```


### Gazebo：三维物理仿真平台

Gazebo是ROS系统中最为常用的**三维物理仿真平台**，支持动力学引擎，可以实现高质量的图形渲染，不仅可以模拟机器人及周边环境，还可以加入摩擦力、弹性系数等物理属性。

启动Gazebo
`$ ros2 launch gazebo_ros gazebo.launch.py`

![Gazebo界面](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528145818827.png)

**为保证模型顺利加载，请将离线模型下载并放置到~/.gazebo/models路径下**

[Gazebo机器人库](https://github.com/osrf/gazebo_models)

#### XACRO机器人模型优化

为URDF的升级格式，加入了更多编程化的实现方法，可以让模型创建更友好

![XACRO格式示例](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528145940535.png)


- 宏定义，一个小车有4个轮子，每个轮子都一样，我们就没必要创建4个一样的link，像函数定义一样，做一个可重复使用的模块就可以了。

- 文件包含，复杂机器人的模型文件可能会很长，为了切分不同的模块，比如底盘、传感器，我们还可以把不同模块的模型放置在不同的文件中，然后再用一个总体文件做包含调用。

- 可编程接口，比如在XACRO模型文件中，定义一些常量，描述机器人的尺寸，定义一些变量，在调用宏定义的时候传递数据，还可以在模型中做数据计算，甚至加入条件语句，比如你的机器人叫A，就有摄像头，如果叫B，就没有摄像头。


#### XACRO文件语法

##### 常量定义

![常量定义](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150321426.png)

`<xacro:property>`标签用来定义一些常量，比如这样定义一个PI的常量名为“M_PI”，值为“3.14159”，在调用的时候，通过`${}`使用常量，里边就可以使用定义好的常量了。

![常量标签](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150331042.png)

##### 数字计算

![数学计算](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150430205.png)

如果需要做数学计算，同样是在“${}”中进行，比如某一个位置，我们可以通过这两个常量做运算得到，就加入了加法和除法运算。

![数字计算代码](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150435206.png)

在移动机器人的模型中，很多有相对关系的数据，我们尽量都改成公式计算，如果直接写结果的数值，未来修改的时候，可能根本想不起来这个数据是怎么来的。

**所有数学运算都会转换成浮点数进行，以保证运算精度**

##### 宏定义

![宏定义](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150542643.png)

机器人的轮子我们也做成宏定义，定义方式是通过这个`<xacro:macro>`标签描述的，还可以像函数一样，设置里边会用到的一些参数，比如这里的A、B、C。

当需要使用这个宏的时候，就可以像这样，通过宏名字的标签，来调用，同时要记得把几个参数设置好。

![宏定义代码](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150603346.png)    

##### 文件包含

![文件包含](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150712684.png)

宏定义是可以嵌套的，于是我们把机器人的底盘也做成了一个宏，然后使用另外一个模型文件，对底盘宏定义的文件做了一个包含，然后再调用。

![宏定义代码](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150741631.png)

#### 机器人仿真模型配置

1.完善物理参数

第一步是确保每一个link都有惯性参数和碰撞属性，因为Gazebo是物理仿真平台，必要的物理参数是一定需要的。

![物理参数](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150842585.png)

2.添加Gazebo标签

第二步是为link添加gazebo标签，主要是为了可以在gazebo中渲染每一个link的颜色，因为URDF中的颜色系统和gazebo中的不同，所以得做一步这样的冗余配置。

![Gazebo标签](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150931153.png)

3.配置传动装置

第三步是要给运动的joint配置传动装置，可以理解为仿真了一个电机。

![传动装置](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528150958309.png)

4.添加控制器插件

第四步，要添加一个gazebo的控制器插件，小车是差速控制的，那就添加差速控制器插件，这样在不同角度下两个电机的速度分配，就可以交给控制器插件来完成了。

![控制器插件](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.4_Gazebo/image-20220528151019063.png)

#### 构建仿真环境

接下来就考虑如何把模型加载到Gazebo中了，需要用到一个gazebo提供的功能节点`spwan_entity`。

`learning_gazebo/launch/load_urdf_into_gazebo.launch.py`

```python
import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='learning_gazebo' #<--- CHANGE ME
    world_file_path = 'worlds/neighborhood.world'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_path = os.path.join(pkg_path, world_file_path)  

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

    mbot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','mbot.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'world':world_path}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mbot',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        mbot,
        gazebo,
        spawn_entity,
    ])
```

##### 机器人模型代码

`learning_gazebo/urdf/mbot_gazebo.xacro`

```xml
<?xml version="1.0"?>
<robot name="mbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find learning_gazebo)/urdf/mbot_base_gazebo.xacro" />

    <xacro:mbot_base_gazebo/>

</robot>
```

`learning_gazebo/urdf/mbot_base_gazebo.xacro`

```xml
<?xml version="1.0"?>
<robot name="mbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- PROPERTY LIST -->
    <xacro:property name="M_PI" value="3.1415926"/>
    <xacro:property name="base_mass"   value="1" /> 
    <xacro:property name="base_radius" value="0.20"/>
    <xacro:property name="base_length" value="0.16"/>

    <xacro:property name="wheel_mass"   value="0.2" />
    <xacro:property name="wheel_radius" value="0.06"/>
    <xacro:property name="wheel_length" value="0.025"/>
    <xacro:property name="wheel_joint_y" value="0.19"/>
    <xacro:property name="wheel_joint_z" value="0.05"/>

    <xacro:property name="caster_mass"    value="0.2" /> 
    <xacro:property name="caster_radius"  value="0.015"/> <!-- wheel_radius - ( base_length/2 - wheel_joint_z) -->
    <xacro:property name="caster_joint_x" value="0.18"/>

    <!-- Defining the colors used in this robot -->
    <material name="yellow">
        <color rgba="1 0.4 0 1"/>
    </material>
    <material name="black">
        <color rgba="0 0 0 0.95"/>
    </material>
    <material name="gray">
        <color rgba="0.75 0.75 0.75 1"/>
    </material>

    <!-- Macro for inertia matrix -->
    <xacro:macro name="sphere_inertial_matrix" params="m r">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                iyy="${2*m*r*r/5}" iyz="0" 
                izz="${2*m*r*r/5}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                izz="${m*r*r/2}" /> 
        </inertial>
    </xacro:macro>

    <!-- Macro for robot wheel -->
    <xacro:macro name="wheel" params="prefix reflect">
        <joint name="${prefix}_wheel_joint" type="continuous">
            <origin xyz="0 ${reflect*wheel_joint_y} ${-wheel_joint_z}" rpy="0 0 0"/>
            <parent link="base_link"/>
            <child link="${prefix}_wheel_link"/>
            <axis xyz="0 1 0"/>
        </joint>

        <link name="${prefix}_wheel_link">
            <visual>
                <origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
                <geometry>
                    <cylinder radius="${wheel_radius}" length = "${wheel_length}"/>
                </geometry>
                <material name="gray" />
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="${M_PI/2} 0 0" />
                <geometry>
                    <cylinder radius="${wheel_radius}" length = "${wheel_length}"/>
                </geometry>
            </collision>
            <xacro:cylinder_inertial_matrix  m="${wheel_mass}" r="${wheel_radius}" h="${wheel_length}" />
        </link>

        <gazebo reference="${prefix}_wheel_link">
            <material>Gazebo/Gray</material>
        </gazebo>

        <!-- Transmission is important to link the joints and the controller -->
        <transmission name="${prefix}_wheel_joint_trans">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="${prefix}_wheel_joint" >
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            </joint>
            <actuator name="${prefix}_wheel_joint_motor">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>
    </xacro:macro>

    <!-- Macro for robot caster -->
    <xacro:macro name="caster" params="prefix reflect">
        <joint name="${prefix}_caster_joint" type="fixed">
            <origin xyz="${reflect*caster_joint_x} 0 ${-(base_length/2 + caster_radius)}" rpy="0 0 0"/>
            <parent link="base_link"/>
            <child link="${prefix}_caster_link"/>
        </joint>

        <link name="${prefix}_caster_link">
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <sphere radius="${caster_radius}" />
                </geometry>
                <material name="black" />
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <sphere radius="${caster_radius}" />
                </geometry>
            </collision>      
            <xacro:sphere_inertial_matrix  m="${caster_mass}" r="${caster_radius}" />
        </link>

        <gazebo reference="${prefix}_caster_link">
            <material>Gazebo/Black</material>
        </gazebo>
    </xacro:macro>

    <xacro:macro name="mbot_base_gazebo">
        <link name="base_footprint">
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <geometry>
                    <box size="0.001 0.001 0.001" />
                </geometry>
            </visual>
        </link>
        <gazebo reference="base_footprint">
            <turnGravityOff>false</turnGravityOff>
        </gazebo>

        <joint name="base_footprint_joint" type="fixed">
            <origin xyz="0 0 ${base_length/2 + caster_radius*2}" rpy="0 0 0" />        
            <parent link="base_footprint"/>
            <child link="base_link" />
        </joint>

        <link name="base_link">
            <visual>
                <origin xyz=" 0 0 0" rpy="0 0 0" />
                <geometry>
                    <cylinder length="${base_length}" radius="${base_radius}"/>
                </geometry>
                <material name="yellow" />
            </visual>
            <collision>
                <origin xyz=" 0 0 0" rpy="0 0 0" />
                <geometry>
                    <cylinder length="${base_length}" radius="${base_radius}"/>
                </geometry>
            </collision>   
            <xacro:cylinder_inertial_matrix  m="${base_mass}" r="${base_radius}" h="${base_length}" />
        </link>

        <gazebo reference="base_link">
            <material>Gazebo/Blue</material>
        </gazebo>

        <xacro:wheel prefix="left"  reflect="1"/>
        <xacro:wheel prefix="right" reflect="-1"/>

        <xacro:caster prefix="front" reflect="-1"/>
        <xacro:caster prefix="back"  reflect="1"/>

        <!-- controller -->
        <gazebo>
            <plugin name="differential_drive_controller" 
                    filename="libgazebo_ros_diff_drive.so">                
                  <update_rate>30</update_rate>
                  <left_joint>left_wheel_joint</left_joint>
                  <right_joint>right_wheel_joint</right_joint>
                  <wheel_separation>${wheel_joint_y*2}</wheel_separation>
                  <wheel_diameter>${2*wheel_radius}</wheel_diameter>
                  <max_wheel_torque>20</max_wheel_torque>
                  <max_wheel_acceleration>1.0</max_wheel_acceleration>
                  <command_topic>cmd_vel</command_topic>
                  <publish_odom>true</publish_odom>
                  <publish_odom_tf>true</publish_odom_tf>
                  <publish_wheel_tf>true</publish_wheel_tf>
                  <odometry_topic>odom</odometry_topic>
                  <odometry_frame>odom</odometry_frame>
                  <robot_base_frame>base_footprint</robot_base_frame>
                  <odometry_source>1</odometry_source>
            </plugin>
        </gazebo> 
    </xacro:macro>

</robot>
```

### Rviz：三维可视化显示平台

Rviz的核心框架是基于Qt可视化工具打造的一个开放式平台，官方出厂就自带了很多机器人常用的可视化显示插件，只要我们按照ROS中的消息发布对应的话题，就可以看到图形化的效果了。如果我们对显示的效果不满意，或者想添加某些新的显示项，也可以在Rviz这个平台中，开发更多可视化效果，方便打造我们自己的上位机。

启动Rviz

`ros2 run rviz2 rviz2`

![Rviz界面](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/image/3.5_Rviz/image-20220528152204601.png)

#### 彩色相机仿真与可视化

摄像头肯定是最为常用的一种传感器了，我们先来给机器人装上摄像头。

**仿真插件配置**

关于传感器的仿真，都需要使用Gazebo提供的插件，摄像头对应的插件叫做`libgazebo_ros_camera.so`，我们对照模型的代码给大家介绍这个插件的使用方法。

`learning_gazebo/urdf/sensers/camera_gazebo.xacro`

```XML
<gazebo reference="${prefix}_link">
    <sensor type="camera" name="camera_node">
        <update_rate>30.0</update_rate>
        <camera name="head">
            <horizontal_fov>1.3962634</horizontal_fov>
            <image>
                <width>1280</width>
                <height>720</height>
                <format>R8G8B8</format>
            </image>
            <clip>
                <near>0.02</near>
                <far>300</far>
            </clip>
            <noise>
                <type>gaussian</type>
                <mean>0.0</mean>
                <stddev>0.007</stddev>
            </noise>
        </camera>
        <plugin name="gazebo_camera" filename="libgazebo_ros_camera.so">
            <ros>
                <!-- <namespace>stereo</namespace> -->
                <remapping>~/image_raw:=image_raw</remapping>
                <remapping>~/camera_info:=camera_info</remapping>
            </ros>
            <camera_name>${prefix}</camera_name>
            <frame_name>${prefix}_link</frame_name>
            <hack_baseline>0.2</hack_baseline>
        </plugin>
    </sensor>
</gazebo>
```

主要配置项如下：

- `<sensor>`标签：描述传感器

    * type：传感器类型，camera

    * name：摄像头命名，自由设置

- `<camera>`标签：描述摄像头参数

    分辨率，编码格式，图像范围，噪音参数等

- `<plugin>`标签：加载摄像头仿真插件

#### 图像数据可视化

我们使用Rviz可视化显示图像信息，先来启动Rviz：

`$ ros2 run rviz2 rviz2`

启动成功后，在左侧Displays窗口中点击“Add”，找到Image显示项，OK确认后就可以加入显示列表啦，然后配置好该显示项订阅的图像话题，就可以顺利看到机器人的摄像头图像啦。

#### 三维相机仿真与可视化

二维摄像头不过瘾，想不想试试三维相机，比如我们常用的Kinect体感传感器，或者Intel的Realsense，可以获取外部环境的点云数据。这种相机的价格比usb摄像头可贵不少，不过我们也可以通过仿真，一分钱不用，就可以玩起来。

仿真插件配置

三维相机使用的Gazebo插件也是`libgazebo_ros_camera.so`，配置方法如下：

`learning_gazebo/urdf/sensers/kinect_gazebo.xacro`

```xml
<gazebo reference="${prefix}_link">
    <sensor type="depth" name="${prefix}">
        <always_on>true</always_on>
        <update_rate>15.0</update_rate>
        <pose>0 0 0 0 0 0</pose>
        <camera name="kinect">
            <horizontal_fov>${60.0*M_PI/180.0}</horizontal_fov>
            <image>
                <format>R8G8B8</format>
                <width>640</width>
                <height>480</height>
            </image>
            <clip>
                <near>0.05</near>
                <far>8.0</far>
            </clip>
        </camera>
        <plugin name="${prefix}_controller" filename="libgazebo_ros_camera.so">
            <ros>
                <!-- <namespace>${prefix}</namespace> -->
                <remapping>${prefix}/image_raw:=rgb/image_raw</remapping>
                <remapping>${prefix}/image_depth:=depth/image_raw</remapping>
                <remapping>${prefix}/camera_info:=rgb/camera_info</remapping>
                <remapping>${prefix}/camera_info_depth:=depth/camera_info</remapping>
                <remapping>${prefix}/points:=depth/points</remapping>
            </ros>
            <camera_name>${prefix}</camera_name>
            <frame_name>${prefix}_frame_optical</frame_name>
            <hack_baseline>0.07</hack_baseline>
            <min_depth>0.001</min_depth>
            <max_depth>300.0</max_depth>
        </plugin>
    </sensor>
</gazebo>
```

#### 激光雷达仿真与可视化

除了摄像头和三维相机，激光雷达也是很多移动机器人常备的传感器，包括自动驾驶汽车，我们也来试一试。
仿真插件配置

雷达使用的Gazebo插件是`libgazebo_ros_ray_sensor.so`，配置方法如下：

`learning_gazebo/urdf/sensers/lidar_gazebo.xacro`

```xml
<gazebo reference="${prefix}_link">
    <sensor type="ray" name="rplidar">
        <update_rate>20</update_rate>
        <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3</min_angle>
                <max_angle>3</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.10</min>
              <max>30.0</max>
              <resolution>0.01</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
        </ray>
        <plugin name="gazebo_rplidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
    <namespace>/</namespace>
    <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
        </plugin>
    </sensor>
</gazebo>
```

#### 点云数据可视化

运行Rviz：

`$ ros2 run rviz2 rviz2`

同样的流程，点击Add，添加PointCloud2，设置订阅的点云话题，还要配置Rviz的参考系是odom，就可以看到点云数据啦，每一个点都是由xyz位置和rgb颜色组成。

#### Gazebo与Rviz区别

- Gazebo是**仿真平台**，核心功能是创造数据，我们没有机器人或者传感器，它可以帮我们做一个虚拟的数据；

- Rviz是**可视化平台**，核心功能是显示数据，如果没有数据，它也巧妇难为无米之炊。

所以在很多时候，我们使用Gazebo做机器人仿真的时候，也会启动Rviz来显示仿真环境的信息，如果自己手上有真实机器人的话，Gazebo就用不到了，不过还是会用Rviz显示真实机器人传感器的信息。

### RQT：模块化可视化工具

ROS中的Rviz功能已经很强大了，不过有些场景下，我们可能更需要一些简单的模块化的可视化工具，比如只显示一个摄像头的图像，使用Rviz的话，难免会觉得操作有点麻烦。

此时，我们就会用到ROS提供的另外一种模块化可视化工具——rqt。

它和Rviz一样，也是基于QT可视化工具开发而来，在使用前，我们需要通过这样一句指令进行安装，然后就可以通过`rqt`这个命令启动使用了。