---

- @author: ShiZhong Tan

- @date: 2022-08-16

---

# ROS
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

### 参数（Parameter）

### 动作（Action）

### 分布式通信（Distributed Communicatiion）

### DDS（Data Distribution Service）