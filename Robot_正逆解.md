# Robot_空间转换、正逆解

---
- @author: ShiZhong Tan

- @date: 2022-06-17
---

<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [Robot_空间转换、正逆解](#robot_空间转换-正逆解)
  - [空间姿态表示方法](#空间姿态表示方法)
    - [坐标变换](#坐标变换)
    - [旋转矩阵(Rotation matrix)](#旋转矩阵rotation-matrix)
    - [欧拉角(Euler angles)](#欧拉角euler-angles)
    - [四元数(Quaternion)](#四元数quaternion)
  - [逆运动学求解](#逆运动学求解)
    - [解析法（Analytical Solution）](#解析法analytical-solution)
    - [迭代解 - 雅可比矩阵求逆法（Iterative Method - Jacobian Inverse）](#迭代解-雅可比矩阵求逆法iterative-method-jacobian-inverse)
    - [雅可比矩阵转置法（Jacobian Transpose](#雅可比矩阵转置法jacobian-transpose)
    - [优化法（Optimization-based Solution）](#优化法optimization-based-solution)
  - [雅可比矩阵](#雅可比矩阵)
  - [Robot_Kinematics_matlab](#robot_kinematics_matlab)
    - [坐标系变换](#坐标系变换)
      - [绕轴旋转](#绕轴旋转)
      - [轴角输入](#轴角输入)
      - [欧拉角输入](#欧拉角输入)
      - [四元数输入](#四元数输入)
      - [旋转矩阵输入](#旋转矩阵输入)
      - [齐次矩阵](#齐次矩阵)
      - [Matlab绘制姿态矩阵及移动代码](#matlab绘制姿态矩阵及移动代码)
    - [正运动学(Forward kinematic)](#正运动学forward-kinematic)
    - [逆运动学(Inverse kinematics)](#逆运动学inverse-kinematics)

<!-- /code_chunk_output -->

## 空间姿态表示方法

欧拉角、旋转矩阵、四元数都可以唯一的描述一个坐标系在三维空间中的位置。

### 坐标变换

通常采用笛卡尔坐标系（右手坐标系），**rgb(红绿蓝）依次对应xyz**，描述一个物体在坐标系中的位置和朝向，总是可以等效为描述坐标系之间的关系
绕轴旋转变换
$$
rotx(\alpha)=
\begin{bmatrix}
1 & 0 & 0 \\
0 & cos(\alpha) & -sin(\alpha) \\
0 & sin(\alpha) & cos(\alpha) \\
\end{bmatrix}
$$
$$
roty(\beta)=
\begin{bmatrix}
cos(\beta) & 0 & sin(\beta) \\
0 & 1 & 0 \\
-sin(\beta) & 0 & cos(\beta) \\
\end{bmatrix}
$$
$$
rotz(\gamma)=
\begin{bmatrix}
cos(\gamma) & -sin(\gamma) & 0 \\
sin(\gamma) & cos(\gamma) & 0 \\
0 & 0 & 1 \\
\end{bmatrix}
$$

### 旋转矩阵(Rotation matrix)

旋转矩阵是一个正交矩阵（orthonormal matrix，不仅正交，且每一行每一列的长度都为1），意味着它的转置等于求逆
一个坐标系在空间中的旋转矩阵是唯一的，通常为$3\times3$的一个矩阵

### 欧拉角(Euler angles)

用三个数描述从一个坐标系到另一个坐标系的变换，每个数分别是绕某一个坐标轴转动的角度。
一个坐标系在空间中的旋转矩阵是唯一的，但是采用不同的旋转顺序会得到不同形式的欧拉角，每一次都是采用旋转后的坐标系，而不是原始坐标系（**坐标系附着在物体上**），因此没有约定好旋转轴顺序和坐标系选择的欧拉角是没有意义的。
欧拉角在实际使用中可能会出现 **“万向节死锁”（Gimbal Lock）** 的情况，当机械臂到奇异点（附近）时，末端执行器上很小的姿态变化竟然会导致接近无限大的关节速度。尤其是在机器人末端三个关节经过180度和360度时，很容易导致机器人关节旋转180或者360度运动至当前点。

### 四元数(Quaternion)

使用一个四维向量来表示三维空间中的旋转，包括转角和旋转轴（ijk）

## 逆运动学求解

从从操作空间的end effector position and orientation，求关节空间的joint position

### 解析法（Analytical Solution）

用代数或几何法直接求解，。通常随着自由度上升，求得解析解的难度也越来越大，对于冗余机械臂还需要从几个解中选择合适的解。这种方法通常用在特定几何结构的机械臂，并且有特定的关节位置求解顺序。

### 迭代解 - 雅可比矩阵求逆法（Iterative Method - Jacobian Inverse）

### 雅可比矩阵转置法（Jacobian Transpose

### 优化法（Optimization-based Solution）

## 雅可比矩阵

雅可比矩阵是连接关节空间和操作空间速度的矩阵 $\dot{x}=J\dot{q}$
$$
J=\left[\begin{array}{ccc}
\frac{d x_{1}}{d q_{1}} & \cdots & \frac{d x_{1}}{d q_{n}} \\
\vdots & \ddots & \vdots \\
\frac{d x_{m}}{d q_{1}} & \cdots & \frac{d x_{m}}{d q_{n}}
\end{array}\right]
$$
J为机器人的雅可比矩阵（$6\times6$）第i行第j列表示的物理意义就是当第j个关节运动时，操作空间的第i个平动/转动方向会如何运动

用笛卡尔坐标描述线速度（linear velocity）和角速度（angular velocity）、以机械臂的基准坐标系（Base frame或frame{0}）作为参照系来描述end effector速度所求得的雅可比矩阵，称为基本雅可比矩阵；其它所有表示方法（比如将笛卡尔坐标改为柱坐标、球坐标；角度改为欧拉角或四元数quaternion等）都可由这个基本雅可比矩阵转换得到。

$$
J=\begin{bmatrix}
V_x\\V_y\\V_z\\w_x\\w_y\\w_z
\end{bmatrix}
=\begin{bmatrix}
J_v\\J_w
\end{bmatrix}
$$
$J_v$表示end effector对应的线速度，$J_w$表示对应的角速度。

当用齐次坐标变换矩阵描述六自由度机器人姿态时，$J_v$只需要将齐次变换矩阵的第四列的前三行（表示机器人的末端位置）对六个关节角分别求导就可以得到$3\times6$的矩阵。对于角速度部分，基本雅可比矩阵是以frame{0}为参照系的，为了写出$J_w$，我们需要把每个旋转关节的z轴[0, 0, 1]，从以关节自身坐标系为参照系转换到基准坐标系frame{0}中表示,第$i$个关节对应的便是${}^0_iT$的$z$向量${}^0\hat{z}_i$。另外，对于平移关节，因为平移关节的运动不可能改变end effector的朝向，所以end effector的orientation对平移关节位置的求导一定是0

- 基本雅可比矩阵的上半部分Jv由end effector的位置向量对关节求导得出；

- end effector的位置向量可由正运动学解得到

- 基本雅可比矩阵的下半部分Jw可由每个旋转关节的z轴以基准坐标系为参照系写出的单位向量得到

- 把Jv和Jw合起来可以得到一个m×n的矩阵，其中m是end effector/操作空间的自由度（对于空间机械臂通常m=6），n是机械臂的关节数量

==当雅可比矩阵不满秩，即特征值为0时，机器人处于奇异点==，意味着在机械臂到达那个configuration的瞬间，不管关节怎么运动，end effector在这个方向的速度总为0。

## Robot_Kinematics_matlab

### 坐标系变换

#### 绕轴旋转

$$rotx(\theta),roty(\theta),rotz(\theta)$$

$$
rotx(\alpha)=
\begin{bmatrix}
1 & 0 & 0 \\
0 & cos(\alpha) & -sin(\alpha) \\
0 & sin(\alpha) & cos(\alpha) \\
\end{bmatrix}
$$
$$
roty(\beta)=
\begin{bmatrix}
cos(\beta) & 0 & sin(\beta) \\
0 & 1 & 0 \\
-sin(\beta) & 0 & cos(\beta) \\
\end{bmatrix}
$$
$$
rotz(\gamma)=
\begin{bmatrix}
cos(\gamma) & -sin(\gamma) & 0 \\
sin(\gamma) & cos(\gamma) & 0 \\
0 & 0 & 1 \\
\end{bmatrix}
$$

#### 轴角输入

```
axang2quat
axang2rotm
axang2tform
```

#### 欧拉角输入

```
eul2quat
eul2rotm
eul2tform
```

#### 四元数输入

```
quat2axang
quat2eul
quat2rotm
quat2tform
```

#### 旋转矩阵输入

```
rotm2axang
rotm2eul
rotm2quat
rotm2tform
```

#### 齐次矩阵

```
tform2axang
tform2eul
tform2quat
tform2rotm
tform2trvec
trvec2tform
```

#### Matlab绘制姿态矩阵及移动代码

**trplot**指令绘制当前姿态矩阵

```matlab {.line-numbers}
R = rotx(180)
subplot(1,2,1)
title("origin")
trplot()
subplot(1,2,2)
title("After rotations")
trplot(R)
```

使用**tranimate**可以动态的观看变化过程

```matlab {.line-numbers}
R1 = rotx(0)
R2 = rotx(180)
tranimate(R1,R2)
```

### 正运动学(Forward kinematic)

针对不同的DH方法，先写出该DH方法的齐次变换矩阵，以改进的DH方法为例

$$
{ }_{i}^{i-1} T=\left[\begin{array}{cccc}
\cos \theta_{i} & -\sin \theta_{i} & 0 & a_{i-1} \\
\sin \theta_{i} \cos \alpha_{i-1} & \cos \theta_{i} \cos \alpha_{i-1} & -\sin \alpha_{i-1} & -\sin \alpha_{i-1} d_{i} \\
\sin \theta_{i} \sin \alpha_{i-1} & \cos \theta_{i} \sin \alpha_{i-1} & \cos \alpha_{i-1} & \cos \alpha_{i-1} d_{i} \\
0 & 0 & 0 & 1
\end{array}\right]
$$

六自由度机器人正运动学为

$$
{ }_{6}^{0} T={ }_{1}^{0} T_{2}^{1} T_{3}^{2} T_{4}^{3} T_{5}^{4} T_{6}^{5} T
$$

其次变换矩阵代码

```matlab {.line-numbers}
function T = DH2tform(a, alpha, d, theta)
ct = cos(theta); 
st = sin(theta);
ca = cos(alpha); 
sa = sin(alpha);
T = [ct,   -st,   0,    a;
    st*ca, ct*ca, -sa,  -sa*d;
    st*sa, ct*sa, ca,   ca*d;
    0,  0,  0,  1];
end
```

正运动学代码为

```matlab {.line-numbers}
function T = fk(dhparams, q)
% dhparams: a,alpha,d,theta
T0_1 = DH2tform(dhparams(1,1),dhparams(1,2),dhparams(1,3),dhparams(1,4) + q(1));
T1_2 = DH2tform(dhparams(2,1),dhparams(2,2),dhparams(2,3),dhparams(2,4) + q(2));
T2_3 = DH2tform(dhparams(3,1),dhparams(3,2),dhparams(3,3),dhparams(3,4) + q(3));
T3_4 = DH2tform(dhparams(4,1),dhparams(4,2),dhparams(4,3),dhparams(4,4) + q(4));
T4_5 = DH2tform(dhparams(5,1),dhparams(5,2),dhparams(5,3),dhparams(5,4) + q(5));
T5_6 = DH2tform(dhparams(6,1),dhparams(6,2),dhparams(6,3),dhparams(6,4) + q(6));
​
T{1} = T0_1;
T{2} = T{1}*T1_2;
T{3} = T{2}*T2_3;
T{4} = T{3}*T3_4;
T{5} = T{4}*T4_5;
T{6} = T{5}*T5_6;
​
end
```

### 逆运动学(Inverse kinematics)

一般分为解析解和数值解

以UR5机械臂为例，解析解的代码如下

```matlab {.line-numbers}

function theta=niyundongxue(T)
    %变换矩阵T已知
    %SDH:标准DH参数表求逆解（解析解）
    %部分DH参数表如下，需要求解theta信息
    
    a=[0,-0.42500,-0.39225,0,0,0];
    d=[0.089159,0,0,0.10915,0.09465,0.08230];

    % alpha没有用到,故此逆解程序只适合alpha=[pi/2,0,0,pi/2,-pi/2,0]的情况！
    alpha=[pi/2,0,0,pi/2,-pi/2,0];
    
    nx=T(1,1);ny=T(2,1);nz=T(3,1);
    ox=T(1,2);oy=T(2,2);oz=T(3,2);
    ax=T(1,3);ay=T(2,3);az=T(3,3);
    px=T(1,4);py=T(2,4);pz=T(3,4);
    
    %求解关节角1
    m=d(6)*ay-py;  n=ax*d(6)-px; 
    theta1(1,1)=atan2(m,n)-atan2(d(4),sqrt(m^2+n^2-(d(4))^2));
    theta1(1,2)=atan2(m,n)-atan2(d(4),-sqrt(m^2+n^2-(d(4))^2));
  
    %求解关节角5
    theta5(1,1:2)=acos(ax*sin(theta1)-ay*cos(theta1));
    theta5(2,1:2)=-acos(ax*sin(theta1)-ay*cos(theta1));      
    
    %求解关节角6
    mm=nx*sin(theta1)-ny*cos(theta1); nn=ox*sin(theta1)-oy*cos(theta1);
    %theta6=atan2(mm,nn)-atan2(sin(theta5),0);
    theta6(1,1:2)=atan2(mm,nn)-atan2(sin(theta5(1,1:2)),0);
    theta6(2,1:2)=atan2(mm,nn)-atan2(sin(theta5(2,1:2)),0);
    
    %求解关节角3
    mmm(1,1:2)=d(5)*(sin(theta6(1,1:2)).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6(1,1:2)).*(ox*cos(theta1)+oy*sin(theta1))) ...
        -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
    nnn(1,1:2)=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6(1,1:2))+nz*sin(theta6(1,1:2)));
    mmm(2,1:2)=d(5)*(sin(theta6(2,1:2)).*(nx*cos(theta1)+ny*sin(theta1))+cos(theta6(2,1:2)).*(ox*cos(theta1)+oy*sin(theta1))) ...
        -d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1);
    nnn(2,1:2)=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6(2,1:2))+nz*sin(theta6(2,1:2)));
    theta3(1:2,:)=acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
    theta3(3:4,:)=-acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
    
    %求解关节角2
    mmm_s2(1:2,:)=mmm;
    mmm_s2(3:4,:)=mmm;
    nnn_s2(1:2,:)=nnn;
    nnn_s2(3:4,:)=nnn;
    s2=((a(3)*cos(theta3)+a(2)).*nnn_s2-a(3)*sin(theta3).*mmm_s2)./ ...
        ((a(2))^2+(a(3))^2+2*a(2)*a(3)*cos(theta3));
    c2=(mmm_s2+a(3)*sin(theta3).*s2)./(a(3)*cos(theta3)+a(2));
    theta2=atan2(s2,c2);   
    
    %整理关节角1 5 6 3 2
    theta(1:4,1)=theta1(1,1);theta(5:8,1)=theta1(1,2);
    theta(:,2)=[theta2(1,1),theta2(3,1),theta2(2,1),theta2(4,1),theta2(1,2),theta2(3,2),theta2(2,2),theta2(4,2)]';
    theta(:,3)=[theta3(1,1),theta3(3,1),theta3(2,1),theta3(4,1),theta3(1,2),theta3(3,2),theta3(2,2),theta3(4,2)]';
    theta(1:2,5)=theta5(1,1);theta(3:4,5)=theta5(2,1);
    theta(5:6,5)=theta5(1,2);theta(7:8,5)=theta5(2,2);
    theta(1:2,6)=theta6(1,1);theta(3:4,6)=theta6(2,1);
    theta(5:6,6)=theta6(1,2);theta(7:8,6)=theta6(2,2); 
    
    %求解关节角4
    theta(:,4)=atan2(-sin(theta(:,6)).*(nx*cos(theta(:,1))+ny*sin(theta(:,1)))-cos(theta(:,6)).* ...
        (ox*cos(theta(:,1))+oy*sin(theta(:,1))),oz*cos(theta(:,6))+nz*sin(theta(:,6)))-theta(:,2)-theta(:,3);  
    
end
```

测试代码

```matlab {.line-numbers}
clc;clear
 
%% 初始化关节角度
%theta=[1,1/2,1/3,1,1,1];
theta=[1,1/2,1,1,1,1];
%theta=[1,1,1,1,1,1];

%% 求运动学逆解
niyundongxue(TT)
```
