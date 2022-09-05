


和陀螺仪测量信息 $w_{t}:$
$$
\begin{gathered}
a_{t}=a_{t(\text { real })}+b_{a_{t}}+R_{w}^{t} g^{w}+n_{a} \\
\end{gathered}\tag{1}
$$
$$
\begin{gathered}
w_{t}=w_{t(\text { real })}+b_{w_{t}}+n_{w}
\end{gathered}\tag{2}
$$
**IMU** 的测量信息是在本体坐标系 (即 **IMU** 坐标) 中获取的。理论上真实的加速度 $a_{t(\text { real })}$ 受加速度偏置 $b_{a_{t}}$ 和噪声 $n_{a}$ 的影响, 它们之和构成了最终的加速度测量值 $a_{t}$ 。理论 上体轴的角速度 $w_{t(\text { real })}$ 受陀螺仪偏置 $b_{w_{t}}$ 和噪声 $n_{w}$ 的影响, 它们之和构成了最终的角速度 测量值 $w_{t}$ 。

假设噪声 $n_{a}$ 和 $n_{w}$ 服从高斯分布:
$$
\begin{aligned}
&n_{a} \sim N\left(0, \sigma_{a}^{2}\right) \\
&n_{w} \sim N\left(0, \sigma_{w}^{2}\right)
\end{aligned}\tag{3}
$$
加速度偏置 $b_{a_{t}}$ 和陀螺仪偏置 $b_{w_{t}}$ 被建模为随机游走:
$$
\begin{aligned}
&\dot{b}_{a_{t}}=n_{b_{a}} \\
&\dot{b}_{w_{t}}=n_{b_{w}}
\end{aligned}\tag{4}
$$
其中 $n_{b_{a}}$ 和 $n_{b_{w}}$ 服从高斯分布:
$$
\begin{aligned}
&n_{b_{a}} \sim N\left(0, \sigma_{b_{a}}^{2}\right) \\
&n_{b_{w}} \sim N\left(0, \sigma_{b_{w}}^{2}\right)
\end{aligned}\tag{5}
$$
对于连续两个关键帧 $b_{k}$ 和 $b_{k+1}$, 它们对应的时刻分别为 $t_{k} 、 t_{k+1}$ 。可以根据 $\left[t_{k}, t_{k+1}\right]$ 时 间间隔内 IMU 的测量值, 对系统的位置、速度和旋转等状态进行传播**（注意这里的四 元数采用了实部在后, 虚部在前的形式):**
$$
\begin{aligned}
p_{b_{k+1}}^{w}=p_{b_{k}}^{w} &+v_{b_{k}}^{w} \Delta t_{k}+\iint_{t \in\left[t_{k}, t_{k+1}\right]}\left(R_{t}^{w}\left(a_{t}-b_{a_{t}}-n_{a}\right)-g^{w}\right) d t^{2} \\
\end{aligned}\tag{6}
$$
$$
\begin{aligned}
v_{b_{k+1}}^{w} &=v_{b_{k}}^{w}+\int_{t \in\left[t_{k}, t_{k+1}\right]}\left(R_{t}^{w}\left(a_{t}-b_{a_{t}}-n_{a}\right)-g^{w}\right) d t \\
\end{aligned}\tag{7}
$$
$$
\begin{aligned}
q_{b_{k+1}}^{w} &=q_{b_{k}}^{w} \otimes \int_{t \in\left[t_{k}, t_{k+1}\right]} \frac{1}{2} q_{t}^{b_{k}} \otimes\left(w_{t}-b_{w_{t}}-n_{w}\right) d t \\

&=q_{b_{k}}^{w} \otimes \int_{t \in\left[t_{k}, t_{k+1}\right]} \frac{1}{2} \Omega\left(w_{t}-b_{w_{t}}-n_{w}\right) q_{t}^{b_{k}} d t
\end{aligned}\tag{8}
$$

其中:
$$
\Omega(w)=\left[\begin{array}{cc}
-[w]_{\times} & w \\
-w^{T} & 0
\end{array}\right],[w]_{\times}=\left[\begin{array}{ccc}
0 & -w_{z} & w_{y} \\
w_{z} & 0 & -w_{x} \\
-w_{y} & w_{x} & 0
\end{array}\right]\tag{9}
$$
$\Delta t_{k}$ 是 $\left[t_{k}, t_{k+1}\right]$ 之间的时间间隔, **$R_{t}^{w}$** 为 $t$ 时刻从**本体坐标系到世界坐标系**的旋转矩阵,$q_{t}^{b_{k}}$ 为用四元数表示的 $t$ 时刻从**本体坐标系到世界坐标系**的旋转。

从等式(6)(7)(8)可以看出, 系统位置、速度和旋转等状态的传播需要关键帧 $b_{k}$ 时刻 的位置 $p_{b_{k}}^{w}$ 、速度 $v_{b_{k}}^{w}$ 和旋转 $q_{b_{k}}^{w}$, 当这些起始状态发生改变时, 就需要按照等式(6)(7)(8)重 新进行状态传播。在基于优化的算法中, 每个关键帧时刻的状态需要频繁调整, 所以就 需要频繁地重新积分, 这样会浪费大量的计算资源。IMU 预积分就是为了避免这种计算 资源上的浪费。

IMU 预积分的思路简单来说, 就是将参考坐标系从世界坐标系 $w$ 调整为第 $k$ 个关键 帧时刻的本体坐标系 $b_{k}$ :
$$
\begin{gathered}
R_{w}^{b_{k}} p_{b_{k+1}}^{w}=R_{w}^{b_{k}}\left(p_{b_{k}}^{w}+v_{b_{k}}^{w} \Delta t_{k}-\frac{1}{2} g^{w} \Delta t_{k}^{2}\right)+\alpha_{b_{k+1}}^{b_{k}} \\
\end{gathered}\tag{10}
$$
$$
\begin{gathered}
R_{w}^{b_{k}} v_{b_{k+1}}^{w}=R_{w}^{b_{k}}\left(v_{b_{k}}^{w}-g^{w} \Delta t_{k}\right)+\beta_{b_{k+1}}^{b_{k}} \\
\end{gathered}\tag{11}
$$
$$
\begin{gathered}
q_{w}^{b_{k}} \otimes q_{b_{k+1}}^{w}=\gamma_{b_{k+1}}^{b_{k}}
\end{gathered}\tag{12}
$$

对于公式(10),(11),两边同时乘以$R_{w}^{b_{k}}$,对于(12)左边乘以$q_{w}^{b_{k}}$

其中：
$$
\begin{aligned}
\alpha_{b_{k+1}}^{b_{k}} &=\iint_{t \in\left[t_{k}, t_{k+1}\right]}\left(R_{t}^{b_{k}}\left(a_{t}-b_{a_{t}}-n_{a}\right)\right) d t^{2} \\
\end{aligned}\tag{13}
$$

$$
\begin{aligned}
\beta_{b_{k+1}}^{b_{k}} &=\int_{t \in\left[t_{k}, t_{k+1}\right]}\left(R_{t}^{b_{k}}\left(a_{t}-b_{a_{t}}-n_{a}\right)\right) d t \\
\end{aligned}\tag{14}
$$
$$
\begin{aligned}
\gamma_{b_{k+1}}^{b_{k}} &=\int_{t \in\left[t_{k}, t_{k+1}\right]} \frac{1}{2} \Omega\left(w_{t}-b_{w_{t}}-n_{w}\right) \gamma_{t}^{b_{k}} d t
\end{aligned}\tag{15}
$$

积分项 $(13)(14)(15)$ 中的参考坐标系变成了 $b_{k}$, 可以理解为这时的积分结果为 $b_{k+1}$ 对 于 $b_{k}$ 的相对运动量, 即使在优化过程中对关键帧的位置、速度和旋转等状态进行调整, 也不对积分项 $(13)(14)(15)$ 产生任何影响, 从而避免了重复积分。

## 连续时间状态方程

**真实状态**用 $\alpha_{t}^{b_{k}} 、 \beta_{t}^{b_{k}} 、 \gamma_{t}^{b_{k}} 、 b_{a_{t}}$ 和 $b_{w_{t}}$ 表示系统 它们都是**去除噪声后的理想状态**。

**标称状态**代表大信号, **末去除噪声**, 用 $\hat{\alpha}_{t}^{b_{k}} 、 \hat{\beta}_{t}^{b_{k}} 、 \hat{\gamma}_{t}^{b_{k}} 、 \hat{b}_{a_{t}}$ 和 $\hat{b}_{w_{t}}$ 表示,是我们拥有的，也可以理解为传感器测得的数据。

**误差状态**表示小信号, 用 $\delta \alpha_{t}^{b_{k}} 、 \delta \beta_{t}^{b_{k}} 、 \delta \theta_{t}^{b_{k}} 、 \delta b_{a_{t}}$ 和 $\delta b_{w_{t}}$ 表示, 噪声带来的误差累积由误差状态的状态方程来描述。

**真实状态**可以分解为**标称状态**和**误差状态**的组合。

真实状态与标称状态、 误差状态之间的关系由公式 $(16)(17)(18)(19)(20)$ 描述。
$$
\begin{gathered}
\alpha_{t}^{b_{k}}=\hat{\alpha}_{t}^{b_{k}}+\delta \alpha_{t}^{b_{k}} \\
\end{gathered}\tag{16}
$$
$$
\begin{gathered}
\beta_{t}^{b_{k}}=\hat{\beta}_{t}^{b_{k}}+\delta \beta_{t}^{b_{k}} \\
\end{gathered}\tag{17}
$$
$$
\begin{gathered}
\gamma_{t}^{b_{k}}=\hat{\gamma}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}} \\
=\hat{\gamma}_{t}^{b_{k}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{t}^{b_{k}}
\end{array}\right] \\
\end{gathered}\tag{18}
$$
$$
\begin{gathered}
b_{a_{t}}=\hat{b}_{a_{t}}+\delta b_{a_{t}}
\end{gathered}\tag{19}
$$

$$
b_{w_{t}}=\hat{b}_{w_{t}}+\delta b_{w_{t}}\tag{20}
$$

## 真实状态(true-state)的状态方程
$$
\begin{gathered}
\dot{\alpha}_{t}^{b_{k}}=\beta_{t}^{b_{k}} \\
\end{gathered}\tag{21}
$$
$$
\begin{gathered}
\dot{\beta}_{t}^{b_{k}}=R_{t}^{b_{k}}\left(a_{t}-b_{a_{t}}-n_{a}\right) \\
\end{gathered}\tag{22}
$$
$$
\begin{gathered}
\dot{\gamma}_{t}^{b_{k}}=\frac{1}{2} \gamma_{t}^{b_{k}} \otimes\left(w_{t}-b_{w_{t}}-n_{w}\right) \\
=\frac{1}{2} \Omega\left(w_{t}-b_{w_{t}}-n_{w}\right) \gamma_{t}^{b_{k}} \\
\end{gathered}\tag{23}
$$
$$
\begin{gathered}
\dot{b}_{a_{t}}=n_{b_{a}} \\
\end{gathered}\tag{24}
$$
$$
\begin{gathered}
\dot{b}_{w_{t}}=n_{b_{w}}
\end{gathered}\tag{25}
$$
其中(23)为什么可以化简为这个样子，请看(8)
## 标称状态(nominal-state)的状态方程

$$
\begin{gathered}
\dot{\hat{\alpha}}_{t}^{b_{k}}={\hat{\beta}}_{t}^{b_{k}} \\
\end{gathered}\tag{26}
$$
$$
\begin{gathered}
\dot{\hat{\beta}}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}}\left(a_{t}-\hat{b}_{a_{t}}\right) \\
\end{gathered}\tag{27}
$$
$$
\begin{gathered}
\dot{\hat{\gamma}}_{t}^{b_{k}}=\frac{1}{2} \hat{\gamma}_{t}^{b_{k}} \otimes\left(w_{t}-\hat{b}_{w_{t}}\right) \\


=\frac{1}{2} \Omega\left(w_{t}-\hat{b}_{w_{t}}\right) \hat{\gamma}_{t}^{b_{k}} \\
\end{gathered}\tag{28}
$$
$$
\begin{gathered}
\dot{\hat{b}}_{a_{t}}=0 \\
\end{gathered}\tag{29}
$$
$$
\begin{gathered}
\dot{\hat{b}}_{w_{t}}=0
\end{gathered}\tag{30}
$$

等式(29)(30)实际上表明了，在实际使用 IMU 信息进行积分时，加速度计和陀螺仪 的偏置一般是保持不变的，只有在得到优化时，它们的值才会发生改变。

其中(28)为什么可以化简为这个样子，请看(8)

## 误差状态(error-state)的状态方程
先给出误差状态的状态方程，后面再逐一推导：
$$
\begin{gathered}
\delta \dot{\alpha}_{t}^{b_{k}}=\alpha_{t}^{b_{k}}-\hat{\alpha}_{t}^{b_{k}} \\

=\delta \beta_{t}^{b_{k}}
\end{gathered}\tag{31}
$$
$$
\begin{gathered}
\delta \dot{\beta}_{t}^{b_{k}}=-\hat{R}_{t}^{b_{k}}\left[a_{t}-\hat{b}_{a_{t}}\right]_{\times} \delta \theta_{t}^{b_{k}}-\hat{R}_{t}^{b_{k}} \delta b_{a_{t}}-\hat{R}_{t}^{b_{k}} n_{a}
\end{gathered}\tag{32}
$$

$$
\begin{gathered}
\delta \dot{\theta}_{t}^{b_{k}}=-\left[w_{t}-\hat{b}_{w_{t}}\right]_{x} \delta \theta_{t}^{b_{k}}-\delta b_{w_{t}}-n_{w} \\
\end{gathered}\tag{33}
$$
$$
\begin{gathered}
\delta \dot{b}_{a_{t}}
=n_{b_{a}} \\
\end{gathered}\tag{34}
$$
$$
\begin{gathered}
\delta \dot{b}_{w_{t}}=n_{b_{w}}
\end{gathered}\tag{35}
$$

将误差状态的状态方程写成矩阵形式:
$$
\left[\begin{array}{c}
\delta \dot{\alpha}_{t}^{b_{k}} \\
\delta \dot{\beta}_{t}^{b_{k}} \\
\delta \dot{\theta}_{t}^{b_{k}} \\
\delta \dot{b}_{a_{t}} \\
\delta \dot{b}_{w_{t}}
\end{array}\right]=\left[\begin{array}{ccccc}
0 & I & 0 & 0 & 0 \\
0 & 0 & -\hat{R}_{t}^{b_{k}}\left[a_{t}-\hat{b}_{a_{t}}\right]_{\times} & -\hat{R}_{t}^{b_{k}} & 0 \\
0 & 0 & -\left[w_{t}-\hat{b}_{w_{t}}\right]_{\times} & 0 & -I \\
0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0
\end{array}\right]\left[\begin{array}{l}
\delta \alpha_{t}^{b_{k}} \\
\delta \beta_{t}^{b_{k}} \\
\delta \theta_{t}^{b_{k}} \\
\delta b_{a_{t}} \\
\delta b_{w_{t}}
\end{array}\right]+\left[\begin{array}{cccc}
0 & 0 & 0 & 0 \\
-\hat{R}_{t}^{b_{k}} & 0 & 0 & 0 \\
0 & -I & 0 & 0 \\
0 & 0 & I & 0 \\
0 & 0 & 0 & I
\end{array}\right]\left[\begin{array}{l}
n_{a} \\
n_{w} \\
n_{b_{a}} \\
n_{b_{w}}
\end{array}\right]\\
=F_{t} \delta z_{t}^{b_{k}}+G_{t} n_{t}\tag{36}
$$
公式(31)(34)(35)是显而易见的，因为这些公式都是线性关系

公式(32)(33)的推导稍微复杂。

先进行等式(32)的推导：
$$
R_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}}\left(I+\left[\delta \theta_{t}^{b_{k}}\right]_{x}\right)+O\left(\left\|\delta \theta_{t}^{b_{k}}\right\|^{2}\right)\tag{37}
$$
为了表述上的方便，定义(此处的$a$,表示的是真实状态下的加速度)：
$$
\begin{gathered}
a=a_{t}-b_{a_{t}}-n_{a} \\
\end{gathered}\tag{38}
$$
$$
\begin{gathered}
\hat{a}=a_{t}-\hat{b}_{a_{t}}
\end{gathered}\tag{39}
$$

$a$ 和 $\hat{a}$ 存在以下关系:
$$
a=\hat{a}+\delta a\tag{40}
$$
可以得到:
$$
\begin{aligned}
\delta a &=a-\hat{a} \\
&=\left(a_{t}-b_{a_{t}}-n_{a}\right)-\left(a_{t}-\hat{b}_{a_{t}}\right) \\
&=-\left(b_{a_{t}}-\hat{b}_{a_{t}}\right)-n_{a} \\
&=-\delta b_{a_{t}}-n_{a}
\end{aligned}\tag{41}
$$
将等式（39）$\hat{a}=a_{t}-\hat{b}_{a_{t}}$代入等式（27）$\dot{\hat{\beta}}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}}\left(a_{t}-\hat{b}_{a_{t}}\right)$：
$$
\dot{\hat{\beta}}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}} \hat{a}\tag{42}
$$
根据(40)$a=\hat{a}+\delta a$将等式(22)$\dot{\beta}_{t}^{b_{k}}=R_{t}^{b_{k}}\left(a_{t}-b_{a_{t}}-n_{a}\right)$中等号右边括号内的项替换为 $\hat{a}+\delta a$ :
$$
\dot{\beta}_{t}^{b_{k}}=R_{t}^{b_{k}}(\hat{a}+\delta a)\tag{43}
$$
将等式(37)$R_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}}\left(I+\left[\delta \theta_{t}^{b_{k}}\right]_{\times}\right)+O\left(\left\|\delta \theta_{t}^{b_{k}}\right\|^{2}\right)$代入等式(43)$\dot{\beta}_{t}^{b_{k}}=R_{t}^{b_{k}}(\hat{a}+\delta a)$, 并忽略 $\delta \theta_{t}^{b_{k}}$ 的二阶及二阶以上的项:
$$
\begin{aligned}
\dot{\beta}_{t}^{b_{k}} &=\hat{R}_{t}^{b_{k}}\left(I+\left[\delta \theta_{t}^{b_{k}}\right]_{x}\right)(\hat{a}+\delta a) \\
&=\hat{R}_{t}^{b_{k}} \hat{a}+\hat{R}_{t}^{b_{k}} \delta a+\hat{R}_{t}^{b_{k}}\left[\delta \theta_{t}^{b_{k}}\right]_{x} \hat{a}+\hat{R}_{t}^{b_{k}}\left[\delta \theta_{t}^{b_{k}}\right]_{x} \delta a
\end{aligned}\tag{44}
$$
根据等式(17)$\beta_{t}^{b_{k}}=\hat{\beta}_{t}^{b_{k}}+\delta \beta_{t}^{b_{k}}$对两边求导可得：
$$
\dot{\beta}_{t}^{b_{k}}=\dot{\hat{\beta}}_{t}^{b_{k}}+\delta \dot{\beta}_{t}^{b_{k}}\tag{45}
$$
将等式(42) $\dot{\hat{\beta}}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}} \hat{a}$代入等式(45) $\dot{\beta}_{t}^{b_{k}}=\dot{\hat{\beta}}_{t}^{b_{k}}+\delta \dot{\beta}_{t}^{b_{k}}$可得：
$$
\begin{aligned}
\dot{\beta}_{t}^{b_{k}} &=\dot{\hat{\beta}}_{t}^{b_{k}}+\delta \dot{\beta}_{t}^{b_{k}} \\
&=\hat{R}_{t}^{b_{k}} \hat{a}+\delta \dot{\beta}_{t}^{b_{k}}
\end{aligned}\tag{46}
$$
将等式(44)与等式(46)联立：
$$
\hat{R}_{t}^{b_{k}} \hat{a}+\delta \dot{\beta}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}} \hat{a}+\hat{R}_{t}^{b_{k}} \delta a+\hat{R}_{t}^{b_{k}}\left[\delta \theta_{t}^{b_{k}}\right]_{x} \hat{a}+\hat{R}_{t}^{b_{k}}\left[\delta \theta_{t}^{b_{k}}\right]_{x} \delta a \tag{47}
$$
化简上式得：
$$
\delta \dot{\beta}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}} \delta a+\hat{R}_{t}^{b_{k}}\left[\delta \theta_{t}^{b_{k}}\right]_{x} \hat{a}+\hat{R}_{t}^{b_{k}}\left[\delta \theta_{t}^{b_{k}}\right]_{x} \delta a \tag{48}
$$
忽略等式(48)中的二阶小量, 并且调整叉乘项的顺序 $\left([a]_{x} b=-[b]_{x} a\right)$ :
$$
\delta \dot{\beta}_{t}^{b_{k}}=\hat{R}_{t}^{b_{k}} \delta a-\hat{R}_{t}^{b_{k}}[\hat{a}]_{\times} \delta \theta_{t}^{b_{k}}\tag{49}
$$
再将等式(39) $\hat{a}=a_{t}-\hat{b}_{a_{t}}$(41) $\delta a=-\delta b_{a_{t}}-n_{a}$代入等式(49)：
$$
\begin{aligned}
\delta \dot{\beta}_{t}^{b_{k}} &=\hat{R}_{t}^{b_{k}} \delta a-\hat{R}_{t}^{b_{k}}[\hat{a}]_{\times} \delta \theta_{t}^{b_{k}} \\
&=-\hat{R}_{t}^{b_{k}}\left[a_{t}-\hat{b}_{a_{t}}\right]_{\times} \delta \theta_{t}^{b_{k}}-\hat{R}_{t}^{b_{k}} \delta b_{a_{t}}-\hat{R}_{t}^{b_{k}} n_{a}
\end{aligned}\tag{50}
$$
至此，等式(32)推导完毕。

 下面推导等式(33)$\begin{gathered}\delta \dot{\theta}_{t}^{b_{k}}=-\left[w_{t}-\hat{b}_{w_{t}}\right]_{x} \delta \theta_{t}^{b_{k}}-\delta b_{w_{t}}-n_{w}\end{gathered}\tag{33}$： 

为了表述上的方便，定义：
$$
w=w_{t}-b_{w_{t}}-n_{w}\tag{51}
$$

$$
\hat{w}=w_{t}-\hat{b}_{w_{t}}\tag{52}
$$

$w$ 和 $\hat{w}$ 存在以下关系:
$$
w=\hat{w}+\delta w\tag{53}
$$
可以得到:
$$
\begin{aligned}
\delta w &=w-\hat{w} \\
&=-\left(b_{w_{t}}-\hat{b}_{w_{t}}\right)-n_{w} \\
&=-\delta b_{w_{t}}-n_{w}
\end{aligned}\tag{54}
$$
对等式(18) $\gamma_{t}^{b_{k}}=\hat{\gamma}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}}$两边求导：
$$
\dot{\gamma}_{t}^{b_{k}}=\dot{\hat{\gamma}}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}}+\hat{\gamma}_{t}^{b_{k}} \otimes \delta \dot{\gamma}_{t}^{b_{k}}\tag{55}
$$
将等式(28)$\dot{\hat{\gamma}}_{t}^{b_{k}}=\frac{1}{2} \hat{\gamma}_{t}^{b_{k}} \otimes\left(w_{t}-\hat{b}_{w_{t}}\right)$代入等式(55)：
$$
\dot{\gamma}_{t}^{b_{k}}=\frac{1}{2} \hat{\gamma}_{t}^{b_{k}} \otimes \hat{w} \otimes \delta \gamma_{t}^{b_{k}}+\hat{\gamma}_{t}^{b_{k}} \otimes \delta \dot{\gamma}_{t}^{b_{k}}\tag{56}
$$
将等式(18) $\gamma_{t}^{b_{k}}=\hat{\gamma}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}}$代入等式(23)$\dot{\gamma}_{t}^{b_{k}}=\frac{1}{2} \gamma_{t}^{b_{k}} \otimes\left(w_{t}-b_{w_{t}}-n_{w}\right)$：
$$
\dot{\gamma}_{t}^{b_{k}}=\frac{1}{2} \hat{\gamma}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}} \otimes w\tag{57}
$$
联立等式(56)(57)得：
$$
\frac{1}{2} \hat{\gamma}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}} \otimes w=\frac{1}{2} \hat{\gamma}_{t}^{b_{k}} \otimes \hat{w} \otimes \delta \gamma_{t}^{b_{k}}+\hat{\gamma}_{t}^{b_{k}} \otimes \delta \dot{\gamma}_{t}^{b_{k}}\tag{58}
$$
化简等式(58)得：
$$
\delta \dot{\gamma}_{t}^{b_{k}}=\frac{1}{2} \delta \gamma_{t}^{b_{k}} \otimes w-\frac{1}{2} \hat{w} \otimes \delta \gamma_{t}^{b_{k}}\tag{59}
$$
由等式(18)$\begin{aligned}
\gamma_{t}^{b_{k}} &=\hat{\gamma}_{t}^{b_{k}} \otimes \delta \gamma_{t}^{b_{k}} \\
&=\hat{\gamma}_{t}^{b_{k}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{t}^{b_{k}}
\end{array}\right]
\end{aligned}$可知：
$$
\delta \gamma_{t}^{b_{k}}=\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{t}^{b_{k}}
\end{array}\right]\tag{60}
$$
等式(60)实际上是对旋转误差的重新参数化, 旋转误差本质上只有 3 个自由度, 但 是以四元数形式表示的误差 $\delta \gamma_{t}^{b_{k}}$ 有 4 个自由度, 所以将其重新参数化为 $\delta \theta_{t}^{b_{k}}$, 它们之间 的转换关系由等式(60)给出。

实际上, 等式(60)中的等号应该是近似相等, 等式右边仅 保留了 $\delta \theta_{t}^{b_{t}}$ 的一阶项, 高阶项被忽略, 但是由于旋转误差是小量, 所以仅仅保留一阶项 可以使该等式近似成立。

对等式(60)等号两边同时求导:
$$
\delta \dot{\gamma}_{t}^{b_{k}}=\left[\begin{array}{c}
0 \\
\frac{1}{2} \delta \dot{\theta}_{t}^{b_{k}}
\end{array}\right]\tag{61}
$$
等式(61)可改写为：
$$
\left[\begin{array}{c}
0 \\
\delta \dot{\theta}_{t}^{b_{k}}
\end{array}\right]=2 \delta \dot{\gamma}_{t}^{b_{k}}\tag{62}
$$
将等式(59) $\delta \dot{\gamma}_{t}^{b_{k}}=\frac{1}{2} \delta \gamma_{t}^{b_{k}} \otimes w-\frac{1}{2} \hat{w} \otimes \delta \gamma_{t}^{b_{k}}$代入等式(62)得：
$$
\begin{aligned}
{\left[\begin{array}{c}
0 \\
\delta \dot{\theta}_{t}^{b_{k}}
\end{array}\right] } &=2 \delta \dot{\gamma}_{t}^{b_{k}} \\
&=\delta \gamma_{t}^{b_{k}} \otimes w-\hat{w} \otimes \delta \gamma_{t}^{b_{k}} \\
&=\left[\begin{array}{cc}
0 & -w^{T} \\
w & -[w]_{\times}
\end{array}\right] \delta \gamma_{t}^{b_{k}}-\left[\begin{array}{cc}
0 & -\hat{w}^{T} \\
\hat{w} & {[\hat{w}]_{\times}}
\end{array}\right] \delta \gamma_{t}^{b_{k}} \\
&=\left[\begin{array}{cc}
0 & -\left(w^{T}-\hat{w}^{T}\right) \\
w-\hat{w} & -[w+\hat{w}]_{\times}
\end{array}\right] \delta \gamma_{t}^{b_{k}}
\end{aligned}\tag{63}
$$
将等式(60)$\delta \gamma_{t}^{b_{k}}=\left[\begin{array}{c}1 \\ \frac{1}{2} \delta \theta_{t}^{b_{k}}\end{array}\right]$代入等式(63)得：
$$
\left[\begin{array}{c}
0 \\
\delta \dot{\theta}_{t}^{b_{k}}
\end{array}\right]=\left[\begin{array}{cc}
0 & -\left(w^{T}-\hat{w}^{T}\right) \\
w-\hat{w} & -[w+\hat{w}]_{\times}
\end{array}\right]\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{t}^{b_{k}}
\end{array}\right]\tag{64}
$$
再将等式(53) $w=\hat{w}+\delta w$代入等式(64)得：
$$
\left[\begin{array}{c}
0 \\
\delta \dot{\theta}_{t}^{b_{k}}
\end{array}\right]=\left[\begin{array}{cc}
0 & -\delta w \\
\delta w & -[2 \hat{w}+\delta w]_{\times}
\end{array}\right]\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{t}^{b_{k}}
\end{array}\right]\tag{65}
$$
将等式(65)拆分为两个等式：


$$
0=-\delta w \frac{1}{2} \delta \theta_{t}^{b_{k}}\tag{66}
$$

$$
\delta \dot{\theta}_{t}^{b_{k}}=\delta w-[\hat{w}]_{x} \delta \theta_{t}^{b_{k}}-\frac{1}{2}[\delta w]_{x} \delta \theta_{t}^{b_{k}}\tag{67}
$$

等式(66)没什么用处。将等式(67)中的二阶小量略去得：
$$
\delta \dot{\theta}_{t}^{b_{k}}=\delta w-[\hat{w}]_{\times} \delta \theta_{t}^{b_{k}}\tag{68}
$$
将等式(52) $\hat{w}=w_{t}-\hat{b}_{w_{t}}$(54) $\begin{aligned} \delta w &=w-\hat{w} \\ &=-\left(b_{w_{t}}-\hat{b}_{w_{t}}\right)-n_{w} \\ &=-\delta b_{w_{t}}-n_{w} \end{aligned}$代入等式(68)得：
$$
\delta \dot{\theta}_{t}^{b_{k}}=-\left[w_{t}-\hat{b}_{w_{t}}\right]_{\times} \delta \theta_{t}^{b_{k}}-\delta b_{w_{t}}-n_{w}
$$
至此，等式(33)推导完毕。
## 中值积分法下的预积分方程
### 标称状态

$$
\begin{gathered}
\hat{\alpha}_{i+1}^{b_{k}}=\hat{\alpha}_{i}^{b_{k}}+\hat{\beta}_{i}^{b_{k}} \delta t+\frac{1}{2} \frac{R\left(\hat{\gamma}_{i}^{b_{k}}\right)\left(w_{i}-\hat{b}_{w_{i}}\right)+R\left(\hat{\gamma}_{i+1}^{b_{k}}\right)\left(w_{i+1}-\hat{b}_{w_{i+1}}\right)}{2} \delta t^{2} \\
\end{gathered}\tag{70}
$$
$$
\begin{gathered}
\hat{\beta}_{i+1}^{b_{k}}=\hat{\beta}_{i}^{b_{k}}+\frac{R\left(\hat{\gamma}_{i}^{b_{k}}\right)\left(w_{i}-\hat{b}_{w_{i}}\right)+R\left(\hat{\gamma}_{i+1}^{b_{k}}\right)\left(w_{i+1}-\hat{b}_{w_{i+1}}\right)}{2} \delta t \\
\end{gathered}\tag{71}
$$
$$
\begin{gathered}
\hat{\gamma}_{i+1}^{b_{k}}=\hat{\gamma}_{i}^{k_{k}} \otimes\left[\frac{1}{2} \frac{\left(w_{i}-\hat{b}_{w_{i}}\right)+\left(w_{i+1}-\hat{b}_{w_{i+1}}\right)}{2} \delta t\right] \\
\end{gathered}\tag{72}
$$
$$
\begin{gathered}
\hat{b}_{a_{i+1}}=\hat{b}_{a_{i}} \\
\end{gathered}\tag{73}
$$
$$
\begin{gathered}
\hat{b}_{w_{i+1}}=\hat{b}_{w_{i}}
\end{gathered}\tag{74}
$$
### 误差状态

中值积分方式下：

$$
\delta \theta_{i+1}^{b_{k}}=\delta \theta_{i}^{b_{k}}+\delta \dot{\theta}_{i+\frac{1}{2}}^{b_{k}} \delta t\tag{75}
$$
由等式(33) $\delta \dot{\theta}_{t}^{b_{k}}=-\left[w_{t}-\hat{b}_{w_{t}}\right]_{\times} \delta \theta_{t}^{b_{k}}-\delta b_{w_{t}}-n_{w}$可得：
$$
\delta \dot{\theta}_{i+\frac{1}{2}}^{b_{k}}=-\left[\frac{\left(w_{i}-\hat{b}_{w_{i}}\right)+\left(w_{i+1}-\hat{b}_{w_{i+1}}\right)}{2}\right]_{\times} \delta \theta_{i}^{b_{k}}-\frac{\delta b_{w_{i}}+\delta b_{w_{i+1}}}{2}-\frac{n_{w_{0}}+n_{w_{1}}}{2}\tag{76}
$$
因为：
$$
\begin{aligned}
\hat{b}_{w_{i+1}} &=\hat{b}_{w_{i}} \\
\end{aligned}\tag{77}
$$
$$
\begin{aligned}
\delta b_{w_{i+1}} &=\delta b_{w_{i}}
\end{aligned}\tag{78}
$$

将等式(77)(78)代入等式(76)得：
$$
\delta \dot{\theta}_{i+\frac{1}{2}}^{b_k}=-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta \theta_i^{b_k}-\delta b_{w_i}-\frac{n_{w_0}+n_{w_1}}{2}\tag{79}
$$
由于 $n_{w_0}$ 和 $n_{w_1}$ 均为高斯噪声, 加也行, 减也行, 但考虑到 **vins-mono** 的代码中使用的是加，我们也改为使用加：
$$
\delta \dot{\theta}_{i+\frac{1}{2}}^{b_k}=-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta \theta_i^{b_k}-\delta b_{w_i}+\frac{n_{w_0}+n_{w_1}}{2}\tag{80}
$$
将等式(80)代入等式(75)$\delta \theta_{i+1}^{b_{k}}=\delta \theta_{i}^{b_{k}}+\delta \dot{\theta}_{i+\frac{1}{2}}^{b_{k}} \delta t\tag{75}$得：
$$
\delta \theta_{i+1}^{b_k}=\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_x \delta t\right) \delta \theta_i^{b_k}-\delta b_{w_i} \delta t+\frac{n_{w_0}+n_{w_1}}{2} \delta t\tag{81}
$$
接下来计算 $\delta \beta_{i+1}^{b_k}$ :
中值积分方式下:
$$
\delta \beta_{i+1}^{b_k}=\delta \beta_i^{b_k}+\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} \delta t\tag{82}
$$
由等式(32)$\begin{gathered}
\delta \dot{\beta}_{t}^{b_{k}}=-\hat{R}_{t}^{b_{k}}\left[a_{t}-\hat{b}_{a_{t}}\right]_{\times} \delta \theta_{t}^{b_{k}}-\hat{R}_{t}^{b_{k}} \delta b_{a_{t}}-\hat{R}_{t}^{b_{k}} n_{a}
\end{gathered}\tag{32}$可得：
$$
\begin{aligned}
\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} &=-\frac{R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_{\times} \delta \theta_i^{b_k}+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_{i+1}}\right]_{\times} \delta \theta_{i+1}^{b_k}}{2}-\frac{R\left(\hat{\gamma}_i^{b_k}\right) \delta b_{a_i}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta b_{a_{i+1}}}{2} \\
&-\frac{R\left(\hat{\gamma}_i^{b_k}\right) n_{a_0}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) n_{a_1}}{2}
\end{aligned}\tag{83}
$$
由于 $n_{a_0}$ 和 $n_{a_1}$ 均为高斯噪声, 加也行, 减也行, 但考虑到 vins-mono 的代码中使用 的是加, 我们也改为使用加:
$$
\begin{aligned}
\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} &=-\frac{R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_x \delta \theta_i^{b_k}+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_{i+1}}\right]_x \delta \theta_{i+1}^{b_k}}{2}-\frac{R\left(\hat{\gamma}_i^{b_k}\right) \delta b_{a_i}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta b_{a_{i+1}}}{2} \\
&+\frac{R\left(\hat{\gamma}_i^{b_k}\right) n_{a_0}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) n_{a_1}}{2}
\end{aligned}\tag{84}
$$


由等式（29）$\begin{gathered}\dot{\hat{b}}_{a_{t}}=0 \end{gathered}\tag{29}$可知我们认为两帧IMU数据之间的bias是不会发生变化的，所以得到：
$$
\hat{b}_{a_{i+1}}=\hat{b}_{a_i}\tag{85}
$$
由等式（34）$\begin{gathered}\delta\dot{b}_{a_{t}}=n_{b_{a}}\end{gathered}\tag{34}$
$$
\delta b_{a_{i+1}}=\delta b_{a_i}\tag{86}
$$
将等式(85)(86)代入等式(83)：
$$
\begin{aligned}
\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} &=-\frac{R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_{\times} \delta \theta_i^{b_k}+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta \theta_{i+1}^{b_k}}{2}-\frac{R\left(\hat{\gamma}_i^{b_k}\right) \delta b_{a_i}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta b_{a_i}}{2} \\
&+\frac{R\left(\hat{\gamma}_i^{b_k}\right) n_{a_0}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) n_{a_1}}{2}
\end{aligned}\tag{87}
$$
将等式(81)$\delta \theta_{i+1}^{b_k}=\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_x \delta t\right) \delta \theta_i^{b_k}-\delta b_{w_i} \delta t+\frac{n_{w_0}+n_{w_1}}{2} \delta t\tag{81}$代入等式(87)：
$$
\begin{aligned}
\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} &=-\frac{1}{2} R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_{\times} \delta \theta_i^{b_k} \\
&-\frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_{i+1}}\right]_{\times}\left(\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta t\right) \delta \theta_i^{b_k}-\delta b_{w_i} \delta t+\frac{n_{w_0}+n_{w_1}}{2} \delta t\right) \\
&-\frac{R\left(\hat{\gamma}_i^{b_k}\right) \delta b_{a_i}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta b_{a_i}}{2} \\
&+\frac{R\left(\hat{\gamma}_i^{b_k}\right) n_{a_0}+R\left(\hat{\gamma}_{i+1}^{b_k}\right) n_{a_1}}{2}
\end{aligned}\tag{88}
$$
将等式(88)代入等式(82)$\delta \beta_{i+1}^{b_k}=\delta \beta_i^{b_k}+\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} \delta t\tag{82}$得：
$$
\begin{aligned}
\delta \beta_{i+1}^{b_k}=& \delta \beta_i^{b_k} \\
&-\frac{1}{2}\left(R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_x \delta t+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times}\left(\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_x \delta t\right) \delta t\right) \delta \theta_i^{b_k}\right.\\
&-\frac{1}{2}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t \delta b_{a_i} \\
&+\frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^2 \delta b_{w_i} \\
&+\frac{1}{2} R\left(\hat{\gamma}_i^{b_k}\right) \delta t n_{a_0} \\
&+\frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t n_{a_1} \\
&-\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^2 n_{w_0} \\
&-\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^2 n_{w_1}
\end{aligned}\tag{89}
$$
请仔细看一下等式（88）是怎么化简到等式（89）

接下来计算 $\delta \alpha_{i+1}^{b_k}$ :
中值积分方式下:
$$
\delta \alpha_{i+1}^{b_k}=\delta \alpha_i^{b_k}+\delta \dot{\alpha}_{i+\frac{1}{2}}^{b_k} \delta t\tag{90}
$$
由等式(31)得:
$$
\begin{aligned}
\delta \dot{\alpha}_{i+\frac{1}{2}}^{b_k} &=\delta \beta_{i+\frac{1}{2}}^{b_k} \\
&=\delta \beta_i^{b_k}+\delta \dot{\beta}_{i+\frac{1}{2}}^{b_k} \frac{1}{2} \delta t
\end{aligned}\tag{91}
$$
将等式(88)代入等式(91)得：
$$
\begin{aligned}
\delta \dot{\alpha}_{i+\frac{1}{2}}^{b_k}=& \delta \beta_i^{b_k} \\
&-\frac{1}{4}\left(R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_x \delta t+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta t\right) \delta t\right) \delta \theta_i^{b_k} \\
&-\frac{1}{4}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t \delta b_{a_i} \\
&+\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times} \delta t^2 \delta b_{w_i} \\
&+\frac{1}{4} R\left(\hat{\gamma}_i^{b_k}\right) \delta t n_{a_0} \\
&+\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t n_{a_1} \\
&-\frac{1}{8} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^2 n_{w_0} \\
&-\frac{1}{8} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^2 n_{w_i}
\end{aligned}\tag{92}
$$
将等式(92)代入等式(90)$\delta \alpha_{i+1}^{b_k}=\delta \alpha_i^{b_k}+\delta \dot{\alpha}_{i+\frac{1}{2}}^{b_k} \delta t\tag{90}$得：
$$
\begin{aligned}
\delta \alpha_{i+1}^{b_k}=& \delta \alpha_i^{b_k^k} \\
&+\delta \beta_i^{b_k} \delta t \\
&-\frac{1}{4}\left(R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_{\times} \delta t^2+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times}\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta t\right) \delta t^2\right) \delta \theta_i^{b_k} \\
&-\frac{1}{4}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t^2 \delta b_{a_i} \\
&+\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times} \delta t^3 \delta b_{w_i} \\
&+\frac{1}{4} R\left(\hat{\gamma}_i^{b_k}\right) \delta t^2 n_{a_0} \\
&+\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t^2 n_{a_1} \\
&-\frac{1}{8} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^3 n_{w_0} \\
&-\frac{1}{8} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^3 n_{w_1}
\end{aligned}\tag{93}
$$

$$
\left[\begin{array}{c}
\delta \alpha_{i+1}^{b_k} \\
\delta \theta_{i+1}^{b_k} \\
\delta \beta_{i+1}^{b_k} \\
\delta b_{a_{i+1}} \\
\delta b_{w_{i+1}}
\end{array}\right]
=\left[\begin{array}{ccccc}
I & f_{01} & \delta t I & -\frac{1}{4}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t^2 & f_{04} \\
0 & I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta t & 0 & 0 & -\delta t I \\
0 & f_{21} & I & -\frac{1}{2}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t & f_{24} \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{array}\right]
\left[\begin{array}{c}
\delta \alpha_i^{b_k} \\
\delta \theta_i^{b_k} \\
\delta \beta_i^{b_k} \\
\delta b_{a_i} \\
\delta b_{w_i}
\end{array}\right]\\
+\left[\begin{array}{cccccc}
\frac{1}{4} R\left(\hat{\gamma}_i^{b_k}\right) \delta t^2 & v_{01} & \frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t^2 & v_{03} & 0 & 0 \\
0 & \frac{1}{2} \delta t I & 0 & \frac{1}{2} \delta t I & 0 & 0 \\
\frac{1}{2} R\left(\hat{\gamma}_i^{b_k}\right) \delta t & v_{21} & \frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t & v_{23} & 0 & 0 \\
0 & 0 & 0 & 0 & \delta t I & 0 \\
0 & 0 & 0 & 0 & 0 & \delta t I
\end{array}\right]\left[\begin{array}{l}
n_{a_0} \\
n_{w_0} \\
n_{a_1} \\
n_{w_{\mathrm{1}}} \\
n_{b_a} \\
n_{b_w}
\end{array}\right]\tag{94}
$$


$$
f_{01}=-\frac{1}{4} R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_{\times} \delta t^2-\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times}\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta t\right) \delta t^2\tag{95}
$$

$$
f_{04}=-\frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t^2(-\delta t)\tag{}\tag{96}
$$

$$
f_{21}=-\frac{1}{2} R\left(\hat{\gamma}_i^{b_k}\right)\left[a_i-\hat{b}_{a_i}\right]_x \delta t-\frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x\left(I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_x \delta t\right) \delta t\tag{97}
$$

$$
f_{24}=-\frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times} \delta t(-\delta t)\tag{98}
$$

$$
v_{01}=\frac{1}{4}\left(-R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times} \delta t^2\right) \frac{1}{2} \delta t\tag{99}
$$

$$
v_{03}=\frac{1}{4}\left(-R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times} \delta t^2\right) \frac{1}{2} \delta t=v_{01}\tag{100}
$$

$$
v_{21}=\frac{1}{2}\left(-R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_x \delta t\right) \frac{1}{2} \delta t\tag{101}
$$

$$
v_{23}=\frac{1}{2}\left(-R\left(\hat{\gamma}_{i+1}^{b_k}\right)\left[a_{i+1}-\hat{b}_{a_i}\right]_{\times} \delta t\right) \frac{1}{2} \delta t=v_{21}\tag{102}
$$

将等式(94)简写为：
$$
\delta z_{i+1}=F \delta z_i+V N\tag{103}
$$
其中：
$$
\delta z_i=\left[\begin{array}{c}
\delta \alpha_i^{b_k} \\
\delta \theta_i^{b_k} \\
\delta \beta_i^{b_k} \\
\delta b_{a_i} \\
\delta b_{w_i}
\end{array}\right]\tag{104}
$$

$$
\delta z_{i+1}=\left[\begin{array}{c}
\delta \alpha_{i+1}^{b_k} \\
\delta \theta_{i+1}^{b_k} \\
\delta \beta_{i+1}^{b_k} \\
\delta b_{a_{i+1}} \\
\delta b_{w_{i+1}}
\end{array}\right]\tag{105}
$$

$$
F=\left[\begin{array}{ccccc}
I & f_{01} & \delta t I & -\frac{1}{4}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t^2 & f_{04} \\
0 & I-\left[\frac{w_i+w_{i+1}}{2}-\hat{b}_{w_i}\right]_{\times} \delta t & 0 & 0 & -\delta t I \\
0 & f_{21} & I & -\frac{1}{2}\left(R\left(\hat{\gamma}_i^{b_k}\right)+R\left(\hat{\gamma}_{i+1}^{b_k}\right)\right) \delta t & f_{24} \\
0 & 0 & 0 & I & 0 \\
0 & 0 & 0 & 0 & I
\end{array}\right]\tag{106}
$$

$$
V=\left[\begin{array}{cccccc}
\frac{1}{4} R\left(\hat{\gamma}_i^{b_k}\right) \delta t^2 & v_{01} & \frac{1}{4} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t^2 & v_{03} & 0 & 0 \\
0 & \frac{1}{2} \delta t I & 0 & \frac{1}{2} \delta t I & 0 & 0 \\
\frac{1}{2} R\left(\hat{\gamma}_i^{b_k}\right) \delta t & v_{21} & \frac{1}{2} R\left(\hat{\gamma}_{i+1}^{b_k}\right) \delta t & v_{23} & 0 & 0 \\
0 & 0 & 0 & 0 & \delta t I & 0 \\
0 & 0 & 0 & 0 & 0 & \delta t I
\end{array}\right]\tag{107}
$$

$$
N=\left[\begin{array}{l}
n_{a_0} \\
n_{w_0} \\
n_{a_1} \\
n_{w_1} \\
n_{b_a} \\
n_{b_w}
\end{array}\right]\tag{108}
$$

由等式(103)可得雅可比矩阵和协方差矩阵的递推方程：
$$
J_{i+1}=F J_i\tag{109}
$$

$$
P_{i+1}=F P_i F^T+V Q V^T\tag{110}
$$

其中 Q 为噪声信号 N 的协方差矩阵，由于假设 N 的各个分量相互独立，所以 Q 为 对角形矩阵：
$$
Q=\left[\begin{array}{cccccc}
\sigma_{n_a}^2 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma_{n_w}^2 & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma_{n_a}^2 & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma_{n_w}^2 & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma_{n_{b_a}}^2 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma_{n_{b_w}}^2
\end{array}\right]\tag{111}
$$
令 $b_k 、 b_{k+1}$ 表示两个相邻的关键帧, 两个关键帧之间的时间间隔内有一些列的 IMU
测量, $i$ 对应其测量的离散时间点。初始时刻（即第关键帧 $b_k$ 对应的时刻）系统状态的
雅可比矩阵和协方差矩阵为:
$$
J_{b_k}=I\tag{112}
$$

$$
P_{b_k}=0\tag{113}
$$

根据初始值(112)(113)和递推方程(109)(110)可得关键帧 $b_{k+1}$ 对应时刻, 系统状态的雅 可比矩阵 $J_{b_{k+1}}$ 和协方差矩阵 $P_{b_{k+1}}$ 。

$\alpha_{b_{k+1}}^{b_k} 、 \beta_{b_{k+1}}^{b_k}$ 和 $\gamma_{b_{k+1}}^{b_k}$ 相对于关键帧 $b_k$ 时刻加速度计偏置误差 $\delta b_{a_k}$ 和陀螺仪偏置误差 $\delta b_{w_k}$ 的一阶近似可以表示为:
$$
\alpha_{b_{k+1}}^{b_k}=\hat{\alpha}_{b_{k+1}}^{b_k}+J_{\delta b_{a_k}}^{\delta \alpha_{k_k}^{b_k}} \delta b_{a_k}+J_{\delta b_{w_k}}^{\delta \alpha_{k+1}} \delta b_{w_k}\tag{114}
$$

$$
\beta_{b_{k+1}}^{b_k}=\hat{\beta}_{b_{k+1}}^{b_k}+J_{\delta b_{a_k}}^{\delta \beta_{k+1}^{b_k}} \delta b_{a_k}+J_{\delta b_{w_k}}^{\delta \beta_{k+1}^{b_k}} \delta b_{w_k}\tag{115}
$$

$$
\gamma_{b_{k+1}}^{b_k}=\hat{\gamma}_{b_{k+1}}^{b_k} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} J_{\delta b_{w_k}}^{\delta \theta_{k+1}} \delta b_{w_k}
\end{array}\right]\tag{116}
$$

$J_{\delta b_{a_k}}^{\delta b_{b_{k+1}}^{b_k}} 、 J_{\delta b_{w_k}}^{\delta \alpha_{b_{k+1}}^{b_k}} 、 J_{\delta b_{a_k}}^{\delta \beta_{b_{k+1}}^{b_k}} 、 J_{\delta b_{w_k}}^{\delta \beta_{b_{k+1}}^{b_k}} 、 J_{\delta b_{w_k}}^{\delta \theta_{b_{k+1}}^{b_k}}$ 的含义为其对应符号中, 上标对下标的雅可比 矩阵, 它们都是 $J_{b_{k+1}}$ 的子块矩阵, 可以从 $J_{b_{k+1}}$ 对应位置中获取。
当对加速度计偏置和陀螺仪的偏置发生（微小）改变时, 就可以根据等式(114)(115) (116)对预积分项进行修正, 避免了重复积分。
至此, 可以写出预积分形式下 IMU 的测量模型:
$$
\left[\begin{array}{c}
\hat{\alpha}_{b_{k+1}}^{b_k} \\
\hat{\gamma}_{b_{k+1}}^{b_k} \\
\hat{\beta}_{b_{k+1}}^{b_k} \\
0 \\
0
\end{array}\right]=\left[\begin{array}{c}
R_w^{b_k}\left(p_{b_{k+1}}^w-p_{b_k}^w-v_{b_k}^w \Delta t_k+\frac{1}{2} g^w \Delta t_k^2\right) \\
\left(q_{b_k}^w\right)^{-1} \otimes q_{b_{k+1}}^w \\
R_w^{b_k}\left(v_{b_{k+1}}^w-v_{b_k}^w+g^w \Delta t_k\right) \\
b_{a b_{k+1}}-b_{a b_k} \\
b_{w b_{k+1}}-b_{w b_k}
\end{array}\right]\tag{117}
$$