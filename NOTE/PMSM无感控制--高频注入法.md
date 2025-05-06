# PMSM无感控制--高频注入法

基本思想：将高频的电压或者电流信号叠加到基频信号上，则响应电流中也包含了基频响应与高频响应，通过带通滤波器(BPF)或作差等方式将两响应电流分离，高频电流中包含了转子的速度信息$\omega_e$和位置信息$\theta_e$，在通过一定手段(解耦、锁相环等)将其提取出来，便可以进行后续的控制

高频注入法大致分为两类(一般使用高频方波比骄简单)：

1. 适用于凸极率高的内嵌式PMSM的**旋转高频电压注入法**
2. 适用于凸极低高的表贴式PMSM的**脉振高频电压注入法**

## 高频注入法原理



### PMSM的dq轴电压方程

$$
\left[
\begin{matrix}
U_d\\
U_q
\end{matrix}
\right]

=

\left[
\begin{matrix}
R+pL_d & -\omega_eL_q\\
\omega_eL_d & R+pL_q
\end{matrix}
\right]

\left[
\begin{matrix}
i_d\\i_q
\end{matrix}
\right] + 
\left[
\begin{matrix}
0 \\ \omega_e\Psi_f
\end{matrix}
\right] \ \ 公式（1）
\\其中的p为微分算子\frac{d}{dt}
$$

在零速、低速情况下，转速很低->反电动势很低，进而可以忽略。又因为在dq轴施加高频信号时，电感的阻抗>>大于电阻的阻抗(电感通直流，阻交流)，故可以忽略电阻上的压降。因此在求高频信号响应时，只考虑电感上的压降
$$
忽略电阻阻抗后可以得到\\
\left[
\begin{matrix}
U_{dh}\\U_{qh}
\end{matrix}
\right] = 

\left[
\begin{matrix}
L_d & 0\\
0 & L_q
\end{matrix}
\right]

\left[
\begin{matrix}
\frac{di_{dh}}{dt}\\
\frac{di_{qh}}{dt}
\end{matrix}
\right]   \ \ 公式（2）
\\
将矩阵进行移相后得到\\
\left[
\begin{matrix}
\frac{di_{dh}}{dt}\\
\frac{di_{qh}}{dt}
\end{matrix}
\right] =

\left[
\begin{matrix}
\frac{1}{L_d} & 0 \\
 0 & \frac{1}{L_q}
\end{matrix}
\right]
\left[
\begin{matrix}
U_{dh} \\ U_{qh}
\end{matrix}
\right]   \ \ 公式（3）
$$
根据响应电流提取转子信息时，可以用dq轴或者$\alpha\beta$轴的电压电流信息进行解算。但是从电机的ABC三相变换到dq轴需要转子角度信息$\theta_e$，但是在电机启动时并不知道$\theta_e$的，在迭代过程中可能会出现观测电角度的收敛速度慢、或者不收敛的问题，因此我们还是选择使用$\alpha\beta$轴进行转子的位置和转速信息进行解算

建立双dq空间坐标轴：dq轴为转子的实际位置，$\dot{d}$$\dot{q}$轴为预估的转子位置

![img](https://picx.zhimg.com/v2-25a502eb6ddb9bcb7ca0fe5abaf92bb5_r.jpg)
$$
对上面的dq轴坐标系进行Clarke变换得到\\
\left[
\begin{matrix}
\frac{di_{\alpha h}}{dt}\\
\frac{di_\beta h}{dt}
\end{matrix}
\right] = 
\left[
\begin{matrix}
cos\theta_e & -sin\theta_e\\
sin\theta_e & cos\theta_e
\end{matrix}
\right]
\left[
\begin{matrix}
\frac{1}{L_d} & 0\\
0 & \frac{1}{L_q}
\end{matrix}
\right]
\left[
\begin{matrix}
U_{dh}\\ U_{qh}
\end{matrix}
\right]
$$
由于我们并不明确dq轴的具体位置，只能预估出一个$\dot{d} \dot{q}$轴，向这个轴进行高频信号注入后，上式表达式变为
$$
\left[
\begin{matrix}
\frac{di_{\alpha h}}{dt}\\
\frac{di_\beta h}{dt}
\end{matrix}
\right] = 
\left[
\begin{matrix}
cos\theta_e & -sin\theta_e\\
sin\theta_e & cos\theta_e
\end{matrix}
\right]
\left[
\begin{matrix}
\frac{1}{L_d} & 0\\
0 & \frac{1}{L_q}
\end{matrix}
\right]

\left[
\begin{matrix}
cos{\theta_s} & sin{\theta_s}\\
-sin{\theta_s} & cos{\theta_s}
\end{matrix}
\right]

\left[
\begin{matrix}
U_{\dot{d}h}\\ U_{\dot{q}h}
\end{matrix}
\right]   \ \ 公式（4）
\\
\theta_s是预测dq轴和实际dq轴之间的误差角
$$
接下来便是进行高频信号注入。利用d轴易磁饱和的特性(因为永磁体的存在)，我们向d轴注入高频方波信号，使其等效电感发生变化，增加电机的凸极率，便于后续位置信息的提取

磁链公式为$\Psi = Li$

由磁链公式可知：$I_d = \frac{\Delta\Psi}{\Delta i_d}$，当id变化时，绕组趋近于磁饱和，到时$\Psi_d$变化很小，因此等效电感数值减小。但是由于q轴难以发生磁饱和现象，因此iq在较大范围内变化时，等效电感Lq基本不变

因此高频信号定为
$$
\left[
\begin{matrix}
U_{\dot{d}h}\\ U_{\dot{q}h}
\end{matrix}
\right]
=
\left[
\begin{matrix}
(-1)^kU_{in}\\0
\end{matrix}
\right]   \ \ 公式（5）
$$
其中$U_{in}$约为电机额定电压的10%

将公式（5）带入公式（4）并展开得到
$$
\frac{di_{\alpha h}}{dt} = 
(-1)^kU_{in}*
\left[
\frac{1}{L_d}cos\theta_ecos\theta_s+\frac{1}{L_q}sin\theta_ecos\theta_s
\right]
\\
同理可得\beta轴的公式\frac{di_{\beta h}}{dt} = 
(-1)^kU_{in}*
\left[
\frac{1}{L_d}cos\theta_ecos\theta_s+\frac{1}{L_q}sin\theta_ecos\theta_s
\right]
\\
对\alpha轴的公式进行化简得到\frac{di_{\alpha h}}{dt}=\frac{(-1)^kU_{in}}{L_dL_q}*
\left[
\frac{L_d+L_q}{2}cos\widehat{\theta_e}-\frac{L_d-L_q}{2}cos(\theta_e+\theta_s)
\right]\\
同理可得
\frac{di_{\beta h}}{dt}=\frac{(-1)^kU_{in}}{L_dL_q}*
\left[
\frac{L_d+L_q}{2}cos\widehat{\theta_e}-\frac{L_d-L_q}{2}cos(\theta_e+\theta_s)
\right]
\\
\\
由微分的定义可得
i_{\alpha h}(t)-i_{\alpha h}(t-1)=\frac{(-1)^kU_{in}*Ts}{L_dL_q}*
\left[
\frac{L_d+L_q}{2}cos\widehat{\theta_e}-\frac{L_d-L_q}{2}cos(\theta_e+\theta_s)
\right]
$$
![image-20250430033349747](C:\Users\28076\AppData\Roaming\Typora\typora-user-images\image-20250430033349747.png)

符号函数消除了$U_{dh}$中的(-1)^k的影响，使得后面的k是一个常数项

通过矢量叉乘的方式进行误差信息解耦：
$$
-I_{\alpha h}sin\widehat{\theta}+I_{\beta h}cos\widehat{\theta} =\\ 
\frac{U_{in}T_s(L_d-L_q)}{2L_dL_q}
\left[
sin\widehat{\theta_e}cos(\theta_e+\theta_s) -
cos\widehat{\theta_e}sin(\theta_e+\theta_s)
\right]\\
=
\frac{kU_{in}T_s(L_d-L_q)}{2L_dL_q}sin(\widehat{\theta_e}-\theta_e-\theta_s)
\\
因为\theta_s = \theta_e - \widehat{\theta_e}
\\
所以-I_{\alpha h}sin\widehat{\theta}+I_{\beta h}cos\widehat{\theta} = 
\frac{kU_{in}T_s(L_d-L_q)}{2L_dL_q}sin(-2\theta_s)
\\
由泰勒公式可知，当\theta_s->0时，sin(\theta_S) = \theta_s
\\
因此，令k=\frac{kU_{in}T_s(L_d-L_q)}{2L_dL_q},解耦的结果为f(\theta_s)=k\theta_s
$$

### PLL锁相环提取电角度以及电角速度信息

得到$f(\theta_s)=k\theta_S$后，将其放入锁相环结构进行转子位置信息与转速信息的提取

![image-20250430035438675](C:\Users\28076\AppData\Roaming\Typora\typora-user-images\image-20250430035438675.png)

$\theta_s即\widetilde{\theta_e} = \theta_e-\widehat{\theta}$

由锁相环的结构图可知从$\theta_e到\widehat{\theta_e}$的传递函数为
$$
\Phi(s)=\frac{\widehat{\theta_e}}{\theta_e}=
\frac{k(k_p+\frac{K_i}{s})\frac{1}{s}}{1+k(kp+\frac{k_i}{s})\frac{1}{s}}=
\frac{kk_ps+kk_i}{s^2+kk_ps+kk_i}
$$
这是一个二阶系统，列些与之形式相同的二阶系统传递函数：
$$
\Phi_2(s)=\frac{c(s)}{R(s)}=
\frac{2\xi \omega_n s+\omega_n^2}{s^2+2\xi\omega_ns+\omega_n^2}\\
该二阶系统的无阻尼自然频率\omega_n=\sqrt{kk_i},k_i=\frac{\omega_n^2}{k},
k_p=\frac{2\xi\omega_n}{k}=\frac{2\xi\sqrt{kk_i}}{k}\\
其增益带宽约为无阻尼自然频率，即\omega_c约等于\omega_n=\sqrt{kk_i}，其中k=\frac{kU_{in}T_s(L_d-L_q)}{2L_dL_q}\\
带宽决定了系统的输出跟随输入信号的速度，同时为了避免超调量的产生，一般设计阻尼比\xi >=1
$$


### 高频电流的提取 

使用无滤波器的电流提取方案：**采样加减法分离**

响应电流、载波和注入信号之间的关系：(将注入信号与载波信号频率保持相等，即每次采样处理时将注入信号改变极性)。由于注入信号远大于基波频率，故可以认为：

1. 相邻采样点处基频电流分量保持恒定
2. 相邻采样点处高频电流分量幅值相等，极性相反 

![701101f93960d4cadc4cf7b7027490d](C:\Users\28076\Documents\WeChat Files\wxid_ok7ts8hs26xo22\FileStorage\Temp\701101f93960d4cadc4cf7b7027490d.jpg)
$$
\begin{cases}
i_{dq}(k-2) = i_{dqh}(k-2)+i_{dqf}(k-2)\\
i_{dq}(k-1) = i_{dqh}(k-1)+i_{dqf}(k-1)\\
i_{dq}(k) = i_{dqh}(k)+i_{dqf}(k)\\
i_{dqh}(k) = -i_{dqh}(k-1)=i_{dqh}(k-2)\\
i_{dqf}(k) = i_{dqf}(k-1)=i_{dqf}(k-2)
\end{cases}
$$
由上面的式子可以得到
$$
\begin{cases}
i_{dqh}(k)=(i{dq}(k)-2i{dq(k-1)}+i_{dq}(k-2))/4\\
i_{dqf}(k)=(i{dq}(k)+2i{dq(k-1)}+i_{dq}(k-2))/4
\end{cases}
$$


 ### 转子极性辨识

借助d轴容易磁饱和的特性来进行极性辨识，分别向$\dot{d}$轴注入正向的直流偏置电压和方向的直流偏置电压，则等效电感$L_d$会因磁饱和和退磁效应而改变：
在直流偏置所导致的$L_d$变化的基础上，我们将采样电流的高频分量进行提取。

#### 过程

- 施加直流偏置的手段：将d轴的电流环的预期值分别设置为+-I_offeset。
- 在直流偏置的基础上，采样电流，并进行clarke变换，将$\alpha \beta$轴的电流分离高频变量参与位置解耦，得到$\widehat{\theta}$，并对$I_\alpha 和I_\beta$进行park变换，再分离出d轴电流的高频分量，进行采样比较
- 假如：|$i_{dhA}$|>|$i_{dhB}$|，则$\widehat{\theta_e}$收敛在$\theta_e$附近，位置信息正确解耦
- 假如：|$i_{dhA}$|<|$i_{dhB}$|，则$\widehat{\theta_e}$收敛在则$\theta_e + \pi$附近，对锁相环输出+$\pi$进行校正

## 巴特沃斯滤波器(DSP永远的神！)

优点：相比于传统的一阶互补低通滤波，巴特沃斯滤波器在阻带截止频率之前信号保留较好，且在阻带频率内的衰减更快，滤波性能更佳。且幅频特性曲线平滑，滤波阶数越高，效果越好 

- 归一化的巴特沃斯滤波器的传递函数为$H(s)=\frac{d_0}{a_0+a_1s+a_2s^2+...+a_Ns^N}$，其中d0=a0=aN=1
- 对于不同阶数的LPF，归一化后的分母多项式可以通过查表获得，可以看DSP课本
- 最后要把s域转换为离散的z域
- 一般可以采用双线性变换法进行设计数字低通滤波器
- 滤波器系数可以借助MATLAB进行辅助计算

## 整体运行的框图

![image-20250430013515190](C:\Users\28076\AppData\Roaming\Typora\typora-user-images\image-20250430013515190.png)

