# VCX Final Project

## 选题说明

选题为参考选题所给出的D-3，即使用[Fast Simulation of Mass-Spring Systems](http://tiantianliu.cn/papers/liu13fast/liu13fast.html)提出的方法加速Lab-4中弹簧质点系统的解算仿真。

## 实现思路

#### 算法说明

设系统包含$n$个节点，将这$n$个节点的位置用$\bold{q} \in \mathbb{R}^{3n}$表示。则隐式欧拉法的方程为
$$
\begin{aligned}
\bold{v}_{t+\Delta t} &= \bold{v}_{t} + \Delta t \bold{M}^{-1}\bold{f}(\bold{q}_{t+\Delta t}) \\
\bold{q}_{t + \Delta t} &= \bold{q}_{t} + \Delta t \bold{v}_{t+\Delta t}
\end{aligned}
$$其中$\bold{M} = \mathrm{diag}\{m_1,\cdots,m_n\} \otimes \bold{I}_{3\times 3}$。记$\bold{x} = \bold{q}_{t+\Delta t}$，$\bold{y} = \bold{q}_{t} + \Delta t \bold{v}_{t}$，整理得
$$
\bold{M(x-y)} = \Delta t^2 \bold{f(x)}
$$设系统能量为$E = E_{\mathrm{ext}}+E_{\mathrm{spring}}$，则$\bold{f(x)} = - \nabla E(\bold{x})$，令
$$
g(\bold{x}) = \cfrac{1}{2}\bold{(x-y)^{T}M(x-y)} + \Delta t^2 E(\bold{x})
$$则$\bold{M(x-y)} = \Delta t^2 \bold{f(x)}$等价于$\nabla_{\bold{x}}g(\bold{x}) = \bold{0}$，也即所求的$\bold{x}$是$g(\bold{x})$的极值点。将$g(\bold{x})$化简
$$
\begin{aligned}
g(\bold{x}) &= 
\cfrac{1}{2}\bold{(x-y)^{T}M(x-y)} + \Delta t^2(E_{\mathrm{ext}}+E_{\mathrm{spring}}) \\
&= \cfrac{1}{2}\bold{(x-y)^{T}M(x-y)} - \Delta t^2 \bold{f}_{\mathrm{ext}}^{\bold{T}}\bold{x} + \Delta t^2\sum_{i=1}^{s}\cfrac{k_i}{2}(||\bold{p}_{i,1} - \bold{p}_{i,2}|| -r_{i})^2
\end{aligned}
$$其中$s$为弹簧数目，$\bold{p}_{i,1}$和$\bold{p}_{i,2}$为第$i$个弹簧两端的质点的位置，$r_i$为弹簧原长度。而对于$r \geq 0$，有$(||\bold{p}_1-\bold{p}_2|| - r)^2 = \min\limits_{||\bold{d}||=r}||(\bold{p}_1-\bold{p}_2)-\bold{d}||^2$，在$\bold{d} = \cfrac{\bold{p}_1-\bold{p}_2}{||\bold{p}_1-\bold{p}_2||}r$时取到极值，故
$$
\begin{aligned}
g(\bold{x}) &= 
\cfrac{1}{2}\bold{(x-y)^{T}M(x-y)} - \Delta t^2 \bold{f}_{\mathrm{ext}}^{\bold{T}}\bold{x} + \Delta t^2\sum_{i=1}^{s}\cfrac{k_i}{2}\min_{\bold{d}_i}||\bold{p}_{i,1} - \bold{p}_{i,2}-\bold{d}_i||^2\\
&= \cfrac{1}{2}\bold{x^{T}}(\bold{M}+\Delta t^2\bold{L})\bold{x} - \Delta t^2\bold{x^{T}Jd}-\bold{x^{T}}(\bold{My} + \Delta t^2\bold{f}_{\mathrm{ext}}) + \mathrm{Constant}
\end{aligned}
$$其中
$$
\begin{aligned}
&\bold{L} = \left(\sum_{i=1}^{s}k_i\bold{A}_i\bold{A}_i^{\bold{T}}\right)\otimes \bold{I}_{3\times 3} \in \mathbb{R}^{3n\times 3n}, &&\bold{J} = \left(\sum_{i=1}^{s}k_i\bold{A}_i\bold{S}_i^{\bold{T}}\right)\otimes\bold{I}_{3\times 3} \in \mathbb{R}^{3n \times 3s} \\
&\bold{A}_{i,j} = {\begin{cases}
    1, & j=i_{1} \\ -1, & j=i_{2} \\ 0, &\mathrm{otherwise}
\end{cases}}
, \quad \bold{A}_i \in \mathbb{R}^{n}, &&\bold{S}_{i,j} = \delta_{i,j},\quad  \bold{S}_i \in \mathbb{R}^{s} \\
&\bold{d} = (\bold{d}_1, \cdots, \bold{d}_s) \in \mathbb{R}^{3s}, && \bold{d}_i = \cfrac{\bold{p}_{i,1} - \bold{p}_{i,2}}{||\bold{p}_{i,1} - \bold{p}_{i,2}||}r_i
\end{aligned}
$$于是
$$
\begin{aligned}
    \nabla_{\bold{x}}g(\bold{x}) = \bold{0} \iff (\bold{M}+ \Delta t^2\bold{L})\bold{x} = \Delta t^2 \bold{Jd} + \bold{My} + \Delta t^2\bold{f}_{\mathrm{ext}} \iff \bold{Qx = b}
\end{aligned}
$$只需解出$\bold{x}$即得各质点新的位置。

#### 实现方法

在上面的算法中，将求解质点新的位置转化为求解线性系统$\bold{Qx = b}$，为此需要求出系数$\bold{Q}$和$\bold{b}$，也即需要求出$\bold{M}$、$\bold{L}$、$\bold{J}$、$\bold{d}$、$\bold{y}$和$\bold{f}_{\mathrm{ext}}$。

其中$\bold{M}$、$\bold{L}$、$\bold{J}$仅与弹簧质点系统本身的性质（质点质量、弹簧的连接方式）有关，故在构建弹簧质点系统时可以直接求出并存储，而无需在每一步渲染时重新求解。

$\bold{y}$和$\bold{f}_{\mathrm{ext}}$的求解，只需在渲染时的每一步遍历一次所有的质点，$\bold{d}$的求解则只需遍历一次所有弹簧。

解出$\bold{x}$后，再利用$\cfrac{\bold{x}-\bold{x}_{\mathrm{old}}}{\Delta t}$即可计算出新的速度$\bold{v}$。

弹簧质点系统的实现使用Lab-4所提供的数据结构，矩阵的建立与求解主要使用`Eigen`库中的`SparseMatrix`和`VectorXf`。

## 编译环境

程序的整体框架来自Lab-4，用Xmake直接编译即可。

## 实验设置

#### 弹簧质点系统

所有质点质量和弹簧强度均一致，可调节。

#### 力

内力为弹簧弹力。

外力包括：

- 重力，竖直向下，可调节。
- 阻尼，沿运动速度逆方向，采用$\bold{f} = \alpha \bold{v}$。
- 风力，大小、方向均可调节。

#### 线性系统求解

每一次求解的迭代次数可调节。

## 实验结果

#### 基本结果

在`res/Demo.gif`中，展示了算法效果和风力的设置过程。

#### 迭代次数对实际效果的影响

将阻尼系数设为较大的值，求解过程的迭代次数分别取较小和较大的值，结果分别在`res/SmallStep.gif`和`res/LargeStep.gif`中。

**现象** 对比可以发现，二者最终均能达到稳定（静止状态），但使用较小的迭代次数时，布料下落速度较快，会经过平衡点一小段距离后再回落；而使用较大的迭代次数时，布料以较慢的速度下落到平衡点便几乎达到停止。

**分析** 迭代次数较小时，速度受阻尼作用而降低的被计算次数减少，故速度衰减率更慢。
