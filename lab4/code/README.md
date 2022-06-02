# Hw4完成了所有的基础题和提高题
### 实验结果
实验结果放在images文件夹下，其中包含了：
（i）bezier_4points.png 同时使用自带的算法和自己实现的算法绘制出4个控制点的bezier曲线，结果是预期的黄色的曲线。
（ii）bezier_8points.png 使用自己实现的算法绘制出8个控制点的bezier曲线，测试算法绘制任意控制点数目曲线的能力。
（iii）bezier_4points_antialiasing.png 实现bezier曲线的反走样。

### 实验思路
（i）用De Casteljau算法来绘制bezier曲线，具体算法实验PDF中写的很清楚，思路就是递归，用下一层的控制点序列和t来得到上一层的控制点序列，每层控制点数目递减一，直到控制点只剩下一个即可得到当前t对应的bezier曲线上的点。
（ii）bezier曲线的反走样，我单独写了一个anti_aliasing函数，思路是对于当前得到的点，计算它与周围四个像素中心点的距离，通过距离的比例来计算其对每个像素点的贡献，同一个像素如果被多次赋值，取最大值。贡献比例关系的计算，以距离最小的作为基准值，对某个像素的贡献比例就是用这个基准值除以点到该像素中心的距离。