原始代码，1.13.3版本
src/modules/steering_engine为一个模块模板
src/drivers/scd为一个I2C驱动模板
其余项目可在此基础上开分支

项目内容：在src/modules/steering_engine模块中订阅了陀螺仪和加速度计数据，调用lib/mahony静态库，使用无约束最优化问题之Rosenbrock方法
	解算出姿态角，并将话题记录在日志中。
