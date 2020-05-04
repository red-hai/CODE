中档：
Q:开摩擦轮
E:关摩擦轮

停止和就绪状态转换：
上拨一次拨弹进入就绪状态，再上拨一次进入停止状态
处于中档时可以用Q控制进入就绪态，E进入停止状态

由就绪态进入射击状态：
拨杆拨一下下档，或按一次鼠标左右键
当拨杆一直处于下档或鼠标长按时保持连发

由射击状态转射击完成状态：
微动开关松开时进入射击完成状态

射击完成后，如果一段时间内没子弹则进入就绪态，如果还有微动开关还处于按下状态，则重新进入射击状态


云台的陀螺仪角度范围为-180度~180度
底盘的陀螺仪角度范围为-180度~180度
云台和底盘的相对角度的范围为0~360度
重点：小陀螺的旋转要的Vx和Vy二维矩阵计算的角度范围要为0~360度
			  sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;