�е���
Q:��Ħ����
E:��Ħ����

ֹͣ�;���״̬ת����
�ϲ�һ�β����������״̬�����ϲ�һ�ν���ֹͣ״̬
�����е�ʱ������Q���ƽ������̬��E����ֹͣ״̬

�ɾ���̬�������״̬��
���˲�һ���µ�����һ��������Ҽ�
������һֱ�����µ�����곤��ʱ��������

�����״̬ת������״̬��
΢�������ɿ�ʱ����������״̬

�����ɺ����һ��ʱ����û�ӵ���������̬���������΢�����ػ����ڰ���״̬�������½������״̬


��̨�������ǽǶȷ�ΧΪ-180��~180��
���̵������ǽǶȷ�ΧΪ-180��~180��
��̨�͵��̵���ԽǶȵķ�ΧΪ0~360��
�ص㣺С���ݵ���תҪ��Vx��Vy��ά�������ĽǶȷ�ΧҪΪ0~360��
			  sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;