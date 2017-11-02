#ifndef __TASK_MAP_GENERATING_H
#define __TASK_MAP_GENERATING_H
/*�궨��*/
//�ļ���ȡ
#define READ_FILE_OK	0	//������ȡ�ļ�
#define READ_FILE_NULL	1	//���ļ����ļ��޷���
#define READ_FILE_ERROR	2	//�ļ����ݴ���
//���Զ���
//����1
#define ATTRIBUTE1_START			0	//���
#define ATTRIBUTE1_ENTRANCE			1	//��������
#define ATTRIBUTE1_EXIT				2	//����ڳ���
#define ATTRIBUTE1_COMMON			3	//��ͨ·��
#define ATTRIBUTE1_PARK_ENTRANCE	4	//����ͣ����
#define ATTRIBUTE1_PARK_EXIT		5	//ʻ��ͣ����
#define ATTRIBUTE1_PARK				6	//ͣ��λλ��
#define ATTRIBUTE1_END				7	//�յ�
#define ATTRIBUTE1_INTERPOLATION	8	//��ֵ��

//����2
#define ATTRIBUTE2_UNKNOWN		0	//δ֪
#define ATTRIBUTE2_GO_STRAIGHT	1	//ֱ��
#define ATTRIBUTE2_TURN_RIGHT	2	//��ת
#define ATTRIBUTE2_TURN_LEFT	3	//��ת
#define ATTRIBUTE2_U_TURN		4	//��ͷ
#define ATTRIBUTE2_TRAFFIC_SIGN	5	//��ͨ��־
//����
#define FACTOR_METRICS (6371004*3.14159/180)	//����γ��ת��Ϊ�׵�ϵ��
/*��������*/

/*���Ͷ���*/
//�����
typedef struct _COORD_T
{
	double longitude;	//����
	double latitude;	//ά��
}COORD_t;
//���
typedef struct _NODE
{
	int id;				//·�����
	double longitude;	//����
	double latitude;	//ά��
	double elevation;	//����
	int attribute1;		//����1������: 0����㣬1���������㣬2������ڳ��㣬3����ͨ·�㣬4������ͣ������5��ʻ��ͣ������6��ͣ��λλ�ã�7���յ�ͣ��
	int attribute2;		//����2��ʻ�����ͨ��־�� 0��δ֪��1��ֱ�У�2����ת��3����ת��4����ͷ��5����ͨ��־
	double turnAngle;
	double coursingAngle;
	int    onetwoway;
	int    waySum;
	int    targetWay;
	struct _NODE *next_node;	//ָ��������ӵ�·��
	struct _WAY *p_way;
} NODE_t;
//·��
typedef struct _WAY
{
	int id;	//����
	NODE_t *head_node;
} WAY_t;

#endif
