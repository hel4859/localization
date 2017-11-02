#ifndef __TASK_MAP_GENERATING_H
#define __TASK_MAP_GENERATING_H
/*宏定义*/
//文件读取
#define READ_FILE_OK	0	//正常读取文件
#define READ_FILE_NULL	1	//无文件或文件无法打开
#define READ_FILE_ERROR	2	//文件内容错误
//属性定义
//属性1
#define ATTRIBUTE1_START			0	//起点
#define ATTRIBUTE1_ENTRANCE			1	//交叉口入点
#define ATTRIBUTE1_EXIT				2	//交叉口出口
#define ATTRIBUTE1_COMMON			3	//普通路点
#define ATTRIBUTE1_PARK_ENTRANCE	4	//进入停车区
#define ATTRIBUTE1_PARK_EXIT		5	//驶出停车区
#define ATTRIBUTE1_PARK				6	//停车位位置
#define ATTRIBUTE1_END				7	//终点
#define ATTRIBUTE1_INTERPOLATION	8	//插值点

//属性2
#define ATTRIBUTE2_UNKNOWN		0	//未知
#define ATTRIBUTE2_GO_STRAIGHT	1	//直行
#define ATTRIBUTE2_TURN_RIGHT	2	//右转
#define ATTRIBUTE2_TURN_LEFT	3	//左转
#define ATTRIBUTE2_U_TURN		4	//掉头
#define ATTRIBUTE2_TRAFFIC_SIGN	5	//交通标志
//其他
#define FACTOR_METRICS (6371004*3.14159/180)	//将经纬度转化为米的系数
/*类型声明*/

/*类型定义*/
//坐标点
typedef struct _COORD_T
{
	double longitude;	//经度
	double latitude;	//维度
}COORD_t;
//结点
typedef struct _NODE
{
	int id;				//路点序号
	double longitude;	//经度
	double latitude;	//维度
	double elevation;	//海拔
	int attribute1;		//属性1点类型: 0：起点，1：交叉口入点，2；交叉口出点，3：普通路点，4：进入停车区，5：驶出停车区，6：停车位位置，7：终点停车
	int attribute2;		//属性2行驶方向或交通标志： 0：未知，1：直行，2：右转，3：左转，4：掉头，5：交通标志
	double turnAngle;
	double coursingAngle;
	int    onetwoway;
	int    waySum;
	int    targetWay;
	struct _NODE *next_node;	//指向后面连接的路点
	struct _WAY *p_way;
} NODE_t;
//路径
typedef struct _WAY
{
	int id;	//等于
	NODE_t *head_node;
} WAY_t;

#endif
