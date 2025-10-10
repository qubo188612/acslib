#ifndef ACS_H_
#define ACS_H_

#define DLL_EXPORTS extern "C" __declspec(dllexport)
#define DLL_CLASSEXP __declspec(dllexport)
#include "dataStructure.h"

namespace acs
{
	/**
	 * @brief 测试dump接口，实现一个野指针赋值触发崩溃
	 */

	DLL_EXPORTS void ACS_DumpTrigger();

	/**
	 * @brief 项目初始化操作
	 */
	DLL_EXPORTS void ACS_Init();

/*
返回:  true已经连接,false未连接
*/
	DLL_EXPORTS bool ACS_InitIsOpen();    //是否已经初始化


	DLL_EXPORTS void ACS_SetQtObject(void *ptr);    //设置QT信号槽地址

/*
返回:  0         //正常
返回:  1         //失败
*/
	DLL_EXPORTS int ACS_GetDeviceNum(int &deviceNum);	//获取驱动总数

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //失败
*/
	DLL_EXPORTS int ACS_GetDeviceInfo(int deviceId, char* strDeviceId, char* strFriendlyName, char* strDriverVersion);//获取驱动信息

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //tcp连接失败
返回:  3         //tcp连接已经存在
*/
	DLL_EXPORTS int ACS_EthernetOpen(int deviceId, char* ip,int port);    //连接传感器

/*
返回:  true已经连接,false未连接
*/
	DLL_EXPORTS bool ACS_EthernetIsOpen(int deviceId);    //是否已经连接传感器

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
*/
	DLL_EXPORTS int ACS_CommunicationClose(int deviceId); //断开传感器

/*
获取报错信息
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
*/
	DLL_EXPORTS int ACS_GetErrorInfo(int deviceId, char *err);

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
*/
	DLL_EXPORTS int ACS_GetAxisTotalnum(int deviceId,int &totalnum); //获取轴总数

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisEn(int deviceId, int axisId,bool enable);//设置轴使能

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisEn(int deviceId, int axisId, bool &enable);//获取轴使能

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisSpeed(int deviceId, int axisId, double &speed);//获取轴速度

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisSpeed(int deviceId, int axisId, double speed);//设置轴速度

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisDeceleratedSpeed(int deviceId, int axisId, double &speed);//获取减速度

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisDeceleratedSpeed(int deviceId, int axisId, double speed);//设置减速度

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisAcceleratedSpeed(int deviceId, int axisId, double &speed);//获取加速度

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisAcceleratedSpeed(int deviceId, int axisId, double speed);//设置加速度

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisPosition(int deviceId, int axisId, double &position);//获取轴坐标

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool enable);//设置软限位左极限使能

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool &enable);//获取软限位左极限使能

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisSoftwareNegativeLimit(int deviceId, int axisId, double pos);//设置软限位左极限坐标

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisSoftwareNegativeLimit(int deviceId, int axisId, double &pos);//获取软限位左极限坐标

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool enable);//设置软限位右极限使能

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool &enable);//获取软限位右极限使能

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisSoftwarePositiveLimit(int deviceId, int axisId, double pos);//设置软限位右极限坐标

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //获取失败
*/
	DLL_EXPORTS int ACS_GetAxisSoftwarePositiveLimit(int deviceId, int axisId, double &pos);//获取软限位右极限坐标

/*
返回:  0         //开始移动(非阻塞)，如果连续调用时会按新点位移动
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //软限位信息获取异常
返回:  5		 //到达软限位
返回:  6		 //移动失败
*/
	DLL_EXPORTS int ACS_MoveAbsPos(int deviceId, int axisId, double pos);//绝对坐标移动

/*
返回:  0         //开始移动(非阻塞)，如果连续调用时会按新点位移动
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //软限位信息获取异常
返回:  5		 //到达软限位
返回:  6		 //移动失败
*/
	DLL_EXPORTS int ACS_MoveRelPos(int deviceId, int axisId, double pos);//相对坐标移动

/*
返回:           //true轴待机,false轴异常
*/
	DLL_EXPORTS bool ACS_GetAxisIsReady(int deviceId, int axisId);//是否轴空闲

/*
返回:     mode  0:UNKNOW
				1:HOME
				2:PTP
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
*/
	DLL_EXPORTS int ACS_GetAxisMoveMode(int deviceId, int axisId,int &mode);//返回轴工作

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //设置失败
*/
	DLL_EXPORTS int ACS_SetAxisHomeBuffer(int deviceId, int axisId, int homebuffer);//设置轴复位buffer


/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //复位失败
*/
	DLL_EXPORTS int ACS_AxisHome(int deviceId, int axisId);//轴复位

/*
返回:  0         //正常
返回:  1         //连接传感器的deviceId要大于等于0，且小于等于MAX_DEVICEID_NUM
返回:  2         //ACS网络未连接
返回:  3		 //轴axisId号超过轴总数
返回:  4		 //操作失败
*/
	DLL_EXPORTS int ACS_AxisStop(int deviceId, int axisId);//轴停止
}
#endif
