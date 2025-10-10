#ifndef ACS_H_
#define ACS_H_

#define DLL_EXPORTS extern "C" __declspec(dllexport)
#define DLL_CLASSEXP __declspec(dllexport)
#include "dataStructure.h"

namespace acs
{
	/**
	 * @brief ����dump�ӿڣ�ʵ��һ��Ұָ�븳ֵ��������
	 */

	DLL_EXPORTS void ACS_DumpTrigger();

	/**
	 * @brief ��Ŀ��ʼ������
	 */
	DLL_EXPORTS void ACS_Init();

/*
����:  true�Ѿ�����,falseδ����
*/
	DLL_EXPORTS bool ACS_InitIsOpen();    //�Ƿ��Ѿ���ʼ��


	DLL_EXPORTS void ACS_SetQtObject(void *ptr);    //����QT�źŲ۵�ַ

/*
����:  0         //����
����:  1         //ʧ��
*/
	DLL_EXPORTS int ACS_GetDeviceNum(int &deviceNum);	//��ȡ��������

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ʧ��
*/
	DLL_EXPORTS int ACS_GetDeviceInfo(int deviceId, char* strDeviceId, char* strFriendlyName, char* strDriverVersion);//��ȡ������Ϣ

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //tcp����ʧ��
����:  3         //tcp�����Ѿ�����
*/
	DLL_EXPORTS int ACS_EthernetOpen(int deviceId, char* ip,int port);    //���Ӵ�����

/*
����:  true�Ѿ�����,falseδ����
*/
	DLL_EXPORTS bool ACS_EthernetIsOpen(int deviceId);    //�Ƿ��Ѿ����Ӵ�����

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
*/
	DLL_EXPORTS int ACS_CommunicationClose(int deviceId); //�Ͽ�������

/*
��ȡ������Ϣ
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
*/
	DLL_EXPORTS int ACS_GetErrorInfo(int deviceId, char *err);

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
*/
	DLL_EXPORTS int ACS_GetAxisTotalnum(int deviceId,int &totalnum); //��ȡ������

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisEn(int deviceId, int axisId,bool enable);//������ʹ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisEn(int deviceId, int axisId, bool &enable);//��ȡ��ʹ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisSpeed(int deviceId, int axisId, double &speed);//��ȡ���ٶ�

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisSpeed(int deviceId, int axisId, double speed);//�������ٶ�

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisDeceleratedSpeed(int deviceId, int axisId, double &speed);//��ȡ���ٶ�

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisDeceleratedSpeed(int deviceId, int axisId, double speed);//���ü��ٶ�

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisAcceleratedSpeed(int deviceId, int axisId, double &speed);//��ȡ���ٶ�

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisAcceleratedSpeed(int deviceId, int axisId, double speed);//���ü��ٶ�

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisPosition(int deviceId, int axisId, double &position);//��ȡ������

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool enable);//��������λ����ʹ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool &enable);//��ȡ����λ����ʹ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisSoftwareNegativeLimit(int deviceId, int axisId, double pos);//��������λ��������

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisSoftwareNegativeLimit(int deviceId, int axisId, double &pos);//��ȡ����λ��������

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool enable);//��������λ�Ҽ���ʹ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool &enable);//��ȡ����λ�Ҽ���ʹ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisSoftwarePositiveLimit(int deviceId, int axisId, double pos);//��������λ�Ҽ�������

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��ȡʧ��
*/
	DLL_EXPORTS int ACS_GetAxisSoftwarePositiveLimit(int deviceId, int axisId, double &pos);//��ȡ����λ�Ҽ�������

/*
����:  0         //��ʼ�ƶ�(������)�������������ʱ�ᰴ�µ�λ�ƶ�
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����λ��Ϣ��ȡ�쳣
����:  5		 //��������λ
����:  6		 //�ƶ�ʧ��
*/
	DLL_EXPORTS int ACS_MoveAbsPos(int deviceId, int axisId, double pos);//���������ƶ�

/*
����:  0         //��ʼ�ƶ�(������)�������������ʱ�ᰴ�µ�λ�ƶ�
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����λ��Ϣ��ȡ�쳣
����:  5		 //��������λ
����:  6		 //�ƶ�ʧ��
*/
	DLL_EXPORTS int ACS_MoveRelPos(int deviceId, int axisId, double pos);//��������ƶ�

/*
����:           //true�����,false���쳣
*/
	DLL_EXPORTS bool ACS_GetAxisIsReady(int deviceId, int axisId);//�Ƿ������

/*
����:     mode  0:UNKNOW
				1:HOME
				2:PTP
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
*/
	DLL_EXPORTS int ACS_GetAxisMoveMode(int deviceId, int axisId,int &mode);//�����Ṥ��

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_SetAxisHomeBuffer(int deviceId, int axisId, int homebuffer);//�����Ḵλbuffer


/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //��λʧ��
*/
	DLL_EXPORTS int ACS_AxisHome(int deviceId, int axisId);//�Ḵλ

/*
����:  0         //����
����:  1         //���Ӵ�������deviceIdҪ���ڵ���0����С�ڵ���MAX_DEVICEID_NUM
����:  2         //ACS����δ����
����:  3		 //��axisId�ų���������
����:  4		 //����ʧ��
*/
	DLL_EXPORTS int ACS_AxisStop(int deviceId, int axisId);//��ֹͣ
}
#endif
