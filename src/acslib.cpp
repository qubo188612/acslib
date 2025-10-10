#include "acslib.h"
#include "commitid.h"
#include "alg_base_common.h"
#include "ACSC.h"
#include "pthread.h"
#include <mutex>
#include <QVariantMap>

namespace  acs
{
	static bool b_sdk_init = false;

	const double EPS = 0.1;						//是否到位判断

	#define USE_VIRTUAL				2			//是否使用虚拟接口:0使用sdk函数,1使用虚拟接口,2使用qtobject信号槽
	#define MAX_DEVICEID_NUM        10          //最多连接10台acs
	static bool link_ftp_state[MAX_DEVICEID_NUM];		//是否连接上ACS网络
	static HANDLE Handle[MAX_DEVICEID_NUM];			//每台ACS句柄
	static int m_axisCount[MAX_DEVICEID_NUM];			//每台ACS轴总数
	static std::mutex mtx; // 全局互斥量
	static void *m_qtobject;//qt信号槽地址
	struct struct_errinfo
	{
		bool b_err;//是否错误
		std::string str;//错误信息
	};
	static struct_errinfo m_errinfo[MAX_DEVICEID_NUM];	    //每台ACS错误信息
	enum MoveMode {		//电机工作模式
		UNKNOW=0,    //未知
		HOME=1,      //回零运动
		PTP=2        //点位运动
	};
	struct struct_axisinfo
	{
		bool *open;//是否存在电机
		bool *En;//是否电机使能
		double *speed;//速度
		double *deceleration;//减速度
		double *acceleration;//加速度
		double *position;//坐标位置
		bool *softwarenegativelimitenabled;//软限位左极限使能
		double *softwarenegativelimit;//软限位左极限
		bool *softwarepositivelimitenabled;//软限位右极限使能
		double *softwarepositivelimit;//软限位右极限
		MoveMode *movemode;	//电机当前工作模式
		bool *moveing;//电机是否再工作中
		double *targetposition;//目标移动位置
		int *homebuffernum;//复位buffer
	};
	static struct_axisinfo m_axisinfo[MAX_DEVICEID_NUM];//电机轴信息
	/*******************/
	//虚拟ACS信息
	#define VIRAXIS_NUM						20			//虚拟轴总数
	#define VIRAXIS_EN						true		//虚拟轴默认使能
	#define VIRAXIS_SPEED					200			//虚拟轴默认速度mm/s
	#define VIRAXIS_DECELERATIONSPEED		1000		//虚拟轴默认减速度mm/s
	#define VIRAXIS_ACCELERATIONSPEED		1000		//虚拟轴默认加速度mm/s
	#define VIRAXIS_POSITION				0			//虚拟轴默认坐标mm
	#define VIRAXIS_SOFTWARELIMITEN			false		//虚拟轴默认软限位使能
	#define VIRAXIS_SOFTWARELIMIT_MIN		-200		//虚拟轴默认软限位左极限
	#define VIRAXIS_SOFTWARELIMIT_MAX		200			//虚拟轴默认软限位右极限
	/******************/

#if USE_VIRTUAL == 1
	static pthread_t _virtualaxisithread[MAX_DEVICEID_NUM];
	void *_virtualaxisi(void *des);
	static bool b_virtualaxisithread_stop[MAX_DEVICEID_NUM];
	static bool b_virtualaxisithread[MAX_DEVICEID_NUM];
	struct Paramvirtualaxisithread
	{
		int deviceId;
	};
	static Paramvirtualaxisithread param_virtualaxisithread[MAX_DEVICEID_NUM];
	static pthread_mutex_t mutex_virtualaxisi = PTHREAD_MUTEX_INITIALIZER;
#endif

	void SLEEP_S(double time)
	{
		Sleep(int(time * 1000));
	}

	std::string getCommitID_appAcs()
	{
		return COMMIT_ID_APP_ACS;
	}

	void ACS_DumpTrigger()
    {
		dumpTrigger_baseCommon();
        return;
    }

	void printComId()
	{
		std::string strCommitId_baseCommon = getCommitID_baseCommon();
		LOG_INFO("default", "CommitId_baseCommon:" + strCommitId_baseCommon);
		std::cout << "CommitId_baseCommon: " << strCommitId_baseCommon << std::endl;

		std::string strCommitId_appAcs = getCommitID_appAcs();
		LOG_INFO("default", "CommitId_Acs:" + strCommitId_appAcs);
		std::cout << "CommitId_Acs: " << strCommitId_appAcs << std::endl;
	}

	void ACS_Init()
    {
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		if (b_sdk_init == false)
		{
			for (int i = 0; i < MAX_DEVICEID_NUM; i++)
			{
				link_ftp_state[i] = false;
				Handle[i] = (HANDLE)-1;
				m_axisCount[i] = 0;
				m_axisinfo[i].open = nullptr;
				m_axisinfo[i].En = nullptr;
				m_axisinfo[i].speed = nullptr;
				m_axisinfo[i].deceleration = nullptr;
				m_axisinfo[i].acceleration = nullptr;
				m_axisinfo[i].position = nullptr;
				m_axisinfo[i].softwarenegativelimitenabled = nullptr;
				m_axisinfo[i].softwarenegativelimit = nullptr;
				m_axisinfo[i].softwarepositivelimitenabled = nullptr;
				m_axisinfo[i].softwarepositivelimit = nullptr;
				m_axisinfo[i].movemode = nullptr;
				m_axisinfo[i].moveing = nullptr;
				m_axisinfo[i].targetposition = nullptr;
				m_axisinfo[i].homebuffernum = nullptr;
			}
			commonInit();
			printComId();
			b_sdk_init = true;
		}
        return ;
    }

	bool ACS_InitIsOpen()    //是否已经初始化
	{
		return b_sdk_init;
	}

	void clearErrorInfo(int deviceId)
	{
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			return;
		}
		m_errinfo[deviceId].b_err = false;
		m_errinfo[deviceId].str.clear();
	}

	void pushErrorInfo(int deviceId, std::string str)
	{
		if (deviceId < 0 || deviceId >= 10)
		{
			return;
		}
		m_errinfo[deviceId].b_err = true;
		if (!m_errinfo[deviceId].str.empty())
		{
			m_errinfo[deviceId].str += ";";  // 用分号分隔多条错误
		}
		m_errinfo[deviceId].str += str;  // 追加新错误信息
	}

	bool inRange(double pos, double targetpos)
	{
		return  ((pos - EPS) <= targetpos) && (targetpos <= (pos + EPS));
	}
#if USE_VIRTUAL==0
	bool isBufferIdle(int deviceId, int buffer)
	{
		int bufferState = 0;
		if (!acsc_GetProgramState(Handle[deviceId], buffer, &bufferState, NULL)) {
			return false;
		}
		if (!bufferState) { return true; }
		if (ACSC_PST_COMPILED != bufferState) {
			return false;
		}
		return true;
	}
#elif USE_VIRTUAL==2
	bool isBufferIdle(int deviceId, int buffer)
	{
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("isBufferIdle"));
		ininfo.insert("deviceId", deviceId);
		ininfo.insert("buffer", buffer);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo),Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onIsBufferIdle is err");
			return false;
		}
		result = outinfo.value("result").toBool();
		return result;
	}
#endif
#if USE_VIRTUAL==0
	bool stopBuffer(int deviceId, int buffer)
	{
		bool ret = true;
		int err = 0;
		if (!acsc_StopBuffer(Handle[deviceId], buffer, NULL)) {
			ret = false;
		}
		return true;
	}
#elif USE_VIRTUAL==2
	bool stopBuffer(int deviceId, int buffer)
	{
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("stopBuffer"));
		ininfo.insert("deviceId", deviceId);
		ininfo.insert("buffer", buffer);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onStopBuffer is err");
			return false;
		}
		result = outinfo.value("result").toBool();
		return result;
	}
#endif

#if USE_VIRTUAL == 1
	void virtualAxisiThreadStart(int deviceId)
	{
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			return;
		}
		b_virtualaxisithread[deviceId] = true;
		b_virtualaxisithread_stop[deviceId] = false;
		param_virtualaxisithread[deviceId].deviceId = deviceId;
		pthread_create(&_virtualaxisithread[deviceId], NULL, _virtualaxisi, (void *)(&param_virtualaxisithread[deviceId]));
	}

	void virtualAxisiThreadStop(int deviceId)
	{
		if (b_virtualaxisithread[deviceId] == true)
		{
			b_virtualaxisithread_stop[deviceId] = false;
			b_virtualaxisithread[deviceId] = false;
			while (b_virtualaxisithread_stop[deviceId] == false)
			{
				SLEEP_S(0);
			}
			pthread_join(_virtualaxisithread[deviceId], NULL);
		}
	}

	void *_virtualaxisi(void *des)
	{
		Paramvirtualaxisithread _p = *((Paramvirtualaxisithread *)des);
		int deviceId = _p.deviceId;
		while (1)
		{
			if (b_virtualaxisithread[deviceId] == true)
			{
				pthread_mutex_lock(&mutex_virtualaxisi);
				//这里更改外部数据
				for (int j = 0;j < m_axisCount[deviceId];j ++)
				{
					if (m_axisinfo[deviceId].moveing[j] == true)
					{
						if (!inRange(m_axisinfo[deviceId].position[j], m_axisinfo[deviceId].targetposition[j]))
						{
							double speed = m_axisinfo[deviceId].speed[j];
							double pos = m_axisinfo[deviceId].position[j];
							double targetpos = m_axisinfo[deviceId].targetposition[j];
							int dir = round((targetpos - pos)/fabs(targetpos - pos));
							double newpos = pos + dir*(0.1 * speed);
							if (dir == 1 && newpos > targetpos)
							{
								newpos = targetpos;
								m_axisinfo[deviceId].moveing[j] = false;
							}
							else if (dir == -1 && newpos < targetpos)
							{
								newpos = targetpos;
								m_axisinfo[deviceId].moveing[j] = false;
							}
							m_axisinfo[deviceId].position[j] = newpos;
						}
						else
						{
							m_axisinfo[deviceId].moveing[j] = false;
						}
					}
				}
				pthread_mutex_unlock(&mutex_virtualaxisi);
			}
			else
			{
				b_virtualaxisithread_stop[deviceId] = true;
				break;
			}
			SLEEP_S(0.1);//每100ms刷新一次数据
		}
		return nullptr;
	}
#endif

#if USE_VIRTUAL==0
	std::string _executeError(HANDLE Handle, int err)
	{
		int errNum = err;
		int received = 0;
		char errString[256];

		if (acsc_GetErrorString(Handle,
			errNum,
			errString,
			255,
			&received
		)) {
			std::string str(errString);
			return str;
		}
		else
		{
			std::string str = "Get execute err";
			return str;
		}
	}
#elif USE_VIRTUAL==2
	std::string _executeError(HANDLE Handle, int err)
	{
		std::string str = "";
		return str;
	}
#endif

	int _GetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool &enable)//获取软限位左极限使能
	{
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		enable = m_axisinfo[deviceId].softwarenegativelimitenabled[axisId];
		return 0;
	}

	int _GetAxisSoftwareNegativeLimit(int deviceId, int axisId, double &pos)//获取软限位左极限坐标
	{
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
#if USE_VIRTUAL==0
		std::string variableName = "SLLIMIT";
		double NLimit[1];
		if (!acsc_ReadReal(Handle[deviceId], ACSC_NONE, variableName.data(), axisId, axisId, ACSC_NONE, ACSC_NONE, NLimit, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].softwarenegativelimit[axisId] = NLimit[0];
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsSoftWareLimit"));
		ininfo.insert("mode",QString("SLLIMIT"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsSoftWareLimit is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].softwarenegativelimit[axisId] = result;
#endif
		pos = m_axisinfo[deviceId].softwarenegativelimit[axisId];
		return 0;
	}

	int _GetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool &enable)//获取软限位右极限使能
	{
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		enable = m_axisinfo[deviceId].softwarepositivelimitenabled[axisId];
		return 0;
	}

	int _GetAxisSoftwarePositiveLimit(int deviceId, int axisId, double &pos)//获取软限位右极限坐标
	{
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
#if USE_VIRTUAL==0
		std::string variableName = "SRLIMIT";
		double PLimit[1];
		if (!acsc_ReadReal(Handle[deviceId], ACSC_NONE, variableName.data(), axisId, axisId, ACSC_NONE, ACSC_NONE, PLimit, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].softwarepositivelimit[axisId] = PLimit[0];
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsSoftWareLimit"));
		ininfo.insert("mode", QString("SRLIMIT"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsSoftWareLimit is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].softwarepositivelimit[axisId] = result;
#endif
		pos = m_axisinfo[deviceId].softwarepositivelimit[axisId];
		return 0;
	}

	int _SetAxisEn(int deviceId, int axisId, bool enable)//设置使能
	{
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		if (true == enable) {
#if USE_VIRTUAL==0
			if (!acsc_Enable(Handle[deviceId], axisId, NULL)) {
				return 4;
			}
#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAcsEnable"));
			ininfo.insert("enable", true);
			ininfo.insert("axisId", axisId);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAcsEnable is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
#endif
			m_axisinfo[deviceId].En[deviceId] = true;
		}
		else {
#if USE_VIRTUAL==0
			if (!acsc_Disable(Handle[deviceId], axisId, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return 4;
			}
#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAcsEnable"));
			ininfo.insert("enable", false);
			ininfo.insert("axisId", axisId);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAcsEnable is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
#endif
			m_axisinfo[deviceId].En[deviceId] = false;
		}
		return 0;
	}
#if USE_VIRTUAL==0
	std::string ACS_executeError(HANDLE Handle, int err)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		int errNum = err;
		int received = 0;
		char errString[256];
		if (acsc_GetErrorString(Handle,
			errNum,
			errString,
			255,
			&received
		)) {
			std::string str(errString);
			return str;
		}
		else
		{
			std::string str="Get execute err";
			return str;
		}
	}
#endif
	int ACS_EthernetOpen(int deviceId, char *ip, int port)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId,"Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] != true)
		{
#if USE_VIRTUAL==0
			std::string s_ip(ip);
			Handle[deviceId] = acsc_OpenCommEthernetTCP(s_ip.data(), port);
			if (ACSC_INVALID == Handle[deviceId]) {	
				pushErrorInfo(deviceId, "Device connection failed");
				return 2;
			}
			double axisCount = 0;
			if (!acsc_GetAxesCount(Handle[deviceId], &axisCount, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				acsc_CloseComm(Handle[deviceId]);
				return 2;
			}
			m_axisCount[deviceId] = axisCount;
#elif USE_VIRTUAL==1
			m_axisCount[deviceId] = VIRAXIS_NUM;
#elif USE_VIRTUAL==2
			int result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", "GetAxisCount");
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onGetAxisCount is err");
				return 4;
			}
			result = outinfo.value("result").toInt();
			m_axisCount[deviceId] = result;
#endif
			if (m_axisinfo[deviceId].open!=nullptr)
			{
				pushErrorInfo(deviceId, "The memory is not empty");
				return 2;
			}
			if (m_axisCount[deviceId]==0)
			{
				pushErrorInfo(deviceId, "The total number of axes is 0");
				return 2;
			}
			m_axisinfo[deviceId].open = new bool[m_axisCount[deviceId]];
			m_axisinfo[deviceId].En = new bool[m_axisCount[deviceId]];
			m_axisinfo[deviceId].speed = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].deceleration = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].acceleration = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].position = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].softwarenegativelimitenabled = new bool[m_axisCount[deviceId]];
			m_axisinfo[deviceId].softwarenegativelimit = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].softwarepositivelimitenabled = new bool[m_axisCount[deviceId]];
			m_axisinfo[deviceId].softwarepositivelimit = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].movemode = new MoveMode[m_axisCount[deviceId]];
			m_axisinfo[deviceId].moveing = new bool[m_axisCount[deviceId]];
			m_axisinfo[deviceId].targetposition = new double[m_axisCount[deviceId]];
			m_axisinfo[deviceId].homebuffernum = new int[m_axisCount[deviceId]];
			for (int j=0;j< m_axisCount[deviceId];j++)
			{
				m_axisinfo[deviceId].open[j] = true;
				m_axisinfo[deviceId].movemode[j] = UNKNOW;
				m_axisinfo[deviceId].moveing[j] = false;//初次连接时默认电机没在动
				m_axisinfo[deviceId].targetposition[j] = 0;
				m_axisinfo[deviceId].homebuffernum[j] = 0;
		#if USE_VIRTUAL==1
				m_axisinfo[deviceId].En[j]= VIRAXIS_EN;
				m_axisinfo[deviceId].speed[j] = VIRAXIS_SPEED;
				m_axisinfo[deviceId].deceleration[j] = VIRAXIS_DECELERATIONSPEED;
				m_axisinfo[deviceId].acceleration[j] = VIRAXIS_ACCELERATIONSPEED;
				m_axisinfo[deviceId].position[j] = VIRAXIS_POSITION;
				m_axisinfo[deviceId].softwarenegativelimitenabled[j] = VIRAXIS_SOFTWARELIMITEN;
				m_axisinfo[deviceId].softwarenegativelimit[j] = VIRAXIS_SOFTWARELIMIT_MIN;
				m_axisinfo[deviceId].softwarepositivelimitenabled[j] = VIRAXIS_SOFTWARELIMITEN;
				m_axisinfo[deviceId].softwarepositivelimit[j] = VIRAXIS_SOFTWARELIMIT_MAX;
		#endif
			}
		#if USE_VIRTUAL == 1
			b_virtualaxisithread[deviceId] = false;
			b_virtualaxisithread_stop[deviceId] = false;
			virtualAxisiThreadStart(deviceId);
		#endif
			link_ftp_state[deviceId] = true;
		}
		else
		{
			pushErrorInfo(deviceId, "The drive has been connected");
			return 3;
		}
		return 0;
	}

	bool ACS_EthernetIsOpen(int deviceId)    //是否已经连接传感器
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return false;
		}
		return link_ftp_state[deviceId];
	}

	void ACS_SetQtObject(void *ptr)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		m_qtobject = ptr;
	}

	int ACS_GetDeviceNum(int &deviceNum)	//获取驱动总数
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
#if USE_VIRTUAL == 1
		deviceNum=MAX_DEVICEID_NUM;	//虚拟返回10个驱动器
#else
		deviceNum=1;	//实际返回
#endif
		return 0;
	}

	int ACS_GetDeviceInfo(int deviceId, char* strDeviceId, char* strFriendlyName, char* strDriverVersion)//获取驱动信息
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
#if USE_VIRTUAL == 1
		std::string s_str1= "VirtualACSDevice ";
		std::string s_str2 = "VirtualACS Stage";
#else
		std::string s_str1 = "ACSDevice ";
		std::string s_str2 = "ACS Stage";
#endif
		std::string s_strDriverVersion = "v 1.0.0";
		std::string s_deviceId = std::to_string(deviceId); // 将整数转换为字符串
		std::string s_strDeviceId = s_str1 + s_deviceId;
		std::string s_strFriendlyName = s_str2 + s_deviceId;
		strcpy(strDeviceId, s_strDeviceId.c_str());
		strcpy(strFriendlyName, s_strFriendlyName.c_str());
		strcpy(strDriverVersion, s_strDriverVersion.c_str());
		return 0;
	}

	int ACS_CommunicationClose(int deviceId)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == true)
		{
	#if USE_VIRTUAL == 1
			virtualAxisiThreadStop(deviceId);
	#endif
			delete[]m_axisinfo[deviceId].open;
			delete[]m_axisinfo[deviceId].En;
			delete[]m_axisinfo[deviceId].speed;
			delete[]m_axisinfo[deviceId].deceleration;
			delete[]m_axisinfo[deviceId].acceleration;
			delete[]m_axisinfo[deviceId].position;
			delete[]m_axisinfo[deviceId].softwarenegativelimitenabled;
			delete[]m_axisinfo[deviceId].softwarenegativelimit;
			delete[]m_axisinfo[deviceId].softwarepositivelimitenabled;
			delete[]m_axisinfo[deviceId].softwarepositivelimit;
			delete[]m_axisinfo[deviceId].movemode;
			delete[]m_axisinfo[deviceId].moveing;
			delete[]m_axisinfo[deviceId].targetposition;
			delete[]m_axisinfo[deviceId].homebuffernum;
			m_axisinfo[deviceId].open = nullptr;
			m_axisinfo[deviceId].En = nullptr;
			m_axisinfo[deviceId].speed = nullptr;
			m_axisinfo[deviceId].deceleration = nullptr;
			m_axisinfo[deviceId].acceleration = nullptr;
			m_axisinfo[deviceId].position = nullptr;
			m_axisinfo[deviceId].softwarenegativelimitenabled = nullptr;
			m_axisinfo[deviceId].softwarenegativelimit = nullptr;
			m_axisinfo[deviceId].softwarepositivelimitenabled = nullptr;
			m_axisinfo[deviceId].softwarepositivelimit = nullptr;
			m_axisinfo[deviceId].movemode = nullptr;
			m_axisinfo[deviceId].moveing = nullptr;
			m_axisinfo[deviceId].targetposition = nullptr;
			m_axisinfo[deviceId].homebuffernum = nullptr;
#if USE_VIRTUAL==0
			acsc_CloseComm(Handle[deviceId]);
#endif
			link_ftp_state[deviceId] = false;
		}
		return 0;
	}

	int ACS_GetErrorInfo(int deviceId, char *err)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		std::string s_err;
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (m_errinfo[deviceId].b_err == false)
		{
			s_err = "no err";
		}
		else
		{
			s_err = m_errinfo[deviceId].str;
		}
		strcpy(err, s_err.c_str());
		return 0;
	}

	int ACS_GetAxisTotalnum(int deviceId, int &totalnum) //获取轴总数
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		totalnum = m_axisCount[deviceId];
		return 0;
	}

	int ACS_SetAxisEn(int deviceId, int axisId,bool enable)//设置使能
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		if (true == enable) {
		#if USE_VIRTUAL==0
			if (!acsc_Enable(Handle[deviceId], axisId, NULL)) {
				return 4;
			}
		#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAcsEnable"));
			ininfo.insert("enable", true);
			ininfo.insert("axisId", axisId);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAcsEnable is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
		#endif
			m_axisinfo[deviceId].En[deviceId] = true;
		}
		else {
		#if USE_VIRTUAL==0
			if (!acsc_Disable(Handle[deviceId], axisId, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return 4;
			}
		#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAcsEnable"));
			ininfo.insert("enable", false);
			ininfo.insert("axisId", axisId);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAcsEnable is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
		#endif
			m_axisinfo[deviceId].En[deviceId] = false;
		}
		return 0;
	}

	int ACS_GetAxisEn(int deviceId, int axisId, bool &enable)//获取轴使能
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
#if USE_VIRTUAL==0
		int motionState = 0;
		if (!acsc_GetMotorState(Handle[deviceId], axisId, &motionState, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		bool en= (ACSC_MST_ENABLE == (motionState & ACSC_MST_ENABLE));
		m_axisinfo[deviceId].En[deviceId] = en;
#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsEnable"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsEnable is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
		m_axisinfo[deviceId].En[deviceId] = result;
#endif
		enable = m_axisinfo[deviceId].En[deviceId];
		return 0;
	}

	int ACS_GetAxisSpeed(int deviceId, int axisId, double &speed)//获取轴速度
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double velocity = 0;
#if USE_VIRTUAL==0
		if (!acsc_GetVelocity(Handle[deviceId], axisId, &velocity, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].speed[axisId] = velocity;
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsSpeed"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsSpeed is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].speed[axisId] = result;
#endif
		speed = m_axisinfo[deviceId].speed[axisId];
		return 0;
	}

	int ACS_SetAxisSpeed(int deviceId, int axisId, double speed)//设置轴速度
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double velocity = speed;
#if USE_VIRTUAL==0
		if (!acsc_SetVelocity(Handle[deviceId], axisId, velocity, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("SetAcsSpeed"));
		ininfo.insert("axisId", axisId);
		ininfo.insert("speed", speed);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onSetAcsSpeed is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
#endif
		m_axisinfo[deviceId].speed[axisId] = velocity;
		return 0;
	}

	int ACS_GetAxisDeceleratedSpeed(int deviceId, int axisId, double &speed)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double deceleration = 0;
#if USE_VIRTUAL==0
		if (!acsc_GetDeceleration(Handle[deviceId], axisId, &deceleration, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].deceleration[axisId] = deceleration;
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetDecelerationSpeed"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetDecelerationSpeed is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].deceleration[axisId] = result;
#endif
		speed = m_axisinfo[deviceId].deceleration[axisId];
		return 0;
	}

	int ACS_SetAxisDeceleratedSpeed(int deviceId, int axisId, double speed)//设置减速度
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double deceleration = speed;
#if USE_VIRTUAL==0
		if (!acsc_SetDeceleration(Handle[deviceId], axisId, deceleration, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("SetDecelerationSpeed"));
		ininfo.insert("axisId", axisId);
		ininfo.insert("deceleration", deceleration);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetDecelerationSpeed is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
#endif
		m_axisinfo[deviceId].deceleration[axisId] = deceleration;
		return 0;
	}

	int ACS_GetAxisAcceleratedSpeed(int deviceId, int axisId, double &speed)//获取加速度
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double acceleration = 0;
#if USE_VIRTUAL==0
		if (!acsc_GetAcceleration(Handle[deviceId], axisId, &acceleration, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].acceleration[axisId] = acceleration;
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAccelerationSpeed"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAccelerationSpeed is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].acceleration[axisId] = result;
#endif
		speed = m_axisinfo[deviceId].acceleration[axisId];
		return 0;
	}

	int ACS_SetAxisAcceleratedSpeed(int deviceId, int axisId, double speed)//设置加速度
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double acceleration = speed;
#if USE_VIRTUAL==0
		if (!acsc_SetAcceleration(Handle[deviceId], axisId, acceleration, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("SetAccelerationSpeed"));
		ininfo.insert("axisId", axisId);
		ininfo.insert("acceleration", acceleration);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onSetAccelerationSpeed is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
#endif
		m_axisinfo[deviceId].acceleration[axisId] = acceleration;
		return 0;
	}

	int ACS_GetAxisPosition(int deviceId, int axisId, double &position)//获取轴坐标
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double pos = 0;
#if USE_VIRTUAL==0
		if (!acsc_GetFPosition(Handle[deviceId], axisId, &pos, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].position[axisId] = pos;
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsPosition"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsPosition is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].position[axisId] = result;
#endif
		position = m_axisinfo[deviceId].position[axisId];
		return 0;
	}

	int ACS_SetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool enable)//软限位左极限使能
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		if (true == enable) {
		#if USE_VIRTUAL==0
			if (!acsc_EnableFault(Handle[deviceId], axisId, ACSC_SAFETY_SLL, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return 4;
			}
		#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAxisSoftwareNegativeLimitEnabled"));
			ininfo.insert("axisId", axisId);
			ininfo.insert("enable", true);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAxisSoftwareNegativeLimitEnabled is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
		#endif
			m_axisinfo[deviceId].softwarenegativelimitenabled[axisId] = true;
		}
		else {
		#if USE_VIRTUAL==0
			if (!acsc_DisableFault(Handle[deviceId], axisId, ACSC_SAFETY_SLL, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return 4;
			}
		#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAxisSoftwareNegativeLimitEnabled"));
			ininfo.insert("axisId", axisId);
			ininfo.insert("enable", false);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAxisSoftwareNegativeLimitEnabled is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
		#endif
			m_axisinfo[deviceId].softwarenegativelimitenabled[axisId] = false;
		}
		return 0;
	}

	int ACS_GetAxisSoftwareNegativeLimitEnabled(int deviceId, int axisId, bool &enable)//获取软限位左极限使能
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		enable = m_axisinfo[deviceId].softwarenegativelimitenabled[axisId];
		return 0;
	}

	int ACS_SetAxisSoftwareNegativeLimit(int deviceId, int axisId, double pos)//设置软限位左极限坐标
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
	#if USE_VIRTUAL==0
		std::string variableName = "SLLIMIT";
		double NLimit[1] = {pos};
		if (!acsc_WriteReal(Handle[deviceId], ACSC_NONE, variableName.data(), axisId, axisId, ACSC_NONE, ACSC_NONE, NLimit, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
	#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("SetAcsSoftWareLimit"));
		ininfo.insert("mode", QString("SLLIMIT"));
		ininfo.insert("axisId", axisId);
		ininfo.insert("pos", pos);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onSetAcsSoftWareLimit is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
	#endif
		m_axisinfo[deviceId].softwarenegativelimit[axisId] = pos;
		return 0;
	}
	
	int ACS_GetAxisSoftwareNegativeLimit(int deviceId, int axisId, double &pos)//获取软限位左极限坐标
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
	#if USE_VIRTUAL==0
		std::string variableName = "SLLIMIT";
		double NLimit[1];
		if (!acsc_ReadReal(Handle[deviceId], ACSC_NONE, variableName.data(), axisId, axisId, ACSC_NONE, ACSC_NONE, NLimit, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].softwarenegativelimit[axisId] = NLimit[0];
	#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsSoftWareLimit"));
		ininfo.insert("mode", QString("SLLIMIT"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsSoftWareLimit is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].softwarepositivelimit[axisId] = result;
	#endif
		pos = m_axisinfo[deviceId].softwarenegativelimit[axisId];
		return 0;
	}

	int ACS_SetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool enable)//设置软限位右极限使能
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		if (true == enable) {
		#if USE_VIRTUAL==0
			if (!acsc_EnableFault(Handle[deviceId], axisId, ACSC_SAFETY_SRL, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return 4;
			}
		#elif USE_VIRTUAL==2
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("SetAxisSoftwarePositiveLimitEnabled"));
			ininfo.insert("axisId", axisId);
			ininfo.insert("enable", true);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onSetAxisSoftwarePositiveLimitEnabled is err");
				return 4;
			}
			result = outinfo.value("result").toBool();
		#endif
			m_axisinfo[deviceId].softwarepositivelimitenabled[axisId] = true;
		}
		else {
		#if USE_VIRTUAL==0
			if (!acsc_DisableFault(Handle[deviceId], axisId, ACSC_SAFETY_SRL, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return 4;
			}
		#endif
			m_axisinfo[deviceId].softwarepositivelimitenabled[axisId] = false;
		}
		return 0;
	}

	int ACS_GetAxisSoftwarePositiveLimitEnabled(int deviceId, int axisId, bool &enable)//获取软限位右极限使能
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		enable = m_axisinfo[deviceId].softwarepositivelimitenabled[axisId];
		return 0;
	}

	int ACS_SetAxisSoftwarePositiveLimit(int deviceId, int axisId, double pos)//设置软限位右极限坐标
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
#if USE_VIRTUAL==0
		std::string variableName = "SRLIMIT";
		double PLimit[1] = { pos };
		if (!acsc_WriteReal(Handle[deviceId], ACSC_NONE, variableName.data(), axisId, axisId, ACSC_NONE, ACSC_NONE, PLimit, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("SetAcsSoftWareLimit"));
		ininfo.insert("mode", QString("SRLIMIT"));
		ininfo.insert("axisId", axisId);
		ininfo.insert("pos", pos);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onSetAcsSoftWareLimit is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
#endif
		m_axisinfo[deviceId].softwarepositivelimit[axisId] = pos;
		return 0;
	}

	int ACS_GetAxisSoftwarePositiveLimit(int deviceId, int axisId, double &pos)//获取软限位右极限坐标
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
#if USE_VIRTUAL==0
		std::string variableName = "SRLIMIT";
		double PLimit[1];
		if (!acsc_ReadReal(Handle[deviceId], ACSC_NONE, variableName.data(), axisId, axisId, ACSC_NONE, ACSC_NONE, PLimit, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].softwarepositivelimit[axisId] = PLimit[0];
#elif USE_VIRTUAL==2
		double result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAcsSoftWareLimit"));
		ininfo.insert("mode", QString("SLLIMIT"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAcsSoftWareLimit is err");
			return 4;
		}
		result = outinfo.value("result").toDouble();
		m_axisinfo[deviceId].softwarepositivelimit[axisId] = result;
#endif
		pos = m_axisinfo[deviceId].softwarepositivelimit[axisId];
		return 0;
	}

	int ACS_MoveAbsPos(int deviceId, int axisId, double pos)//绝对坐标移动
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		bool softwareNegativeLimitEnabled=false;
		double softwareNegativeLimit;
		bool softwarePositiveLimitEnabled = false;
		double softwarePositiveLimit;
		if (0!=_GetAxisSoftwareNegativeLimitEnabled(deviceId, axisId, softwareNegativeLimitEnabled)) return 4;
		if (0!=_GetAxisSoftwareNegativeLimit(deviceId, axisId, softwareNegativeLimit)) return 4;
		if (0 != _GetAxisSoftwarePositiveLimitEnabled(deviceId, axisId, softwarePositiveLimitEnabled)) return 4;
		if (0 != _GetAxisSoftwarePositiveLimit(deviceId, axisId, softwarePositiveLimit)) return 4;
		if ((softwareNegativeLimitEnabled == true && pos < softwareNegativeLimit)||(softwarePositiveLimitEnabled==true&&pos> softwarePositiveLimit))
		{
			pushErrorInfo(deviceId, "The current position of the movement exceeds the soft limit");
			return 5;
		}
	#if USE_VIRTUAL==0
		if (!acsc_ToPoint(Handle[deviceId], 0, axisId, pos, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 6;
		}
	#elif USE_VIRTUAL==1
		if (m_axisinfo[deviceId].En[axisId]==false)
		{
			pushErrorInfo(deviceId, "Axis is disabled");
			return 6;
		}
	#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("MoveAbsPos"));
		ininfo.insert("axisId", axisId);
		ininfo.insert("pos", pos);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onMoveAbsPos is err");
			return 6;
		}
		result = outinfo.value("result").toBool();
	#endif
		m_axisinfo[deviceId].movemode[axisId] = PTP;
		m_axisinfo[deviceId].targetposition[axisId] = pos;
		m_axisinfo[deviceId].moveing[axisId] = true;
		return 0;
	}

	int ACS_MoveRelPos(int deviceId, int axisId, double pos)//相对坐标移动
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		double relpos;
#if USE_VIRTUAL==0
		if (!acsc_GetFPosition(Handle[deviceId], axisId, &relpos, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 6;
		}
		m_axisinfo[deviceId].position[axisId] = relpos;
#elif USE_VIRTUAL==2
		{
			double result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("GetAcsPosition"));
			ininfo.insert("axisId", axisId);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onGetAcsPosition is err");
				return 4;
			}
			result = outinfo.value("result").toDouble();
			m_axisinfo[deviceId].position[axisId] = result;
		}
#endif
		relpos = m_axisinfo[deviceId].position[axisId];
		double targetpos = relpos + pos;
		bool softwareNegativeLimitEnabled = false;
		double softwareNegativeLimit;
		bool softwarePositiveLimitEnabled = false;
		double softwarePositiveLimit;
		if (0 != _GetAxisSoftwareNegativeLimitEnabled(deviceId, axisId, softwareNegativeLimitEnabled)) return 4;
		if (0 != _GetAxisSoftwareNegativeLimit(deviceId, axisId, softwareNegativeLimit)) return 4;
		if (0 != _GetAxisSoftwarePositiveLimitEnabled(deviceId, axisId, softwarePositiveLimitEnabled)) return 4;
		if (0 != _GetAxisSoftwarePositiveLimit(deviceId, axisId, softwarePositiveLimit)) return 4;
		if ((softwareNegativeLimitEnabled == true && targetpos < softwareNegativeLimit) || (softwarePositiveLimitEnabled == true && targetpos > softwarePositiveLimit))
		{
			pushErrorInfo(deviceId, "The current position of the movement exceeds the soft limit");
			return 5;
		}
#if USE_VIRTUAL==0
		if (!acsc_ToPoint(Handle[deviceId], ACSC_AMF_RELATIVE, axisId, targetpos, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 6;
		}
#elif USE_VIRTUAL==1
		if (m_axisinfo[deviceId].En[axisId] == false)
		{
			pushErrorInfo(deviceId, "Axis is disabled");
			return 6;
		}
#elif USE_VIRTUAL==2
		{
			bool result;
			QVariantMap outinfo;
			QVariantMap ininfo;
			ininfo.insert("cmd", QString("MoveAbsPos"));
			ininfo.insert("axisId", axisId);
			ininfo.insert("pos", targetpos);
			bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
			if (false == success) {
				pushErrorInfo(deviceId, "onMoveAbsPos is err");
				return 6;
			}
			result = outinfo.value("result").toBool();
		}
#endif
		m_axisinfo[deviceId].movemode[axisId] = PTP;
		m_axisinfo[deviceId].targetposition[axisId] = targetpos;
		m_axisinfo[deviceId].moveing[axisId] = true;
		return 0;
	}

	bool ACS_GetAxisIsReady(int deviceId, int axisId)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return false;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return false;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return false;
		}
	#if USE_VIRTUAL==0
		double pos;
		if (!acsc_GetFPosition(Handle[deviceId], axisId, &pos, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return false;
		}
		m_axisinfo[deviceId].position[axisId] = pos;
		if (m_axisinfo[deviceId].movemode[axisId] == HOME)
		{
			int bufferState = 0;
			if (!acsc_GetProgramState(Handle[deviceId], m_axisinfo[deviceId].homebuffernum[axisId], &bufferState, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return false;
			}
			if (ACSC_PST_RUN == (ACSC_PST_RUN & bufferState)) {
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			int motorError = 0;
			if (!acsc_GetMotorError(Handle[deviceId], axisId, &motorError, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return false;
			}
			if (motorError > 0) {
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			m_axisinfo[deviceId].moveing[axisId] = false;
			return true;
		}
		else if (m_axisinfo[deviceId].movemode[axisId] == PTP)
		{
			int motionState = 0;
			if (!acsc_GetMotorState(Handle[deviceId], axisId, &motionState, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return false;
			}
			if (ACSC_MST_MOVE == (motionState & ACSC_MST_MOVE))
			{
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			if (ACSC_MST_INPOS != (motionState & ACSC_MST_INPOS))
			{
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			int axisFault = 0;
			if (!acsc_GetFault(Handle[deviceId], axisId, &axisFault, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return false;
			}
			if (ACSC_SAFETY_DRIVE == (axisFault & ACSC_SAFETY_DRIVE)) {
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			if (ACSC_SAFETY_ES == (axisFault & ACSC_SAFETY_ES)) {
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			int motorError = 0;
			if (!acsc_GetMotorError(Handle[deviceId], axisId, &motorError, NULL)) {
				pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
				return false;
			}
			if (motorError > 0) {
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			if (!inRange(m_axisinfo[deviceId].targetposition[axisId], pos)) {
				m_axisinfo[deviceId].moveing[axisId] = true;
				return false;
			}
			m_axisinfo[deviceId].moveing[axisId] = false;
			return true;
		}
		else
		{
			m_axisinfo[deviceId].moveing[axisId] = false;
			return true;
		}
	#elif USE_VIRTUAL==1
		if (false == m_axisinfo[deviceId].moveing[axisId])
		{
			return true;
		}
		else
		{
			return false;
		}
	#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("GetAxisIsReady"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onGetAxisIsReady is err");
			return false;
		}
		result = outinfo.value("result").toBool();
		if (result == true)
		{
			m_axisinfo[deviceId].moveing[axisId] = false;
			return true;
		}
		else
		{
			m_axisinfo[deviceId].moveing[axisId] = true;
			return false;
		}
	#endif
		return true;
	}

	int ACS_GetAxisMoveMode(int deviceId, int axisId,int &mode)//返回轴工作
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		mode=m_axisinfo[deviceId].movemode[axisId];
		return 0;
	}

	int ACS_SetAxisHomeBuffer(int deviceId, int axisId, int homebuffer)
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
		m_axisinfo[deviceId].homebuffernum[axisId] = homebuffer;
		return 0;
	}

	int ACS_AxisHome(int deviceId, int axisId)//轴复位
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
#if USE_VIRTUAL==0
		int bufferState = 0;
		if (!acsc_GetProgramState(Handle[deviceId], m_axisinfo[deviceId].homebuffernum[axisId], &bufferState, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		if (ACSC_PST_RUN == (ACSC_PST_RUN & bufferState)) return 0;

		if (!acsc_CompileBuffer(Handle[deviceId], m_axisinfo[deviceId].homebuffernum[axisId], NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		m_axisinfo[deviceId].targetposition[axisId] = 0;
		m_axisinfo[deviceId].movemode[axisId] = HOME;
		m_axisinfo[deviceId].moveing[axisId] = true;
		//根据homeBuffer运行相应的buffer
		if (!acsc_RunBuffer(Handle[deviceId], m_axisinfo[deviceId].homebuffernum[axisId], NULL, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
#elif USE_VIRTUAL == 1
		m_axisinfo[deviceId].targetposition[axisId] = 0;
		m_axisinfo[deviceId].movemode[axisId] = HOME;
		m_axisinfo[deviceId].moveing[axisId] = true;
#elif USE_VIRTUAL == 2
		m_axisinfo[deviceId].targetposition[axisId] = 0;
		m_axisinfo[deviceId].movemode[axisId] = HOME;
		m_axisinfo[deviceId].moveing[axisId] = true;
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("AxisHome"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onAxisHome is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
#endif
		return 0;
	}

	int ACS_AxisStop(int deviceId, int axisId)//轴停止
	{
		std::lock_guard<std::mutex> lock(mtx); // 进入函数时加锁，离开时自动解锁
		clearErrorInfo(deviceId);
		if (deviceId < 0 || deviceId >= MAX_DEVICEID_NUM)
		{
			pushErrorInfo(deviceId, "Device number exceeds threshold");
			return 1;
		}
		if (link_ftp_state[deviceId] == false)
		{
			pushErrorInfo(deviceId, "The drive is not connected.");
			return 2;
		}
		if (axisId >= m_axisCount[deviceId])
		{
			pushErrorInfo(deviceId, "AxisId number exceeds threshold");
			return 3;
		}
	#if USE_VIRTUAL==0
		if (!isBufferIdle(deviceId, m_axisinfo[deviceId].homebuffernum[axisId])) {
			int rc;
			rc= _SetAxisEn(deviceId, axisId, false);
			if (rc != 0) return 4;
			m_axisinfo[deviceId].En[axisId] = false;
			bool ret = stopBuffer(deviceId, m_axisinfo[deviceId].homebuffernum[axisId]);
			if (!ret) return 4;
			rc = _SetAxisEn(deviceId, axisId, true);
			if (rc != 0) return 4;
			m_axisinfo[deviceId].En[axisId] = true;
		}
		if (!acsc_Halt(Handle[deviceId], deviceId, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
		//轴停止把buffer也停止
		if (!acsc_StopBuffer(Handle[deviceId], deviceId, NULL)) {
			pushErrorInfo(deviceId, _executeError(Handle[deviceId], acsc_GetLastError()));
			return 4;
		}
	#elif USE_VIRTUAL==2
		bool result;
		QVariantMap outinfo;
		QVariantMap ininfo;
		ininfo.insert("cmd", QString("AxisStop"));
		ininfo.insert("axisId", axisId);
		bool success = QMetaObject::invokeMethod((QObject*)m_qtobject, "onACScmd", Q_RETURN_ARG(QVariantMap, outinfo), Q_ARG(QVariantMap, ininfo));
		if (false == success) {
			pushErrorInfo(deviceId, "onAxisStop is err");
			return 4;
		}
		result = outinfo.value("result").toBool();
	#endif
		m_axisinfo[deviceId].moveing[axisId] = false;
		return 0;
	}
}

