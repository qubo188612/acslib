#include "acslib.h"
#include <iostream>
#include "example.h"
using namespace acs;
int main() 
{
#ifdef DEVELOP
	main_self();
#else
	//调用初始化接口
	ACS_Init();
	
	//测试dump
	ACS_DumpTrigger();
#endif // DEVELOP
	return 0;
}
