#include "acslib.h"
#include <iostream>
#include "example.h"
using namespace acs;
int main() 
{
#ifdef DEVELOP
	main_self();
#else
	//���ó�ʼ���ӿ�
	ACS_Init();
	
	//����dump
	ACS_DumpTrigger();
#endif // DEVELOP
	return 0;
}
