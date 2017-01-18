// ffdec_testWin32.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ffdec.h"


int _tmain(int argc, _TCHAR* argv[])
{
	int ret = 0;
	ret = np_ffdec_init(FFDEC_ID_H264);
	return 0;
}
