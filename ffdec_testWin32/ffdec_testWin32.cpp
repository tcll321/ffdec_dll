// ffdec_testWin32.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "ffdec.h"


int _tmain(int argc, _TCHAR* argv[])
{
	int ret = 0;
	NP_FF_DEC dec = {0};
	unsigned char data[1024] = {0};

	ret = np_ffdec_init(FFDEC_ID_H264);

	FILE *pf = fopen("1080p.h264", "rb");

	if (pf)
	{
		fread(data, 1, 1024, pf);
		ret = np_ffdec_frame(dec, data, 1024);
	}

	np_ffdec_destroy(dec);

	return 0;
}
