// ffdec.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "ffdec.h"
#include "avcodec.h"

// This is an example of an exported function.
FFDEC_API int np_ffdec_init(NPFFDEC_ID id)
{
	return 42;
}
