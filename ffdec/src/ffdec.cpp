// ffdec.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "ffdec.h"

#ifdef __cplusplus
extern "C"{
#endif
#include "avcodec.h"
extern AVCodec h264_decoder;

#ifdef __cplusplus
};
#endif

// This is an example of an exported function.
FFDEC_API int np_ffdec_init(NPFFDEC_ID id)
{
	register_avcodec(&h264_decoder);
	avcodec_init();

	CodecID ffID = CODEC_ID_NONE;
	if (FFDEC_ID_H264 == id)
		ffID = CODEC_ID_H264;

	avcodec_find_decoder(ffID);
	return 42;
}
