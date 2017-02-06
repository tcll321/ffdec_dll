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
	return 0;
}

FFDEC_API int np_ffdec_frame(NP_FF_DEC pff,unsigned char* bs,int len)
{
	AVCodec* pDecoder = NULL;
	AVCodecContext* pContext = NULL;
	AVFrame* pFrame = NULL;
	if (NULL == pff.ff_codec)
		pff.ff_codec = &h264_decoder;
	if (NULL == pff.ff_ct)
		pff.ff_ct = avcodec_alloc_context();
	if (NULL == pff.ff_frame)
		pff.ff_frame = avcodec_alloc_frame();

	pDecoder = (AVCodec*)pff.ff_codec;
	pContext = (AVCodecContext*)pff.ff_ct;
	pFrame = (AVFrame*)pff.ff_frame;

	pDecoder->init(pContext);
	pDecoder->decode(pContext, pff.data, &pff.size, bs, len);

	return 0;
}

FFDEC_API void np_ffdec_destroy(NP_FF_DEC pff)
{
	if (pff.ff_ct)
	{
		av_free(pff.ff_ct);
		pff.ff_ct = NULL;
	}
	if (pff.ff_frame)
	{
		av_free(pff.ff_frame);
		pff.ff_frame = NULL;
	}
}
