// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the FFDEC_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// FFDEC_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef FFDEC_EXPORTS
#define FFDEC_API __declspec(dllexport)
#else
#define FFDEC_API __declspec(dllimport)
#endif

///////////ffdecodec part//////////
typedef enum
{
	//video
	FFDEC_ID_H261,
	FFDEC_ID_H263,
	FFDEC_ID_H263I,
	FFDEC_ID_H264,
	FFDEC_ID_MPEG4,
	FFDEC_ID_MPEG1,
	FFDEC_ID_MPEG2,
	FFDEC_ID_MJPEG,
	FFDEC_ID_FLV,  //Flash Video h263
	FFDEC_ID_AVS,
	FFDEC_ID_MSMPEG4V1, //MS MPEG-4 Video v1
	FFDEC_ID_MSMPEG4V2, //MS MPEG-4 Video v2
	FFDEC_ID_MSMPEG4V3, //MS MPEG-4 Video v3
	FFDEC_ID_WMV1, //Windows Media Video 1
	FFDEC_ID_WMV2, //Windows Media Video 2
	FFDEC_ID_WMV3, //Windows Media Video 3
	FFDEC_ID_VC1,  //Windows Media Video VC1
	FFDEC_ID_PNG,
	FFDEC_ID_JPEGLS,
	//audio
	FFDEC_ID_WMAV1, //Windows Media Audio 1
	FFDEC_ID_WMAV2, //Windows Media Audio 2
	FFDEC_ID_MP2,
	FFDEC_ID_MP3,
	FFDEC_ID_AAC,
	FFDEC_ID_ADPCM_G726,
	FFDEC_ID_PCM_MULAW,
	FFDEC_ID_PCM_ALAW,
} NPFFDEC_ID;

typedef struct
{
	void* ff_ct;   //实现者使用
	void* ff_codec;//实现者使用
	void* ff_frame;//实现者使用
	int   isAudio; //实现者使用
	void* data;	//yuv or pcm16 数据 
	int   size; //yuv or pcm16 数据大小 (bytes)
	unsigned short width; 
	unsigned short height; 
} NP_FF_DEC, *PNP_FF_DEC;

FFDEC_API int np_ffdec_init(NPFFDEC_ID id);
FFDEC_API int np_ffdec_frame(NP_FF_DEC pff,unsigned char* bs,int len);
FFDEC_API void np_ffdec_destroy(NP_FF_DEC pff);
