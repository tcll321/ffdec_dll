#include "dsputil.h"
#include "avcodec.h"
#include "mpegvideo.h"
#include "h264.h"
#include "h264data.h"
#include "h264_parser.h"
#include "golomb.h"
#include "rectangle.h"
#include "define.h"
#include "internal.h"
#include<string.h>
#include "cabac.h"
#ifdef ARCH_X86
#include "i386/h264_i386.h"
#endif

//#undef NDEBUG
#include <assert.h>

#define DELAYED_PIC_REF 4

static VLC coeff_token_vlc[4];
static VLC chroma_dc_coeff_token_vlc;

static VLC total_zeros_vlc[15];
static VLC chroma_dc_total_zeros_vlc[3];

static VLC run_vlc[6];
static VLC run7_vlc;

static void svq3_luma_dc_dequant_idct_c(DCTELEM *block, int qp);
static void svq3_add_idct_c(uint8_t *dst, DCTELEM *block, int stride, int qp, int dc);
static void filter_mb( H264Context *h, int mb_x, int mb_y, uint8_t *img_y, uint8_t *img_cb, uint8_t *img_cr, unsigned int linesize, unsigned int uvlinesize);
static void filter_mb_fast( H264Context *h, int mb_x, int mb_y, uint8_t *img_y, uint8_t *img_cb, uint8_t *img_cr, unsigned int linesize, unsigned int uvlinesize);

static av_always_inline uint32_t pack16to32(int a, int b){
#ifdef WORDS_BIGENDIAN
   return (b&0xFFFF) + (a<<16);
#else
   return (a&0xFFFF) + (b<<16);
#endif
}

const uint8_t ff_rem6[52]={
0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3,
};
const uint8_t ff_log2_tab[256]={
        0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
        5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
        6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
        6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
        7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};
const uint8_t ff_div6[52]={
0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8,
};


static const uint8_t *decode_nal(H264Context *h, const uint8_t *src, int *dst_length, int *consumed, int length){
    int i, si, di;
    uint8_t *dst;
    int bufidx;

    h->nal_ref_idc= src[0]>>5;
    h->nal_unit_type= src[0]&0x1F;

    src++; length--;

    for(i=0; i+1<length; i+=2){
        if(src[i]) continue;
        if(i>0 && src[i-1]==0) i--;
        if(i+2<length && src[i+1]==0 && src[i+2]<=3){
            if(src[i+2]!=3){
                /* startcode, so we must be past the end */
                length=i;
            }
            break;
        }
    }

    if(i>=length-1){ //no escaped 0
        *dst_length= length;
        *consumed= length+1; //+1 for the header
        return src;
    }

    bufidx = h->nal_unit_type == NAL_DPC ? 1 : 0; // use second escape buffer for inter data
    h->rbsp_buffer[bufidx]= av_fast_realloc(h->rbsp_buffer[bufidx], &h->rbsp_buffer_size[bufidx], length);
    dst= h->rbsp_buffer[bufidx];

    if (dst == NULL){
        return NULL;
    }

//printf("decoding esc\n");
    si=di=0;
    while(si<length){
        //remove escapes (very rare 1:2^22)
        if(si+2<length && src[si]==0 && src[si+1]==0 && src[si+2]<=3){
            if(src[si+2]==3){ //escape
                dst[di++]= 0;
                dst[di++]= 0;
                si+=3;
                continue;
            }else //next start code
                break;
        }

        dst[di++]= src[si++];
    }

    *dst_length= di;
    *consumed= si + 1;//+1 for the header
//FIXME store exact number of bits in the getbitcontext (it is needed for decoding)
    return dst;
}

/**
 * identifies the exact end of the bitstream
 * @return the length of the trailing, or 0 if damaged
 */
static int decode_rbsp_trailing(H264Context *h, const uint8_t *src){
    int v= *src;
    int r;

    tprintf(h->s.avctx, "rbsp trailing %d\n", v);

    for(r=1; r<9; r++){
        if(v&1) return r;
        v>>=1;
    }
    return 0;
}





#undef xStride
#undef stride









static av_cold void decode_init_vlc(void){
    static int done = 0;

    if (!done) {
        int i;
        done = 1;

        init_vlc(&chroma_dc_coeff_token_vlc, CHROMA_DC_COEFF_TOKEN_VLC_BITS, 4*5,
                 &chroma_dc_coeff_token_len [0], 1, 1,
                 &chroma_dc_coeff_token_bits[0], 1, 1, 1);

        for(i=0; i<4; i++){
            init_vlc(&coeff_token_vlc[i], COEFF_TOKEN_VLC_BITS, 4*17,
                     &coeff_token_len [i][0], 1, 1,
                     &coeff_token_bits[i][0], 1, 1, 1);
        }

        for(i=0; i<3; i++){
            init_vlc(&chroma_dc_total_zeros_vlc[i], CHROMA_DC_TOTAL_ZEROS_VLC_BITS, 4,
                     &chroma_dc_total_zeros_len [i][0], 1, 1,
                     &chroma_dc_total_zeros_bits[i][0], 1, 1, 1);
        }
        for(i=0; i<15; i++){
            init_vlc(&total_zeros_vlc[i], TOTAL_ZEROS_VLC_BITS, 16,
                     &total_zeros_len [i][0], 1, 1,
                     &total_zeros_bits[i][0], 1, 1, 1);
        }

        for(i=0; i<6; i++){
            init_vlc(&run_vlc[i], RUN_VLC_BITS, 7,
                     &run_len [i][0], 1, 1,
                     &run_bits[i][0], 1, 1, 1);
        }
        init_vlc(&run7_vlc, RUN7_VLC_BITS, 16,
                 &run_len [6][0], 1, 1,
                 &run_bits[6][0], 1, 1, 1);
    }
}

static void free_tables(H264Context *h){
    int i;
    H264Context *hx;
    av_freep(&h->intra4x4_pred_mode);
    av_freep(&h->chroma_pred_mode_table);
    av_freep(&h->cbp_table);
    av_freep(&h->mvd_table[0]);
    av_freep(&h->mvd_table[1]);
    av_freep(&h->direct_table);
    av_freep(&h->non_zero_count);
    av_freep(&h->slice_table_base);
    h->slice_table= NULL;

    av_freep(&h->mb2b_xy);
    av_freep(&h->mb2b8_xy);

    for(i = 0; i < MAX_SPS_COUNT; i++)
        av_freep(h->sps_buffers + i);

    for(i = 0; i < MAX_PPS_COUNT; i++)
        av_freep(h->pps_buffers + i);

    for(i = 0; i < h->s.avctx->thread_count; i++) {
        hx = h->thread_context[i];
        if(!hx) continue;
        av_freep(&hx->top_borders[1]);
        av_freep(&hx->top_borders[0]);
        av_freep(&hx->s.obmc_scratchpad);
    }
}





static av_cold void common_init(H264Context *h){
    MpegEncContext * const s = &h->s;

    s->width = s->avctx->width;
    s->height = s->avctx->height;
    s->codec_id= s->avctx->codec->id;

    ff_h264_pred_init(&h->hpc, s->codec_id);

    h->dequant_coeff_pps= -1;
    s->unrestricted_mv=1;
    s->decode=1; //FIXME

    memset(h->pps.scaling_matrix4, 16, 6*16*sizeof(uint8_t));
    memset(h->pps.scaling_matrix8, 16, 2*64*sizeof(uint8_t));
}

static av_cold int decode_init(AVCodecContext *avctx){
    H264Context *h= avctx->priv_data;
    MpegEncContext * const s = &h->s;

    MPV_decode_defaults(s);

    s->avctx = avctx;
    common_init(h);

    s->out_format = FMT_H264;
    s->workaround_bugs= avctx->workaround_bugs;

    // set defaults
//    s->decode_mb= ff_h263_decode_mb;
    s->quarter_sample = 1;
    s->low_delay= 1;
    avctx->pix_fmt= PIX_FMT_YUV420P;

    decode_init_vlc();

    if(avctx->extradata_size > 0 && avctx->extradata &&
       *(char *)avctx->extradata == 1){
        h->is_avc = 1;
        h->got_avcC = 0;
    } else {
        h->is_avc = 0;
    }

    h->thread_context[0] = h;
    return 0;
}




static void print_short_term(H264Context *h);
static void print_long_term(H264Context *h);






static inline int unreference_pic(H264Context *h, Picture *pic, int refmask){
    int i;
    if (pic->reference &= refmask) {
        return 0;
    } else {
        if(pic == h->delayed_output_pic)
            pic->reference=DELAYED_PIC_REF;
        else{
            for(i = 0; h->delayed_pic[i]; i++)
                if(pic == h->delayed_pic[i]){
                    pic->reference=DELAYED_PIC_REF;
                    break;
                }
        }
        return 1;
    }
}


static void idr(H264Context *h){
    int i;

    for(i=0; i<16; i++){
        if (h->long_ref[i] != NULL) {
            unreference_pic(h, h->long_ref[i], 0);
            h->long_ref[i]= NULL;
        }
    }
    h->long_ref_count=0;

    for(i=0; i<h->short_ref_count; i++){
        unreference_pic(h, h->short_ref[i], 0);
        h->short_ref[i]= NULL;
    }
    h->short_ref_count=0;
}

/* forget old pics after a seek */
static void flush_dpb(AVCodecContext *avctx){
    H264Context *h= (H264Context *)avctx->priv_data;
    int i;
    for(i=0; i<16; i++) {
        if(h->delayed_pic[i])
            h->delayed_pic[i]->reference= 0;
        h->delayed_pic[i]= NULL;
    }
    if(h->delayed_output_pic)
        h->delayed_output_pic->reference= 0;
    h->delayed_output_pic= NULL;
    idr(h);
    if(h->s.current_picture_ptr)
        h->s.current_picture_ptr->reference= 0;
    h->s.first_field= 0;
    ff_mpeg_flush(avctx);
}


static void print_short_term(H264Context *h) {
    uint32_t i;
    if(h->s.avctx->debug&FF_DEBUG_MMCO) {
        av_log(h->s.avctx, AV_LOG_DEBUG, "short term list:\n");
        for(i=0; i<h->short_ref_count; i++){
            Picture *pic= h->short_ref[i];
          
        }
    }
}


static void print_long_term(H264Context *h) {
    uint32_t i;
    if(h->s.avctx->debug&FF_DEBUG_MMCO) {
        av_log(h->s.avctx, AV_LOG_DEBUG, "long term list:\n");
        for(i = 0; i < 16; i++){
            Picture *pic= h->long_ref[i];
            if (pic) {
                av_log(h->s.avctx, AV_LOG_DEBUG, "%d fn:%d poc:%d %p\n", i, pic->frame_num, pic->poc, pic->data[0]);
            }
        }
    }
}




static inline void decode_hrd_parameters(H264Context *h, SPS *sps){
    MpegEncContext * const s = &h->s;
    int cpb_count, i;
    cpb_count = get_ue_golomb(&s->gb) + 1;
    get_bits(&s->gb, 4); /* bit_rate_scale */
    get_bits(&s->gb, 4); /* cpb_size_scale */
    for(i=0; i<cpb_count; i++){
        get_ue_golomb(&s->gb); /* bit_rate_value_minus1 */
        get_ue_golomb(&s->gb); /* cpb_size_value_minus1 */
        get_bits1(&s->gb);     /* cbr_flag */
    }
    get_bits(&s->gb, 5); /* initial_cpb_removal_delay_length_minus1 */
    get_bits(&s->gb, 5); /* cpb_removal_delay_length_minus1 */
    get_bits(&s->gb, 5); /* dpb_output_delay_length_minus1 */
    get_bits(&s->gb, 5); /* time_offset_length */
}

static inline int decode_vui_parameters(H264Context *h, SPS *sps){
    MpegEncContext * const s = &h->s;
    int aspect_ratio_info_present_flag;
    unsigned int aspect_ratio_idc;
    int nal_hrd_parameters_present_flag, vcl_hrd_parameters_present_flag;
	float framerate;
    aspect_ratio_info_present_flag= get_bits1(&s->gb);

    if( aspect_ratio_info_present_flag ) {
        aspect_ratio_idc= get_bits(&s->gb, 8);
        if( aspect_ratio_idc == EXTENDED_SAR ) {
            sps->sar.num= get_bits(&s->gb, 16);
            sps->sar.den= get_bits(&s->gb, 16);
        }else if(aspect_ratio_idc < sizeof(pixel_aspect)/sizeof(*pixel_aspect)){
            sps->sar=  pixel_aspect[aspect_ratio_idc];
        }else{
            av_log(h->s.avctx, AV_LOG_ERROR, "illegal aspect ratio\n");
            return -1;
        }
    }else{
        sps->sar.num=
        sps->sar.den= 0;
    }
//            s->avctx->aspect_ratio= sar_width*s->width / (float)(s->height*sar_height);

    if(get_bits1(&s->gb)){      /* overscan_info_present_flag */
        get_bits1(&s->gb);      /* overscan_appropriate_flag */
    }

    if(get_bits1(&s->gb)){      /* video_signal_type_present_flag */
        get_bits(&s->gb, 3);    /* video_format */
        get_bits1(&s->gb);      /* video_full_range_flag */
        if(get_bits1(&s->gb)){  /* colour_description_present_flag */
            get_bits(&s->gb, 8); /* colour_primaries */
            get_bits(&s->gb, 8); /* transfer_characteristics */
            get_bits(&s->gb, 8); /* matrix_coefficients */
        }
    }

    if(get_bits1(&s->gb)){      /* chroma_location_info_present_flag */
        get_ue_golomb(&s->gb);  /* chroma_sample_location_type_top_field */
        get_ue_golomb(&s->gb);  /* chroma_sample_location_type_bottom_field */
    }

    sps->timing_info_present_flag = get_bits1(&s->gb);
    if(sps->timing_info_present_flag){
        sps->num_units_in_tick = get_bits_long(&s->gb, 32);
        sps->time_scale = get_bits_long(&s->gb, 32);
        sps->fixed_frame_rate_flag = get_bits1(&s->gb);
		 framerate=(float)sps->time_scale/(float)sps->num_units_in_tick/2;//20141114ÁõÃÍ
		//printf("sps->num_units_in_tick=%d\n",sps->num_units_in_tick);
		//printf("sps->time_scale=%d\n",sps->time_scale);
		//printf("framerate=%.2f\n",framerate);
	
    }

    nal_hrd_parameters_present_flag = get_bits1(&s->gb);
    if(nal_hrd_parameters_present_flag)
        decode_hrd_parameters(h, sps);
    vcl_hrd_parameters_present_flag = get_bits1(&s->gb);
    if(vcl_hrd_parameters_present_flag)
        decode_hrd_parameters(h, sps);
    if(nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag)
        get_bits1(&s->gb);     /* low_delay_hrd_flag */
    get_bits1(&s->gb);         /* pic_struct_present_flag */

    sps->bitstream_restriction_flag = get_bits1(&s->gb);
    if(sps->bitstream_restriction_flag){
        unsigned int num_reorder_frames;
        get_bits1(&s->gb);     /* motion_vectors_over_pic_boundaries_flag */
        get_ue_golomb(&s->gb); /* max_bytes_per_pic_denom */
        get_ue_golomb(&s->gb); /* max_bits_per_mb_denom */
        get_ue_golomb(&s->gb); /* log2_max_mv_length_horizontal */
        get_ue_golomb(&s->gb); /* log2_max_mv_length_vertical */
        num_reorder_frames= get_ue_golomb(&s->gb);
        get_ue_golomb(&s->gb); /*max_dec_frame_buffering*/

        if(num_reorder_frames > 16 /*max_dec_frame_buffering || max_dec_frame_buffering > 16*/){
            av_log(h->s.avctx, AV_LOG_ERROR, "illegal num_reorder_frames %d\n", num_reorder_frames);
            return -1;
        }

        sps->num_reorder_frames= num_reorder_frames;
    }

    return 0;
}

static void decode_scaling_list(H264Context *h, uint8_t *factors, int size,
                                const uint8_t *jvt_list, const uint8_t *fallback_list){
    MpegEncContext * const s = &h->s;
    int i, last = 8, next = 8;
    const uint8_t *scan = size == 16 ? zigzag_scan : zigzag_scan8x8;
    if(!get_bits1(&s->gb)) /* matrix not written, we use the predicted one */
        memcpy(factors, fallback_list, size*sizeof(uint8_t));
    else
    for(i=0;i<size;i++){
        if(next)
            next = (last + get_se_golomb(&s->gb)) & 0xff;
        if(!i && !next){ /* matrix not written, we use the preset one */
            memcpy(factors, jvt_list, size*sizeof(uint8_t));
            break;
        }
        last = factors[scan[i]] = next ? next : last;
    }
}

static void decode_scaling_matrices(H264Context *h, SPS *sps, PPS *pps, int is_sps,
                                   uint8_t (*scaling_matrix4)[16], uint8_t (*scaling_matrix8)[64]){
    MpegEncContext * const s = &h->s;
    int fallback_sps = !is_sps && sps->scaling_matrix_present;
    const uint8_t *fallback[4] = {
        fallback_sps ? sps->scaling_matrix4[0] : default_scaling4[0],
        fallback_sps ? sps->scaling_matrix4[3] : default_scaling4[1],
        fallback_sps ? sps->scaling_matrix8[0] : default_scaling8[0],
        fallback_sps ? sps->scaling_matrix8[1] : default_scaling8[1]
    };
    if(get_bits1(&s->gb)){
        sps->scaling_matrix_present |= is_sps;
        decode_scaling_list(h,scaling_matrix4[0],16,default_scaling4[0],fallback[0]); // Intra, Y
        decode_scaling_list(h,scaling_matrix4[1],16,default_scaling4[0],scaling_matrix4[0]); // Intra, Cr
        decode_scaling_list(h,scaling_matrix4[2],16,default_scaling4[0],scaling_matrix4[1]); // Intra, Cb
        decode_scaling_list(h,scaling_matrix4[3],16,default_scaling4[1],fallback[1]); // Inter, Y
        decode_scaling_list(h,scaling_matrix4[4],16,default_scaling4[1],scaling_matrix4[3]); // Inter, Cr
        decode_scaling_list(h,scaling_matrix4[5],16,default_scaling4[1],scaling_matrix4[4]); // Inter, Cb
        if(is_sps || pps->transform_8x8_mode){
            decode_scaling_list(h,scaling_matrix8[0],64,default_scaling8[0],fallback[2]);  // Intra, Y
            decode_scaling_list(h,scaling_matrix8[1],64,default_scaling8[1],fallback[3]);  // Inter, Y
        }
    } else if(fallback_sps) {
        memcpy(scaling_matrix4, sps->scaling_matrix4, 6*16*sizeof(uint8_t));
        memcpy(scaling_matrix8, sps->scaling_matrix8, 2*64*sizeof(uint8_t));
    }
}

/**
 * Returns and optionally allocates SPS / PPS structures in the supplied array 'vec'
 */
static void *
alloc_parameter_set(H264Context *h, void **vec, const unsigned int id, const unsigned int max,
                    const size_t size, const char *name)
{
    if(id>=max) {
        av_log(h->s.avctx, AV_LOG_ERROR, "%s_id (%d) out of range\n", name, id);
        return NULL;
    }

    if(!vec[id]) {
        vec[id] = av_mallocz(size);
        if(vec[id] == NULL)
            av_log(h->s.avctx, AV_LOG_ERROR, "cannot allocate memory for %s\n", name);
    }
    return vec[id];
}

static inline int decode_seq_parameter_set(H264Context *h, int* width, int* height, float* frameRate){
    MpegEncContext * const s = &h->s;
    int profile_idc, level_idc;
    unsigned int sps_id, tmp, mb_width, mb_height;//20141114ÁõÃÍ  Ôö¼Ówidth£¬height
    int i;
    SPS *sps;
	*width = 0;
	*height = 0;
	*frameRate = 25.0;

    profile_idc= get_bits(&s->gb, 8);
    get_bits1(&s->gb);   //constraint_set0_flag
    get_bits1(&s->gb);   //constraint_set1_flag
    get_bits1(&s->gb);   //constraint_set2_flag
    get_bits1(&s->gb);   //constraint_set3_flag
    get_bits(&s->gb, 4); // reserved
    level_idc= get_bits(&s->gb, 8);
    sps_id= get_ue_golomb(&s->gb);

    sps = alloc_parameter_set(h, (void **)h->sps_buffers, sps_id, MAX_SPS_COUNT, sizeof(SPS), "sps");
    if(sps == NULL)
        return -1;

    sps->profile_idc= profile_idc;
    sps->level_idc= level_idc;

    if(sps->profile_idc >= 100){ //high profile
		
        if(get_ue_golomb(&s->gb) == 3) //chroma_format_idc
            get_bits1(&s->gb);  //residual_color_transform_flag
        get_ue_golomb(&s->gb);  //bit_depth_luma_minus8
        get_ue_golomb(&s->gb);  //bit_depth_chroma_minus8
        sps->transform_bypass = get_bits1(&s->gb);
        decode_scaling_matrices(h, sps, NULL, 1, sps->scaling_matrix4, sps->scaling_matrix8);
    }else
        sps->scaling_matrix_present = 0;

    sps->log2_max_frame_num= get_ue_golomb(&s->gb);
	
    sps->poc_type= get_ue_golomb(&s->gb);
    if(sps->poc_type == 0){ //FIXME #define
        sps->log2_max_poc_lsb= get_ue_golomb(&s->gb);
    } else if(sps->poc_type == 1){//FIXME #define
        sps->delta_pic_order_always_zero_flag= get_bits1(&s->gb);
        sps->offset_for_non_ref_pic= get_se_golomb(&s->gb);
        sps->offset_for_top_to_bottom_field= get_se_golomb(&s->gb);
        tmp= get_ue_golomb(&s->gb);

        if(tmp >= sizeof(sps->offset_for_ref_frame) / sizeof(sps->offset_for_ref_frame[0])){
            av_log(h->s.avctx, AV_LOG_ERROR, "poc_cycle_length overflow %u\n", tmp);
            return -1;
        }
        sps->poc_cycle_length= tmp;

        for(i=0; i<sps->poc_cycle_length; i++)
            sps->offset_for_ref_frame[i]= get_se_golomb(&s->gb);
    }else if(sps->poc_type != 2){
        av_log(h->s.avctx, AV_LOG_ERROR, "illegal POC type %d\n", sps->poc_type);
        return -1;
    }

    tmp= get_ue_golomb(&s->gb);
    if(tmp > MAX_PICTURE_COUNT-2 || tmp >= 32){
        av_log(h->s.avctx, AV_LOG_ERROR, "too many reference frames\n");
        return -1;
    }
    sps->ref_frame_count= tmp;
    sps->gaps_in_frame_num_allowed_flag= get_bits1(&s->gb);
    mb_width= get_ue_golomb(&s->gb) + 1;
    mb_height= get_ue_golomb(&s->gb) + 1;
   if(mb_width >= INT_MAX/16 || mb_height >= INT_MAX/16 ||
       avcodec_check_dimensions(NULL, 16*mb_width, 16*mb_height)){
        av_log(h->s.avctx, AV_LOG_ERROR, "mb_width/height overflow\n");
        return -1;
    }
    sps->mb_width = mb_width;
    sps->mb_height= mb_height;

    sps->frame_mbs_only_flag= get_bits1(&s->gb);
    if(!sps->frame_mbs_only_flag)
        sps->mb_aff= get_bits1(&s->gb);
    else
        sps->mb_aff= 0;

    sps->direct_8x8_inference_flag= get_bits1(&s->gb);


    if(!sps->direct_8x8_inference_flag && sps->mb_aff)
        av_log(h->s.avctx, AV_LOG_ERROR, "MBAFF + !direct_8x8_inference is not implemented\n");

    sps->crop= get_bits1(&s->gb);
    if(sps->crop){
        sps->crop_left  = get_ue_golomb(&s->gb);
        sps->crop_right = get_ue_golomb(&s->gb);
        sps->crop_top   = get_ue_golomb(&s->gb);
        sps->crop_bottom= get_ue_golomb(&s->gb);
        if(sps->crop_left || sps->crop_top){
            av_log(h->s.avctx, AV_LOG_ERROR, "insane cropping not completely supported, this could look slightly wrong ...\n");
        }
        if(sps->crop_right >= 8 || sps->crop_bottom >= (8>> !h->sps.frame_mbs_only_flag)){
            av_log(h->s.avctx, AV_LOG_ERROR, "brainfart cropping not supported, this could look slightly wrong ...\n");
        }
    }else{
        sps->crop_left  =
        sps->crop_right =
        sps->crop_top   =
        sps->crop_bottom= 0;
    }
	*width=mb_width*16-2*sps->crop_left-2*sps->crop_right;//20141114ÁõÃÍ
	*height=mb_height*16-2*sps->crop_top-2*sps->crop_bottom;
// 	printf("width=%d\n",width);
// 	printf("height=%d\n",height);

    sps->vui_parameters_present_flag= get_bits1(&s->gb);
// 	printf("VUI=%d\n",sps->vui_parameters_present_flag);//20141114ÁõÃÍ
    if( (sps->vui_parameters_present_flag) && (sps->num_units_in_tick != 0) && (sps->time_scale != 0) )
	{
        decode_vui_parameters(h, sps);
		*frameRate=(float)sps->time_scale/(float)sps->num_units_in_tick/2;
	}

   if(s->avctx->debug&FF_DEBUG_PICT_INFO){
        av_log(h->s.avctx, AV_LOG_DEBUG, "sps:%u profile:%d/%d poc:%d ref:%d %dx%d %s %s crop:%d/%d/%d/%d %s\n",
               sps_id, sps->profile_idc, sps->level_idc,
               sps->poc_type,
               sps->ref_frame_count,
               sps->mb_width, sps->mb_height,
               sps->frame_mbs_only_flag ? "FRM" : (sps->mb_aff ? "MB-AFF" : "PIC-AFF"),
               sps->direct_8x8_inference_flag ? "8B8" : "",
               sps->crop_left, sps->crop_right,
               sps->crop_top, sps->crop_bottom,
               sps->vui_parameters_present_flag ? "VUI" : ""
               );
    }
    return 0;
}

static void
build_qp_table(PPS *pps, int t, int index)
{
    int i;
    for(i = 0; i < 255; i++)
        pps->chroma_qp_table[t][i & 0xff] = chroma_qp[av_clip(i + index, 0, 51)];
}







static int decode_nal_units(H264Context *h, const uint8_t *buf, int buf_size, int* width, int* height, float* frameRate){
    MpegEncContext * const s = &h->s;
    AVCodecContext * const avctx= s->avctx;
    int buf_index=0;
    H264Context *hx; ///< thread context
    int context_count = 0;

    h->max_contexts = avctx->thread_count;



    for(;;){
        int consumed;
        int dst_length;
        int bit_length;
        const uint8_t *ptr;
        int i, nalsize = 0;
        int err;

        if(h->is_avc) {
            if(buf_index >= buf_size) break;
            nalsize = 0;
            for(i = 0; i < h->nal_length_size; i++)
                nalsize = (nalsize << 8) | buf[buf_index++];
            if(nalsize <= 1 || (nalsize+buf_index > buf_size)){
                if(nalsize == 1){
                    buf_index++;
                    continue;
                }else{
                  
                    break;
                }
            }
        } else {
            // start code prefix search
            for(; buf_index + 3 < buf_size; buf_index++){
                // This should always succeed in the first iteration.
                if(buf[buf_index] == 0 && buf[buf_index+1] == 0 && buf[buf_index+2] == 1)
                    break;
            }

            if(buf_index+3 >= buf_size) break;

            buf_index+=3;
        }

        hx = h->thread_context[context_count];

        ptr= decode_nal(hx, buf + buf_index, &dst_length, &consumed, h->is_avc ? nalsize : buf_size - buf_index);
        if (ptr==NULL || dst_length < 0){
            return -1;
        }
        while(ptr[dst_length - 1] == 0 && dst_length > 0)
            dst_length--;
        bit_length= !dst_length ? 0 : (8*dst_length - decode_rbsp_trailing(h, ptr + dst_length - 1));

        if(s->avctx->debug&FF_DEBUG_STARTCODE){
           
        }

        if (h->is_avc && (nalsize != consumed)){
            
            consumed= nalsize;
        }

        buf_index += consumed;

        if(  (s->hurry_up == 1 && h->nal_ref_idc  == 0) //FIXME do not discard SEI id
           ||(avctx->skip_frame >= AVDISCARD_NONREF && h->nal_ref_idc  == 0))
            continue;

   
        err = 0;
       
        if(hx->nal_unit_type==NAL_SPS){
       
            init_get_bits(&s->gb, ptr, bit_length);
            decode_seq_parameter_set(h, width, height, frameRate);

            if(s->flags& CODEC_FLAG_LOW_DELAY)
                s->low_delay=1;

            if(avctx->has_b_frames < 2)
                avctx->has_b_frames= !s->low_delay;
            break;
       
       
        }

       
     
    }
  
    return buf_index;
}

/**
 * returns the number of bytes consumed for building the current frame
 */


 int  decode_frame(AVCodecContext *avctx,
                             void *data, int *data_size,
                             const uint8_t *buf, int buf_size,
							 int* width, int* height, float* frameRate)
{
    H264Context *h = (H264Context *)avctx->priv_data;
    MpegEncContext *s = &h->s;
    AVFrame *pict =(AVFrame *) data;
    int buf_index;

    s->flags= avctx->flags;
    s->flags2= avctx->flags2;



    if(h->is_avc && !h->got_avcC) {
        int i, cnt, nalsize;
        unsigned char *p = avctx->extradata;
        if(avctx->extradata_size < 7) {
            
            return -1;
        }
        if(*p != 1) {
          
            return -1;
        }
        /* sps and pps in the avcC always have length coded with 2 bytes,
           so put a fake nal_length_size = 2 while parsing them */
        h->nal_length_size = 2;
        // Decode sps from avcC
        cnt = *(p+5) & 0x1f; // Number of sps
        p += 6;
        for (i = 0; i < cnt; i++) {
            nalsize = AV_RB16(p) + 2;
            if(decode_nal_units(h, p, nalsize, width, height, frameRate) < 0) {
          
                return -1;
            }
            p += nalsize;
        }

    }

 
	
    buf_index=decode_nal_units(h, buf, buf_size, width, height, frameRate);
    if(buf_index < 0)
        return -1;

	return 0;
 }


static av_cold int decode_end(AVCodecContext *avctx)
{
    H264Context *h = avctx->priv_data;
    MpegEncContext *s = &h->s;

    av_freep(&h->rbsp_buffer[0]);
    av_freep(&h->rbsp_buffer[1]);
    free_tables(h); //FIXME cleanup init stuff perhaps
    MPV_common_end(s);

//    memset(h, 0, sizeof(H264Context));

    return 0;
}


AVCodec h264_decoder = {
    "h264",
    CODEC_TYPE_VIDEO,
    CODEC_ID_H264,
    sizeof(H264Context),
    decode_init,
    NULL,
    decode_end,
    decode_frame,
    /*CODEC_CAP_DRAW_HORIZ_BAND |*/ CODEC_CAP_DR1 | CODEC_CAP_TRUNCATED | CODEC_CAP_DELAY,
     flush_dpb,
     NULL_IF_CONFIG_SMALL("H.264 / AVC / MPEG-4 AVC / MPEG-4 part 10"),
};

#include "svq3.c"
