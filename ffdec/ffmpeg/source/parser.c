#include "parser.h"
#include "define.h"
#include "avcodec.h"
#include <stdio.h>

AVCodecParser *av_first_parser = NULL;

AVCodecParser* av_parser_next(AVCodecParser *p){
    if(p) return p->next;
    else  return av_first_parser;
}

void av_register_codec_parser(AVCodecParser *parser)
{
    parser->next = av_first_parser;
    av_first_parser = parser;
}

AVCodecParserContext *av_parser_init(int codec_id)
{
    AVCodecParserContext *s;
    AVCodecParser *parser;
    int ret;

    if(codec_id == CODEC_ID_NONE)
        return NULL;

    for(parser = av_first_parser; parser != NULL; parser = parser->next) {
        if (parser->codec_ids[0] == codec_id ||
            parser->codec_ids[1] == codec_id ||
            parser->codec_ids[2] == codec_id ||
            parser->codec_ids[3] == codec_id ||
            parser->codec_ids[4] == codec_id)
            goto found;
    }
    return NULL;
 found:
    s = av_mallocz(sizeof(AVCodecParserContext));
    if (!s)
        return NULL;
    s->parser = parser;
    s->priv_data = av_mallocz(parser->priv_data_size);
    if (!s->priv_data) {
        av_free(s);
        return NULL;
    }
    if (parser->parser_init) {
        ret = parser->parser_init(s);
        if (ret != 0) {
            av_free(s->priv_data);
            av_free(s);
            return NULL;
        }
    }
    s->fetch_timestamp=1;
    s->pict_type = FF_I_TYPE;
    return s;
}

void ff_fetch_timestamp(AVCodecParserContext *s, int off, int remove){
    int i;

    s->dts= s->pts= AV_NOPTS_VALUE;
    s->offset= 0;
    for(i = 0; i < AV_PARSER_PTS_NB; i++) {
        if (   s->next_frame_offset + off >= s->cur_frame_offset[i]
            &&(s->     frame_offset       <  s->cur_frame_offset[i] || !s->frame_offset)
            //check is disabled  becausue mpeg-ts doesnt send complete PES packets
            && /*s->next_frame_offset + off <*/  s->cur_frame_end[i]){
            s->dts= s->cur_frame_dts[i];
            s->pts= s->cur_frame_pts[i];
            s->offset = s->next_frame_offset - s->cur_frame_offset[i];
            if(remove)
                s->cur_frame_offset[i]= INT64_MAX;
        }
    }
}



int av_parser_change(AVCodecParserContext *s,
                     AVCodecContext *avctx,
                     uint8_t **poutbuf, int *poutbuf_size,
                     const uint8_t *buf, int buf_size, int keyframe){

    if(s && s->parser->split){
        if((avctx->flags & CODEC_FLAG_GLOBAL_HEADER) || (avctx->flags2 & CODEC_FLAG2_LOCAL_HEADER)){
            int i= s->parser->split(avctx, buf, buf_size);
            buf += i;
            buf_size -= i;
        }
    }

    /* cast to avoid warning about discarding qualifiers */
    *poutbuf= (uint8_t *) buf;
    *poutbuf_size= buf_size;
    if(avctx->extradata){
        if(  (keyframe && (avctx->flags2 & CODEC_FLAG2_LOCAL_HEADER))
            /*||(s->pict_type != FF_I_TYPE && (s->flags & PARSER_FLAG_DUMP_EXTRADATA_AT_NOKEY))*/
            /*||(? && (s->flags & PARSER_FLAG_DUMP_EXTRADATA_AT_BEGIN)*/){
            int size= buf_size + avctx->extradata_size;
            *poutbuf_size= size;
            *poutbuf= av_malloc(size + FF_INPUT_BUFFER_PADDING_SIZE);

            memcpy(*poutbuf, avctx->extradata, avctx->extradata_size);
            memcpy((*poutbuf) + avctx->extradata_size, buf, buf_size + FF_INPUT_BUFFER_PADDING_SIZE);
            return 1;
        }
    }

    return 0;
}



int ff_combine_frame(ParseContext *pc, int next, const uint8_t **buf, int *buf_size)
{
#if 0
    if(pc->overread){
//         printf("overread %d, state:%X next:%d index:%d o_index:%d\n", pc->overread, pc->state, next, pc->index, pc->overread_index);
//         printf("%X %X %X %X\n", (*buf)[0], (*buf)[1],(*buf)[2],(*buf)[3]);
    }
#endif

    /* Copy overread bytes from last frame into buffer. */
    for(; pc->overread>0; pc->overread--){
        pc->buffer[pc->index++]= pc->buffer[pc->overread_index++];
    }

    /* flush remaining if EOF */
    if(!*buf_size && next == END_NOT_FOUND){
        next= 0;
    }

    pc->last_index= pc->index;

    /* copy into buffer end return */
    if(next == END_NOT_FOUND){
        void* new_buffer = av_fast_realloc(pc->buffer, &pc->buffer_size, (*buf_size) + pc->index + FF_INPUT_BUFFER_PADDING_SIZE);

        if(!new_buffer)
            return AVERROR(ENOMEM);
        pc->buffer = new_buffer;
        memcpy(&pc->buffer[pc->index], *buf, *buf_size);
        pc->index += *buf_size;
        return -1;
    }

    *buf_size=
    pc->overread_index= pc->index + next;

    /* append to buffer */
    if(pc->index){
        void* new_buffer = av_fast_realloc(pc->buffer, &pc->buffer_size, next + pc->index + FF_INPUT_BUFFER_PADDING_SIZE);

        if(!new_buffer)
            return AVERROR(ENOMEM);
        pc->buffer = new_buffer;
        memcpy(&pc->buffer[pc->index], *buf, next + FF_INPUT_BUFFER_PADDING_SIZE );
        pc->index = 0;
        *buf= pc->buffer;
    }

    /* store overread bytes */
    for(;next < 0; next++){
        pc->state = (pc->state<<8) | pc->buffer[pc->last_index + next];
        pc->overread++;
    }

#if 0
    if(pc->overread){
//         printf("overread %d, state:%X next:%d index:%d o_index:%d\n", pc->overread, pc->state, next, pc->index, pc->overread_index);
//         printf("%X %X %X %X\n", (*buf)[0], (*buf)[1],(*buf)[2],(*buf)[3]);
    }
#endif

    return 0;
}

void ff_parse_close(AVCodecParserContext *s)
{
    ParseContext *pc = s->priv_data;

    av_free(pc->buffer);
}

