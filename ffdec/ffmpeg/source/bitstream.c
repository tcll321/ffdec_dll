#include "avcodec.h"
#include "bitstream.h"
#include "define.h"
static void *ff_realloc_static(void *ptr, unsigned int size)
{
    return av_realloc(ptr, size);
}





#define GET_DATA(v, table, i, wrap, size) \
{\
    const uint8_t *ptr = (const uint8_t *)table + i * wrap;\
    switch(size) {\
    case 1:\
        v = *(const uint8_t *)ptr;\
        break;\
    case 2:\
        v = *(const uint16_t *)ptr;\
        break;\
    default:\
        v = *(const uint32_t *)ptr;\
        break;\
    }\
}


static int alloc_table(VLC *vlc, int size, int use_static)
{
    int index;
    index = vlc->table_size;
    vlc->table_size += size;
    if (vlc->table_size > vlc->table_allocated) {
        if(use_static>1)
            abort(); //cant do anything, init_vlc() is used with too little memory
        vlc->table_allocated += (1 << vlc->bits);
        if(use_static)
            vlc->table = ff_realloc_static(vlc->table,
                                           sizeof(VLC_TYPE) * 2 * vlc->table_allocated);
        else
            vlc->table = av_realloc(vlc->table,
                                    sizeof(VLC_TYPE) * 2 * vlc->table_allocated);
        if (!vlc->table)
            return -1;
    }
    return index;
}

static int build_table(VLC *vlc, int table_nb_bits,
                       int nb_codes,
                       const void *bits, int bits_wrap, int bits_size,
                       const void *codes, int codes_wrap, int codes_size,
                       const void *symbols, int symbols_wrap, int symbols_size,
                       uint32_t code_prefix, int n_prefix, int flags)
{
    int i, j, k, n, table_size, table_index, nb, n1, index, code_prefix2, symbol;
    uint32_t code;
    VLC_TYPE (*table)[2];

    table_size = 1 << table_nb_bits;
    table_index = alloc_table(vlc, table_size, flags & (INIT_VLC_USE_STATIC|INIT_VLC_USE_NEW_STATIC));
#ifdef DEBUG_VLC
    av_log(NULL,AV_LOG_DEBUG,"new table index=%d size=%d code_prefix=%x n=%d\n",
           table_index, table_size, code_prefix, n_prefix);
#endif
    if (table_index < 0)
        return -1;
    table = &vlc->table[table_index];

    for(i=0;i<table_size;i++) {
        table[i][1] = 0; //bits
        table[i][0] = -1; //codes
    }

    /* first pass: map codes and compute auxillary table sizes */
    for(i=0;i<nb_codes;i++) {
        GET_DATA(n, bits, i, bits_wrap, bits_size);
        GET_DATA(code, codes, i, codes_wrap, codes_size);
        /* we accept tables with holes */
        if (n <= 0)
            continue;
        if (!symbols)
            symbol = i;
        else
            GET_DATA(symbol, symbols, i, symbols_wrap, symbols_size);
#if defined(DEBUG_VLC) && 0
        av_log(NULL,AV_LOG_DEBUG,"i=%d n=%d code=0x%x\n", i, n, code);
#endif
        /* if code matches the prefix, it is in the table */
        n -= n_prefix;
        if(flags & INIT_VLC_LE)
            code_prefix2= code & (n_prefix>=32 ? 0xffffffff : (1 << n_prefix)-1);
        else
            code_prefix2= code >> n;
        if (n > 0 && code_prefix2 == code_prefix) {
            if (n <= table_nb_bits) {
                /* no need to add another table */
                j = (code << (table_nb_bits - n)) & (table_size - 1);
                nb = 1 << (table_nb_bits - n);
                for(k=0;k<nb;k++) {
                    if(flags & INIT_VLC_LE)
                        j = (code >> n_prefix) + (k<<n);
#ifdef DEBUG_VLC
                    av_log(NULL, AV_LOG_DEBUG, "%4x: code=%d n=%d\n",
                           j, i, n);
#endif
                    if (table[j][1] /*bits*/ != 0) {
                        av_log(NULL, AV_LOG_ERROR, "incorrect codes\n");
                        return -1;
                    }
                    table[j][1] = n; //bits
                    table[j][0] = symbol;
                    j++;
                }
            } else {
                n -= table_nb_bits;
                j = (code >> ((flags & INIT_VLC_LE) ? n_prefix : n)) & ((1 << table_nb_bits) - 1);
#ifdef DEBUG_VLC
                av_log(NULL,AV_LOG_DEBUG,"%4x: n=%d (subtable)\n",
                       j, n);
#endif
                /* compute table size */
                n1 = -table[j][1]; //bits
                if (n > n1)
                    n1 = n;
                table[j][1] = -n1; //bits
            }
        }
    }

    /* second pass : fill auxillary tables recursively */
    for(i=0;i<table_size;i++) {
        n = table[i][1]; //bits
        if (n < 0) {
            n = -n;
            if (n > table_nb_bits) {
                n = table_nb_bits;
                table[i][1] = -n; //bits
            }
            index = build_table(vlc, n, nb_codes,
                                bits, bits_wrap, bits_size,
                                codes, codes_wrap, codes_size,
                                symbols, symbols_wrap, symbols_size,
                                (flags & INIT_VLC_LE) ? (code_prefix | (i << n_prefix)) : ((code_prefix << table_nb_bits) | i),
                                n_prefix + table_nb_bits, flags);
            if (index < 0)
                return -1;
            /* note: realloc has been done, so reload tables */
            table = &vlc->table[table_index];
            table[i][0] = index; //code
        }
    }
    return table_index;
}


int init_vlc_sparse(VLC *vlc, int nb_bits, int nb_codes,
             const void *bits, int bits_wrap, int bits_size,
             const void *codes, int codes_wrap, int codes_size,
             const void *symbols, int symbols_wrap, int symbols_size,
             int flags)
{
    vlc->bits = nb_bits;
    if(flags & INIT_VLC_USE_NEW_STATIC){
        if(vlc->table_size && vlc->table_size == vlc->table_allocated){
            return 0;
        }else if(vlc->table_size){
            abort(); // fatal error, we are called on a partially initialized table
        }
    }else if(!(flags & INIT_VLC_USE_STATIC)) {
        vlc->table = NULL;
        vlc->table_allocated = 0;
        vlc->table_size = 0;
    } else {
        /* Static tables are initially always NULL, return
           if vlc->table != NULL to avoid double allocation */
        if(vlc->table)
            return 0;
    }

#ifdef DEBUG_VLC
    av_log(NULL,AV_LOG_DEBUG,"build table nb_codes=%d\n", nb_codes);
#endif

    if (build_table(vlc, nb_bits, nb_codes,
                    bits, bits_wrap, bits_size,
                    codes, codes_wrap, codes_size,
                    symbols, symbols_wrap, symbols_size,
                    0, 0, flags) < 0) {
        av_freep(&vlc->table);
        return -1;
    }
    if((flags & INIT_VLC_USE_NEW_STATIC) && vlc->table_size != vlc->table_allocated)
        av_log(NULL, AV_LOG_ERROR, "needed %d had %d\n", vlc->table_size, vlc->table_allocated);
    return 0;
}




