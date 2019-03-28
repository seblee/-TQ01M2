/*
 * Copyright (c) 2014-2016 Alibaba Group. All rights reserved.
 * License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "utils_md5.h"
#include "utils_hmac.h"
#include <rtthread.h>

// #define DBG_ENABLE
#define DBG_SECTION_NAME "hmac"
#define DBG_LEVEL DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>
#ifndef LOG_D
#error "Please update the 'rtdbg.h' file to GitHub latest version (https://github.com/RT-Thread/rt-thread/blob/master/include/rtdbg.h)"
#endif

#ifndef INCLUDE_HMAC
#include "sys_conf.h"

#define KEY_IOPAD_SIZE 64

#define MD5_DIGEST_SIZE 16
#define SHA1_DIGEST_SIZE 20

void utils_hmac_md5(const char *msg, int msg_len, char *digest, const char *key, int key_len)
{
    if ((RT_NULL == msg) || (RT_NULL == digest) || (RT_NULL == key))
    {
        // log_err("parameter is Null,failed!");
        return;
    }

    if (key_len > KEY_IOPAD_SIZE)
    {
        // log_err("key_len > size(%d) of array", KEY_IOPAD_SIZE);
        return;
    }

    iot_md5_context context;
    unsigned char k_ipad[KEY_IOPAD_SIZE]; /* inner padding - key XORd with ipad  */
    unsigned char k_opad[KEY_IOPAD_SIZE]; /* outer padding - key XORd with opad */
    unsigned char out[MD5_DIGEST_SIZE];
    int i;

    /* start out by storing key in pads */
    memset(k_ipad, 0, sizeof(k_ipad));
    memset(k_opad, 0, sizeof(k_opad));
    memcpy(k_ipad, key, key_len);
    memcpy(k_opad, key, key_len);

    /* XOR key with ipad and opad values */
    for (i = 0; i < KEY_IOPAD_SIZE; i++)
    {
        k_ipad[i] ^= 0x36;
        k_opad[i] ^= 0x5c;
    }

    /* perform inner MD5 */
    utils_md5_init(&context);                                  /* init context for 1st pass */
    utils_md5_starts(&context);                                /* setup context for 1st pass */
    utils_md5_update(&context, k_ipad, KEY_IOPAD_SIZE);        /* start with inner pad */
    utils_md5_update(&context, (unsigned char *)msg, msg_len); /* then text of datagram */
    utils_md5_finish(&context, out);                           /* finish up 1st pass */

    /* perform outer MD5 */
    utils_md5_init(&context);                           /* init context for 2nd pass */
    utils_md5_starts(&context);                         /* setup context for 2nd pass */
    utils_md5_update(&context, k_opad, KEY_IOPAD_SIZE); /* start with outer pad */
    utils_md5_update(&context, out, MD5_DIGEST_SIZE);   /* then results of 1st hash */
    utils_md5_finish(&context, out);                    /* finish up 2nd pass */

    for (i = 0; i < MD5_DIGEST_SIZE; ++i)
    {
        digest[i * 2] = utils_hb2hex(out[i] >> 4);
        digest[i * 2 + 1] = utils_hb2hex(out[i]);
    }
}

#else

uint32_t utils_hmac_md5(const char *msg, int msg_len, char *digest, const char *key, int key_len)
{
    HMAC_MD5ctx_stt HMAC_MD5ctx_st;
    uint32_t error_status = HASH_SUCCESS;

    HMAC_MD5ctx_st.mFlags = E_HASH_DEFAULT;
    HMAC_MD5ctx_st.mTagSize = CRL_MD5_SIZE;
    LOG_D("msg:%s", msg);
    LOG_D("key:%s", key);
    Crypto_DeInit();
    HMAC_MD5ctx_st.pmKey = (const uint8_t *)(key);
    HMAC_MD5ctx_st.mKeySize = key_len;
    error_status = HMAC_MD5_Init(&HMAC_MD5ctx_st);
    /* check for initialization errors */
    if (error_status == HASH_SUCCESS)
    {
        /* Add data to be hashed */
        error_status = HMAC_MD5_Append(&HMAC_MD5ctx_st, (const uint8_t *)msg, msg_len);

        if (error_status == HASH_SUCCESS)
        {
            int32_t P_pOutputSize;
            int8_t OutputBuffer[16];
            rt_memset(digest, 0, 32);
            /* retrieve */
            error_status = HMAC_MD5_Finish(&HMAC_MD5ctx_st, (uint8_t *)OutputBuffer, &P_pOutputSize);
            if (error_status == HASH_SUCCESS)
            {
                // rt_kprintf("\r\nOutputSize:%dmd5:", P_pOutputSize);
                for (int i = 0; i < P_pOutputSize; ++i)
                {
                    digest[i * 2] = utils_hb2hex(OutputBuffer[i] >> 4);
                    digest[i * 2 + 1] = utils_hb2hex(OutputBuffer[i]);
                }

                //  LOG_D("digest:%s", digest);
            }
            else
            {
                LOG_E("HMAC_MD5_Append err:%d", error_status);
            }
        }
        else
        {
            LOG_E("HMAC_MD5_Finish err:%d", error_status);
        }
    }
    else
    {
        LOG_E("HMAC_MD5_Init err:%d", error_status);
    }
    return error_status;
}

#endif

// void utils_hmac_sha1(const char *msg, int msg_len, char *digest, const char *key, int key_len)
// {
//     if ((NULL == msg) || (NULL == digest) || (NULL == key))
//     {
//         log_err("parameter is Null,failed!");
//         return;
//     }

//     if (key_len > KEY_IOPAD_SIZE)
//     {
//         log_err("key_len > size(%d) of array", KEY_IOPAD_SIZE);
//         return;
//     }

//     iot_sha1_context context;
//     unsigned char k_ipad[KEY_IOPAD_SIZE]; /* inner padding - key XORd with ipad  */
//     unsigned char k_opad[KEY_IOPAD_SIZE]; /* outer padding - key XORd with opad */
//     unsigned char out[SHA1_DIGEST_SIZE];
//     int i;

//     /* start out by storing key in pads */
//     memset(k_ipad, 0, sizeof(k_ipad));
//     memset(k_opad, 0, sizeof(k_opad));
//     memcpy(k_ipad, key, key_len);
//     memcpy(k_opad, key, key_len);

//     /* XOR key with ipad and opad values */
//     for (i = 0; i < KEY_IOPAD_SIZE; i++)
//     {
//         k_ipad[i] ^= 0x36;
//         k_opad[i] ^= 0x5c;
//     }

//     /* perform inner SHA */
//     utils_sha1_init(&context);                                  /* init context for 1st pass */
//     utils_sha1_starts(&context);                                /* setup context for 1st pass */
//     utils_sha1_update(&context, k_ipad, KEY_IOPAD_SIZE);        /* start with inner pad */
//     utils_sha1_update(&context, (unsigned char *)msg, msg_len); /* then text of datagram */
//     utils_sha1_finish(&context, out);                           /* finish up 1st pass */

//     /* perform outer SHA */
//     utils_sha1_init(&context);                           /* init context for 2nd pass */
//     utils_sha1_starts(&context);                         /* setup context for 2nd pass */
//     utils_sha1_update(&context, k_opad, KEY_IOPAD_SIZE); /* start with outer pad */
//     utils_sha1_update(&context, out, SHA1_DIGEST_SIZE);  /* then results of 1st hash */
//     utils_sha1_finish(&context, out);                    /* finish up 2nd pass */

//     for (i = 0; i < SHA1_DIGEST_SIZE; ++i)
//     {
//         digest[i * 2] = utils_hb2hex(out[i] >> 4);
//         digest[i * 2 + 1] = utils_hb2hex(out[i]);
//     }
// }
