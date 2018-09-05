/* Copyright (c) 2018, Eric McCorkle.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/endian.h>
#include <string.h>

#include "hc256.h"

/* Most of the contents of this file are taken directly from the paper.*/

#define IV_WORDS (IV_BITS / 32)

#define KEY_WORDS (KEY_BITS / 32)

#define TABLE_MASK (TABLE_SIZE - 1)

#define INIT_SIZE  2660

#define BLOCK_SIZE 16

#define f1(x) (((x >> 7) | (x << 25)) ^  \
               ((x >> 18) | (x << 14)) ^ \
               (x >> 3))

#define f2(x) (((x >> 17) | (x << 15)) ^ \
               ((x >> 19) | (x << 13)) ^ \
               (x >> 10))

#define g1(x, y, ctx) ((((x >> 10) | (x << 22)) ^ \
                        ((y >> 23) | (y << 9))) + \
                       ctx->qtable[(x ^ y) & TABLE_MASK])

#define g2(x, y, ctx) ((((x >> 10) | (x << 22)) ^ \
                        ((y >> 23) | (y << 9))) + \
                       ctx->ptable[(x ^ y) & TABLE_MASK])

#define h1(x, ctx) (ctx->qtable[x & 0xff] +                 \
                    ctx->qtable[256 + ((x >> 8) & 0xff)] +  \
                    ctx->qtable[512 + ((x >> 16) & 0xff)] + \
                    ctx->qtable[768 + ((x >> 24) & 0xff)])

#define h2(x, ctx) (ctx->ptable[x & 0xff] +                 \
                    ctx->ptable[256 + ((x >> 8) & 0xff)] +  \
                    ctx->ptable[512 + ((x >> 16) & 0xff)] + \
                    ctx->ptable[768 + ((x >> 24) & 0xff)])

#define subMod(lhs, rhs) \
  ((((lhs - rhs) & TABLE_MASK) + TABLE_SIZE) % TABLE_SIZE)

/* We stash the key in ptable and the IV in qtable for an
 * uninitialized context.
 */
#define ctx_key(ctx) (ctx->ptable)
#define ctx_iv(ctx) (ctx->qtable)

static void generate_word(HC256_ctx *ctx)
{
        const int i = ctx->idx;
        const int j = ctx->idx & TABLE_MASK;

        ctx->idx = (ctx->idx + 1) & (2048 - 1);

        if (i < 1024) {
                ctx->ptable[j] = ctx->ptable[j] +
                  ctx->ptable[(j - 10) & TABLE_MASK] +
                  g1(ctx->ptable[(j - 3) & TABLE_MASK],
                     ctx->ptable[(j - 1023) & TABLE_MASK],
                     ctx);

                ctx->word = h1(ctx->ptable[(j - 12) & TABLE_MASK], ctx) ^
                  ctx->ptable[j];
        } else {
                ctx->qtable[j] = ctx->qtable[j] +
                  ctx->qtable[(j - 10) & TABLE_MASK] +
                  g2(ctx->qtable[(j - 3) & TABLE_MASK],
                     ctx->qtable[(j - 1023) & TABLE_MASK],
                     ctx);

                ctx->word = h2(ctx->qtable[(j - 12) & TABLE_MASK], ctx) ^
                  ctx->qtable[j];
        }

        ctx->offset = 0;
}

static void init_state(HC256_ctx *ctx)
{
        uint32_t *key = ctx_key(ctx);
        uint32_t *iv = ctx_iv(ctx);
        uint32_t data[INIT_SIZE];

        for(int i = 0; i < KEY_WORDS; i++) {
                data[i] = key[i];
        }

        for(int i = 0; i < IV_WORDS; i++) {
                data[i + KEY_WORDS] =
                  (uint32_t)(iv[(4 * i)]) |
                  (uint32_t)(iv[(4 * i) + 1]) << 8 |
                  (uint32_t)(iv[(4 * i) + 2]) << 16 |
                  (uint32_t)(iv[(4 * i) + 3]) << 24;
        }

        for(int i = IV_WORDS + KEY_WORDS; i < INIT_SIZE; i++) {
                data[i] = f2(data[i - 2]) + data[i - 7] +
                  f1(data[i - 15]) + data[i - 16] + i;
        }

        for(int i = 0; i < TABLE_SIZE; i++) {
                ctx->ptable[i] = data[i + 512];
        }

        for(int i = 0; i < TABLE_SIZE; i++) {
                ctx->qtable[i] = data[i + 1536];
        }

        memset(data, 0, sizeof(data));
        ctx->idx = 0;

        for(int i = 0; i < 4096; i++) {
                generate_word(ctx);
        }

        ctx->offset = 4;
}

void HC256_set_key(HC256_ctx *ctx,
                   const unsigned char *key)
{
        memcpy(ctx_key(ctx), key, KEY_LEN);
}

void HC256_set_iv(HC256_ctx *ctx,
                  const unsigned char *iv)
{
        memcpy(ctx_key(ctx), iv, KEY_LEN);
}

void HC256(HC256_ctx *ctx,
           unsigned char *out,
           const unsigned char *in,
           size_t len)
{
        for(size_t i = 0; i < len;) {
                for(uint8_t j = ctx->offset; j < 4; j++, i++) {
                        out[i] = in[i] ^
                          (uint8_t)((ctx->word >> (8 * j)) & 0xff);
                }

                generate_word(ctx);
        }
}

void CRYPTO_HC256(unsigned char *out,
                  const unsigned char *in,
                  size_t len,
                  const unsigned char key[KEY_LEN],
                  const unsigned char iv[IV_LEN])
{
        HC256_ctx ctx;

        HC256_set_key(&ctx, key);
        HC256_set_iv(&ctx, iv);
        init_state(&ctx);
        HC256(&ctx, out, in, len);
}
