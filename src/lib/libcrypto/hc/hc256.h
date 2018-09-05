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

#ifndef HEADER_HC256_H
#define HEADER_HC256_H

#include <stddef.h>
#include <stdint.h>

#define TABLE_SIZE 1024

#define IV_BITS 256

#define IV_LEN (IV_BITS / 8)

#define KEY_BITS 256

#define KEY_LEN (KEY_BITS / 8)

typedef struct {
  /* ptable and qtable as per the paper */
  uint32_t ptable[TABLE_SIZE];
  uint32_t qtable[TABLE_SIZE];
  /* Current keystream word */
  uint32_t word;
  /* Current generation index, as per the paper */
  unsigned int idx;
  /* Offset into the keystream word, 0-3, set to 4 initially */
  uint8_t offset;

} HC256_ctx;

void HC256_set_key(HC256_ctx *ctx,
                   const unsigned char *key);
void HC256_set_iv(HC256_ctx *ctx,
                  const unsigned char *iv);
void HC256(HC256_ctx *ctx,
           unsigned char *out,
           const unsigned char *in,
           size_t len);

void CRYPTO_HC256(unsigned char *out,
                  const unsigned char *in,
                  size_t len,
                  const unsigned char key[KEY_LEN],
                  const unsigned char iv[IV_LEN]);

#endif
