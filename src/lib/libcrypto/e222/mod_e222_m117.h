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
#ifndef MOD_E222_M117_H
#define MOD_E222_M117_H

#include <stdbool.h>

#define MOD_E222_M117_NDIGITS 4
#define MOD_E222_M117_NBYTES 28

typedef long mod_e222_m117[MOD_E222_M117_NDIGITS];

typedef struct mod_e222_m117_scratchpad {
  mod_e222_m117 d0;
  mod_e222_m117 d1;
  mod_e222_m117 d2;
} mod_e222_m117_scratchpad;

extern const mod_e222_m117 mod_e222_m117_zero;

extern const mod_e222_m117 mod_e222_m117_one;

extern void mod_e222_m117_unpack(const unsigned char bytes[MOD_E222_M117_NBYTES],
                                 mod_e222_m117 digits);
extern void mod_e222_m117_pack(mod_e222_m117 num,
                               unsigned char bytes[MOD_E222_M117_NBYTES],
                               mod_e222_m117_scratchpad *scratch);
extern void mod_e222_m117_copy(mod_e222_m117 to,
                               const mod_e222_m117 from);
extern void mod_e222_m117_set(mod_e222_m117 num, int val);
extern bool mod_e222_m117_eq(mod_e222_m117 a,
                             mod_e222_m117 b,
                             mod_e222_m117_scratchpad *scratch);
extern bool mod_e222_m117_eq_norm(const mod_e222_m117 a,
                                  const mod_e222_m117 b);
extern void mod_e222_m117_pack_norm(const mod_e222_m117 num,
                                    unsigned char *bytes);

extern void mod_e222_m117_norm(mod_e222_m117 num,
                               mod_e222_m117_scratchpad *scratch);

extern void mod_e222_m117_add(const mod_e222_m117 a,
                              const mod_e222_m117 b,
                              mod_e222_m117 c);
extern void mod_e222_m117_add_small(const mod_e222_m117 a,
                                    int b,
                                    mod_e222_m117 c);
extern void mod_e222_m117_sub(const mod_e222_m117 a,
                              const mod_e222_m117 b,
                              mod_e222_m117 c);
extern void mod_e222_m117_sub_small(const mod_e222_m117 a,
                                    int b,
                                    mod_e222_m117 c);
extern void mod_e222_m117_neg(const mod_e222_m117 a,
                              mod_e222_m117 c);
extern void mod_e222_m117_square(const mod_e222_m117 a,
                                 mod_e222_m117 c);
extern void mod_e222_m117_mul(const mod_e222_m117 a,
                              const mod_e222_m117 b,
                              mod_e222_m117 c);
extern void mod_e222_m117_mul_small(const mod_e222_m117 a,
                                    int b,
                                    mod_e222_m117 c);
extern void mod_e222_m117_div(const mod_e222_m117 a,
                              const mod_e222_m117 b,
                              mod_e222_m117 c,
                              mod_e222_m117_scratchpad *scratch);
extern void mod_e222_m117_inv(const mod_e222_m117 a,
                              mod_e222_m117 c,
                              mod_e222_m117_scratchpad *scratch);

extern int mod_e222_m117_leg(const mod_e222_m117 a,
                             mod_e222_m117_scratchpad *scratch);
extern int mod_e222_m117_is_zero(mod_e222_m117 a,
                                 mod_e222_m117_scratchpad *scratch);
extern int mod_e222_m117_is_zero_norm(const mod_e222_m117 a);

extern void mod_e222_m117_sqrt(const mod_e222_m117 a,
                               mod_e222_m117 c,
                               mod_e222_m117_scratchpad *scratch);
extern void mod_e222_m117_invsqrt(const mod_e222_m117 a,
                                  mod_e222_m117 c,
                                  mod_e222_m117_scratchpad *scratch);

extern void mod_e222_m117_signum(mod_e222_m117 a,
                                 mod_e222_m117_scratchpad *scratch);
extern void mod_e222_m117_abs(mod_e222_m117 a,
                              mod_e222_m117 c,
                              mod_e222_m117_scratchpad *scratch);

#endif
