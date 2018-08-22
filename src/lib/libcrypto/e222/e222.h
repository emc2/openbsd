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
#ifndef E222_H
#define E222_H

#include <stdbool.h>

#include "mod_e222_m117.h"

typedef struct e222 {
  mod_e222_m117 x;
  mod_e222_m117 y;
  mod_e222_m117 t;
  mod_e222_m117 z;
} e222;

typedef struct e222_scratchpad {
  mod_e222_m117_scratchpad inner;
  mod_e222_m117 r0;
  mod_e222_m117 r1;
  mod_e222_m117 r2;
  mod_e222_m117 r3;
  mod_e222_m117 r4;
  mod_e222_m117 r5;
} e222_scratchpad;

/* Initialization and copying */
extern void e222_from_edwards(const mod_e222_m117 x,
                              const mod_e222_m117 y,
                              e222 *p);
extern void e222_from_extended(const mod_e222_m117 x,
                               const mod_e222_m117 y,
                               const mod_e222_m117 t,
                               const mod_e222_m117 z,
                               e222 *p);
extern void e222_copy(const e222 *from,
                      e222 *to);

/* Extended representation */
extern void e222_scale(e222 *p,
                       mod_e222_m117_scratchpad *scratch);
extern bool e222_scaled_eq(e222 *a,
                           e222 *b,
                           mod_e222_m117_scratchpad *scratch);
extern bool e222_eq(e222 *a,
                    e222 *b,
                    mod_e222_m117_scratchpad *scratch);

/* Arithmetic */
extern void e222_add(const e222 *a,
                     const e222 *b,
                     e222 *c,
                     e222_scratchpad *scratch);
extern void e222_double(const e222 *a,
                        e222 *c);
extern void e222_mul_x(const e222 *p,
                       const mod_e222_m117 n,
                       mod_e222_m117 x);
extern void e222_mul(const e222 *p,
                     const mod_e222_m117 n,
                     e222 *out);

/* Elligator 1 hash on plain points */
extern void e222_from_hash(const mod_e222_m117 r,
                           e222 *p,
                           e222_scratchpad *scratch);
extern bool e222_to_hash(const e222 *p,
                         mod_e222_m117 r,
                         e222_scratchpad *scratch);
extern bool e222_can_hash(const e222 *p,
                          e222_scratchpad *scratch);

/* Decaf-compressed points */
extern bool e222_decaf_eq(const e222 *a,
                          const e222 *b,
                          e222_scratchpad *scratch);
extern bool e222_decompress(const mod_e222_m117 s,
                            e222 *p,
                            e222_scratchpad *scratch);
extern void e222_compress(const e222 *p,
                          mod_e222_m117 s,
                          e222_scratchpad *scratch);
extern void e222_decaf_from_hash(const mod_e222_m117 r,
                                 e222 *p,
                                 e222_scratchpad *scratch);
extern void e222_decaf_to_hash(const e222 *p,
                               const mod_e222_m117 r,
                               e222_scratchpad *scratch);
extern bool e222_decaf_can_hash(const e222 *p,
                                e222_scratchpad *scratch);

#endif
