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

#include <string.h>
#include "mod_e222_m117.h"

#define DIGIT_BITS 58
#define CARRY_BITS (64 - DIGIT_BITS)
#define DIGIT_MASK 0x03ffffffffffffffL
#define HIGH_DIGIT_BITS 48
#define HIGH_CARRY_BITS (64 - HIGH_DIGIT_BITS)
#define HIGH_DIGIT_MASK 0x0000ffffffffffffL
#define MUL_DIGIT_BITS 29
#define MUL_DIGIT_MASK 0x1fffffff
#define MUL_OVERLAP_BITS (DIGIT_BITS - HIGH_DIGIT_BITS)
#define C_VAL 117

const mod_e222_m117 MODULUS  = { 0x03ffffffffffff8bL, 0x03ffffffffffffffL,
                                 0x03ffffffffffffffL, 0x0000ffffffffffffL };

static short carry_out(const mod_e222_m117 digits) {
  return digits[MOD_E222_M117_NDIGITS - 1] >> HIGH_DIGIT_BITS;
}

const mod_e222_m117 mod_e222_m117_zero = { 0L, 0L, 0L, 0L };

const mod_e222_m117 mod_e222_m117_one = { 0L, 0L, 0L, 1L };

void mod_e222_m117_unpack(const unsigned char bytes[MOD_E222_M117_NBYTES],
                          mod_e222_m117 digits) {
        digits[0] = ((long)bytes[0] & 0x00000000000000ffL) |
                    (((long)bytes[1] << 8) & 0x000000000000ff00L) |
                    (((long)bytes[2] << 16) & 0x0000000000ff0000L) |
                    (((long)bytes[3] << 24) & 0x00000000ff000000L) |
                    (((long)bytes[4] << 32) & 0x000000ff00000000L) |
                    (((long)bytes[5] << 40) & 0x0000ff0000000000L) |
                    (((long)bytes[6] << 48) & 0x00ff000000000000L) |
                    (((long)bytes[7] << 56) & 0x0300000000000000L);
        digits[1] = (((long)bytes[7] >> 2) & 0x000000000000003fL) |
                    (((long)bytes[8] << 6) & 0x0000000000003fc0L) |
                    (((long)bytes[9] << 14) & 0x00000000003fc000L) |
                    (((long)bytes[10] << 22) & 0x000000003fc00000L) |
                    (((long)bytes[11] << 30) & 0x0000003fc0000000L) |
                    (((long)bytes[12] << 38) & 0x00003fc000000000L) |
                    (((long)bytes[13] << 46) & 0x003fc00000000000L) |
                    (((long)bytes[14] << 54) & 0x03c0000000000000L);
        digits[2] = (((long)bytes[14] >> 4) & 0x000000000000000fL) |
                    (((long)bytes[15] << 4) & 0x0000000000000ff0L) |
                    (((long)bytes[16] << 12) & 0x00000000000ff000L) |
                    (((long)bytes[17] << 20) & 0x000000000ff00000L) |
                    (((long)bytes[18] << 28) & 0x0000000ff0000000L) |
                    (((long)bytes[19] << 36) & 0x00000ff000000000L) |
                    (((long)bytes[20] << 44) & 0x000ff00000000000L) |
                    (((long)bytes[21] << 52) & 0x03f0000000000000L);
        digits[3] = (((long)bytes[21] >> 6) & 0x0000000000000003L) |
                    (((long)bytes[22] << 2) & 0x00000000000003fcL) |
                    (((long)bytes[23] << 10) & 0x000000000003fc00L) |
                    (((long)bytes[24] << 18) & 0x0000000003fc0000L) |
                    (((long)bytes[25] << 26) & 0x00000003fc000000L) |
                    (((long)bytes[26] << 34) & 0x000003fc00000000L) |
                    (((long)bytes[27] << 42) & 0x0000fc0000000000L);
}

bool mod_e222_m117_eq(mod_e222_m117 a,
                      mod_e222_m117 b,
                      mod_e222_m117_scratchpad *scratch) {
  mod_e222_m117_norm(a, scratch);
  mod_e222_m117_norm(b, scratch);

  return mod_e222_m117_eq_norm(a, b);
}

bool mod_e222_m117_eq_norm(const mod_e222_m117 a,
                           const mod_e222_m117 b) {
  int out = 0;

  for(int i = 0; i < MOD_E222_M117_NBYTES; i++) {
    out |= a[i] ^ b[i];
  }

  return out != 0;
}

void mod_e222_m117_pack(mod_e222_m117 digits,
                        unsigned char bytes[MOD_E222_M117_NBYTES],
                        mod_e222_m117_scratchpad *scratch) {
  mod_e222_m117_norm(digits, scratch);
  mod_e222_m117_pack_norm(digits, bytes);
}

void mod_e222_m117_copy(mod_e222_m117 to,
                        const mod_e222_m117 from) {
  memcpy(to, from, sizeof(mod_e222_m117));
}

void mod_e222_m117_set(mod_e222_m117 digits, int val) {
  memset(digits, 0, sizeof(mod_e222_m117));
  mod_e222_m117_add_small(digits, val, digits);
}

void mod_e222_m117_pack_norm(const mod_e222_m117 digits,
                             unsigned char bytes[MOD_E222_M117_NBYTES]) {
        bytes[0] = (digits[0] & 0xff);
        bytes[1] = ((digits[0] >> 8) & 0xff);
        bytes[2] = ((digits[0] >> 16) & 0xff);
        bytes[3] = ((digits[0] >> 24) & 0xff);
        bytes[4] = ((digits[0] >> 32) & 0xff);
        bytes[5] = ((digits[0] >> 40) & 0xff);
        bytes[6] = ((digits[0] >> 48) & 0xff);
        bytes[7] = (((digits[0] >> 56) & 0x03) |
                    ((digits[1] << 2) & 0xfc));
        bytes[8] = ((digits[1] >> 6) & 0xff);
        bytes[9] = ((digits[1] >> 14) & 0xff);
        bytes[10] = ((digits[1] >> 22) & 0xff);
        bytes[11] = ((digits[1] >> 30) & 0xff);
        bytes[12] = ((digits[1] >> 38) & 0xff);
        bytes[13] = ((digits[1] >> 46) & 0xff);
        bytes[14] = (((digits[1] >> 54) & 0x0f) |
                     ((digits[2] << 4) & 0xf0));
        bytes[15] = ((digits[2] >> 4) & 0xff);
        bytes[16] = ((digits[2] >> 12) & 0xff);
        bytes[17] = ((digits[2] >> 20) & 0xff);
        bytes[18] = ((digits[2] >> 28) & 0xff);
        bytes[19] = ((digits[2] >> 36) & 0xff);
        bytes[20] = ((digits[2] >> 44) & 0xff);
        bytes[21] = (((digits[2] >> 52) & 0x3f) |
                     ((digits[3] << 6) & 0xc0));
        bytes[22] = ((digits[3] >> 2) & 0xff);
        bytes[23] = ((digits[3] >> 10) & 0xff);
        bytes[24] = ((digits[3] >> 18) & 0xff);
        bytes[25] = ((digits[3] >> 26) & 0xff);
        bytes[26] = ((digits[3] >> 34) & 0xff);
        bytes[27] = ((digits[3] >> 42) & 0x3f);
}

void mod_e222_m117_norm(mod_e222_m117 digits,
                        mod_e222_m117_scratchpad *scratch) {
  long *offset = scratch->d0;
  long *plusc = scratch->d1;

  mod_e222_m117_copy(offset, MODULUS);
  mod_e222_m117_copy(plusc, digits);

  mod_e222_m117_add_small(plusc, C_VAL, plusc);
  mod_e222_m117_mul_small(offset, carry_out(plusc), offset);
  mod_e222_m117_sub(digits, offset, digits);
}

void mod_e222_m117_add(const mod_e222_m117 a,
                       const mod_e222_m117 b,
                       mod_e222_m117 out) {
  const long a0 = a[0];
  const long a1 = a[1];
  const long a2 = a[2];
  const long a3 = a[3] & HIGH_DIGIT_MASK;

  const long b0 = b[0];
  const long b1 = b[1];
  const long b2 = b[2];
  const long b3 = b[3] & HIGH_DIGIT_MASK;

  const long cin = carry_out(a) + carry_out(b);
  const long s0 = a0 + b0 + (cin * C_VAL);
  const long c0 = s0 >> DIGIT_BITS;
  const long s1 = a1 + b1 + c0;
  const long c1 = s1 >> DIGIT_BITS;
  const long s2 = a2 + b2 + c1;
  const long c2 = s2 >> DIGIT_BITS;
  const long s3 = a3 + b3 + c2;

  out[0] = s0 & DIGIT_MASK;
  out[1] = s1 & DIGIT_MASK;
  out[2] = s2 & DIGIT_MASK;
  out[3] = s3;
}

void mod_e222_m117_add_small(const mod_e222_m117 a,
                             int b,
                             mod_e222_m117 out) {
  const long a0 = a[0];
  const long a1 = a[1];
  const long a2 = a[2];
  const long a3 = a[3] & HIGH_DIGIT_MASK;

  const long cin = carry_out(a);
  const long s0 = a0 + b + (cin * C_VAL);
  const long c0 = s0 >> DIGIT_BITS;
  const long s1 = a1 + c0;
  const long c1 = s1 >> DIGIT_BITS;
  const long s2 = a2 + c1;
  const long c2 = s2 >> DIGIT_BITS;
  const long s3 = a3 + c2;

  out[0] = s0 & DIGIT_MASK;
  out[1] = s1 & DIGIT_MASK;
  out[2] = s2 & DIGIT_MASK;
  out[3] = s3;
}

void mod_e222_m117_sub(const mod_e222_m117 a,
                       const mod_e222_m117 b,
                       mod_e222_m117 out) {
  const long a0 = a[0];
  const long a1 = a[1];
  const long a2 = a[2];
  const long a3 = a[3] & HIGH_DIGIT_MASK;

  const long b0 = b[0];
  const long b1 = b[1];
  const long b2 = b[2];
  const long b3 = b[3] & HIGH_DIGIT_MASK;

  const long cin = carry_out(a) - carry_out(b);
  const long s0 = a0 - b0 + (cin * C_VAL);
  const long c0 = s0 >> DIGIT_BITS;
  const long s1 = a1 - b1 + c0;
  const long c1 = s1 >> DIGIT_BITS;
  const long s2 = a2 - b2 + c1;
  const long c2 = s2 >> DIGIT_BITS;
  const long s3 = a3 - b3 + c2;

  out[0] = s0 & DIGIT_MASK;
  out[1] = s1 & DIGIT_MASK;
  out[2] = s2 & DIGIT_MASK;
  out[3] = s3;
}

void mod_e222_m117_sub_small(const mod_e222_m117 a,
                             int b,
                             mod_e222_m117 out) {
  const long a0 = a[0];
  const long a1 = a[1];
  const long a2 = a[2];
  const long a3 = a[3] & HIGH_DIGIT_MASK;

  const long cin = carry_out(a);
  const long s0 = a0 - b + (cin * C_VAL);
  const long c0 = s0 >> DIGIT_BITS;
  const long s1 = a1 + c0;
  const long c1 = s1 >> DIGIT_BITS;
  const long s2 = a2 + c1;
  const long c2 = s2 >> DIGIT_BITS;
  const long s3 = a3 + c2;

  out[0] = s0 & DIGIT_MASK;
  out[1] = s1 & DIGIT_MASK;
  out[2] = s2 & DIGIT_MASK;
  out[3] = s3;
}

void mod_e222_m117_neg(const mod_e222_m117 a,
                       mod_e222_m117 out) {
  mod_e222_m117_sub(mod_e222_m117_zero, a, out);
}

void mod_e222_m117_square(const mod_e222_m117 a,
                          mod_e222_m117 out) {
  const long a0 = a[0] & MUL_DIGIT_MASK;
  const long a1 = a[0] >> MUL_DIGIT_BITS;
  const long a2 = a[1] & MUL_DIGIT_MASK;
  const long a3 = a[1] >> MUL_DIGIT_BITS;
  const long a4 = a[2] & MUL_DIGIT_MASK;
  const long a5 = a[2] >> MUL_DIGIT_BITS;
  const long a6 = a[3] & MUL_DIGIT_MASK;
  const long a7 = a[3] >> MUL_DIGIT_BITS;

  // Combined multiples
  const long m_0_0 = a0 * a0;
  const long m_0_1 = a0 * a1;
  const long m_0_2 = a0 * a2;
  const long m_0_3 = a0 * a3;
  const long m_0_4 = a0 * a4;
  const long m_0_5 = a0 * a5;
  const long m_0_6 = a0 * a6;
  const long m_0_7 = a0 * a7;
  const long m_1_0 = m_0_1;
  const long m_1_1 = a1 * a1;
  const long m_1_2 = a1 * a2;
  const long m_1_3 = a1 * a3;
  const long m_1_4 = a1 * a4;
  const long m_1_5 = a1 * a5;
  const long m_1_6 = a1 * a6;
  const long m_1_7 = a1 * a7;
  const long m_2_0 = m_0_2;
  const long m_2_1 = m_1_2;
  const long m_2_2 = a2 * a2;
  const long m_2_3 = a2 * a3;
  const long m_2_4 = a2 * a4;
  const long m_2_5 = a2 * a5;
  const long m_2_6 = a2 * a6;
  const long m_2_7 = a2 * a7;
  const long m_3_0 = m_0_3;
  const long m_3_1 = m_1_3;
  const long m_3_2 = m_2_3;
  const long m_3_3 = a3 * a3;
  const long m_3_4 = a3 * a4;
  const long m_3_5 = a3 * a5;
  const long m_3_6 = a3 * a6;
  const long m_3_7 = a3 * a7;
  const long m_4_0 = m_0_4;
  const long m_4_1 = m_1_4;
  const long m_4_2 = m_2_4;
  const long m_4_3 = m_3_4;
  const long m_4_4 = a4 * a4;
  const long m_4_5 = a4 * a5;
  const long m_4_6 = a4 * a6;
  const long m_4_7 = a4 * a7;
  const long m_5_0 = m_0_5;
  const long m_5_1 = m_1_5;
  const long m_5_2 = m_2_5;
  const long m_5_3 = m_3_5;
  const long m_5_4 = m_4_5;
  const long m_5_5 = a5 * a5;
  const long m_5_6 = a5 * a6;
  const long m_5_7 = a5 * a7;
  const long m_6_0 = m_0_6;
  const long m_6_1 = m_1_6;
  const long m_6_2 = m_2_6;
  const long m_6_3 = m_3_6;
  const long m_6_4 = m_4_6;
  const long m_6_5 = m_5_6;
  const long m_6_6 = a6 * a6;
  const long m_6_7 = a6 * a7;
  const long m_7_0 = m_0_7;
  const long m_7_1 = m_1_7;
  const long m_7_2 = m_2_7;
  const long m_7_3 = m_3_7;
  const long m_7_4 = m_4_7;
  const long m_7_5 = m_5_7;
  const long m_7_6 = m_6_7;
  const long m_7_7 = a7 * a7;

  // Compute the 40-digit combined product using 64-bit operations.
  const long d0 =
      m_0_0 + ((m_0_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_1_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS);
  const long c0 = d0 >> DIGIT_BITS;
  const long d1 =
      (m_0_1 >> MUL_DIGIT_BITS) + m_0_2 +
      ((m_0_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_1_0 >> MUL_DIGIT_BITS) + m_1_1 +
      ((m_1_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      m_2_0 + ((m_2_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_3_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c0;
  const long c1 = d1 >> DIGIT_BITS;
  const long d2 =
      (m_0_3 >> MUL_DIGIT_BITS) + m_0_4 +
      ((m_0_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_1_2 >> MUL_DIGIT_BITS) + m_1_3 +
      ((m_1_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_2_1 >> MUL_DIGIT_BITS) + m_2_2 +
      ((m_2_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_3_0 >> MUL_DIGIT_BITS) + m_3_1 +
      ((m_3_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      m_4_0 + ((m_4_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_5_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c1;
  const long c2 = d2 >> DIGIT_BITS;
  const long d3 =
      (m_0_5 >> MUL_DIGIT_BITS) + m_0_6 +
      ((m_0_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_1_4 >> MUL_DIGIT_BITS) + m_1_5 +
      ((m_1_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_2_3 >> MUL_DIGIT_BITS) + m_2_4 +
      ((m_2_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_3_2 >> MUL_DIGIT_BITS) + m_3_3 +
      ((m_3_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_4_1 >> MUL_DIGIT_BITS) + m_4_2 +
      ((m_4_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_5_0 >> MUL_DIGIT_BITS) + m_5_1 +
      ((m_5_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      m_6_0 + ((m_6_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_7_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c2;
  const long c3 = d3 >> DIGIT_BITS;
  const long d4 =
      (m_0_7 >> MUL_DIGIT_BITS) +
      (m_1_6 >> MUL_DIGIT_BITS) + m_1_7 +
      (m_2_5 >> MUL_DIGIT_BITS) + m_2_6 +
      ((m_2_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_3_4 >> MUL_DIGIT_BITS) + m_3_5 +
      ((m_3_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_4_3 >> MUL_DIGIT_BITS) + m_4_4 +
      ((m_4_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_5_2 >> MUL_DIGIT_BITS) + m_5_3 +
      ((m_5_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_6_1 >> MUL_DIGIT_BITS) + m_6_2 +
      ((m_6_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_7_0 >> MUL_DIGIT_BITS) + m_7_1 +
      ((m_7_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      c3;
  const long c4 = d4 >> DIGIT_BITS;
  const long d5 =
      (m_2_7 >> MUL_DIGIT_BITS) +
      (m_3_6 >> MUL_DIGIT_BITS) + m_3_7 +
      (m_4_5 >> MUL_DIGIT_BITS) + m_4_6 +
      ((m_4_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_5_4 >> MUL_DIGIT_BITS) + m_5_5 +
      ((m_5_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_6_3 >> MUL_DIGIT_BITS) + m_6_4 +
      ((m_6_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_7_2 >> MUL_DIGIT_BITS) + m_7_3 +
      ((m_7_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      c4;
  const long c5 = d5 >> DIGIT_BITS;
  const long d6 =
      (m_4_7 >> MUL_DIGIT_BITS) +
      (m_5_6 >> MUL_DIGIT_BITS) + m_5_7 +
      (m_6_5 >> MUL_DIGIT_BITS) + m_6_6 +
      ((m_6_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_7_4 >> MUL_DIGIT_BITS) + m_7_5 +
      ((m_7_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      c5;
  const long c6 = d6 >> DIGIT_BITS;
  const long d7 =
      (m_6_7 >> MUL_DIGIT_BITS) +
      (m_7_6 >> MUL_DIGIT_BITS) + m_7_7 +
      c6;

  // Modular reduction by a pseudo-mersenne prime of the form 2^n - c.

  // These are the n low-order
  const long l0_0 = d0 & DIGIT_MASK;
  const long l1_0 = d1 & DIGIT_MASK;
  const long l2_0 = d2 & DIGIT_MASK;
  const long l3_0 = d3 & HIGH_DIGIT_MASK;

  // Shift the high bits down into another n-bit number.
  const long h0_0 = ((d3 & DIGIT_MASK) >> HIGH_DIGIT_BITS) |
                    ((d4 & 0x000000000007ffffL) << 10);
  const long h1_0 = (d4 & 0x0000fffffff80000L) >> 19;
  const long h2_0 = ((d4 & 0x03ff000000000000L) >> HIGH_DIGIT_BITS) |
                    ((d5 & 0x000000000007ffffL) << 10);
  const long h3_0 = (d5 & 0x0000fffffff80000L) >> 19;
  const long h4_0 = ((d5 & 0x03ff000000000000L) >> HIGH_DIGIT_BITS) |
                    ((d6 & 0x000000000007ffffL) << 10);
  const long h5_0 = (d6 & 0x0000fffffff80000L) >> 19;
  const long h6_0 = ((d6 & 0x03ff000000000000L) >> HIGH_DIGIT_BITS) |
                    ((d7 & 0x000000000007ffffL) << 10);
  const long h7_0 = d7 >> 19;

  // Multiply by C
  const long hc0_0 = h0_0 * C_VAL;
  const long hc1_0 = h1_0 * C_VAL;
  const long hc2_0 = h2_0 * C_VAL;
  const long hc3_0 = h3_0 * C_VAL;
  const long hc4_0 = h4_0 * C_VAL;
  const long hc5_0 = h5_0 * C_VAL;
  const long hc6_0 = h6_0 * C_VAL;
  const long hc7_0 = h7_0 * C_VAL;

  const long hm0_0 = hc0_0 + ((hc1_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS);
  const long hmk0_0 = hm0_0 >> DIGIT_BITS;
  const long hm1_0 =
      (hc1_0 >> MUL_DIGIT_BITS) + hc2_0 +
      ((hc3_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + hmk0_0;
  const long hmk1_0 = hm1_0 >> DIGIT_BITS;
  const long hm2_0 =
      (hc3_0 >> MUL_DIGIT_BITS) + hc4_0 +
      ((hc5_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + hmk1_0;
  const long hmk2_0 = hm2_0 >> DIGIT_BITS;
  const long hm3_0 =
      (hc5_0 >> MUL_DIGIT_BITS) + hc6_0 +
      ((hc7_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + hmk2_0;

  // Add h and l.
  const long kin_0 = hm3_0 >> HIGH_DIGIT_BITS;
  const long s0_0 = l0_0 + (hm0_0 & DIGIT_MASK) + (kin_0 * C_VAL);
  const long k0_0 = s0_0 >> DIGIT_BITS;
  const long s1_0 = l1_0 + (hm1_0 & DIGIT_MASK) + k0_0;
  const long k1_0 = s1_0 >> DIGIT_BITS;
  const long s2_0 = l2_0 + (hm2_0 & DIGIT_MASK) + k1_0;
  const long k2_0 = s2_0 >> DIGIT_BITS;
  const long s3_0 = l3_0 + (hm3_0 & HIGH_DIGIT_MASK) + k2_0;

  out[0] = s0_0 & DIGIT_MASK;
  out[1] = s1_0 & DIGIT_MASK;
  out[2] = s2_0 & DIGIT_MASK;
  out[3] = s3_0;
}

void mod_e222_m117_mul(const mod_e222_m117 a,
                       const mod_e222_m117 b,
                       mod_e222_m117 out) {
  const long a0 = a[0] & MUL_DIGIT_MASK;
  const long a1 = a[0] >> MUL_DIGIT_BITS;
  const long a2 = a[1] & MUL_DIGIT_MASK;
  const long a3 = a[1] >> MUL_DIGIT_BITS;
  const long a4 = a[2] & MUL_DIGIT_MASK;
  const long a5 = a[2] >> MUL_DIGIT_BITS;
  const long a6 = a[3] & MUL_DIGIT_MASK;
  const long a7 = a[3] >> MUL_DIGIT_BITS;

  const long b0 = b[0] & MUL_DIGIT_MASK;
  const long b1 = b[0] >> MUL_DIGIT_BITS;
  const long b2 = b[1] & MUL_DIGIT_MASK;
  const long b3 = b[1] >> MUL_DIGIT_BITS;
  const long b4 = b[2] & MUL_DIGIT_MASK;
  const long b5 = b[2] >> MUL_DIGIT_BITS;
  const long b6 = b[3] & MUL_DIGIT_MASK;
  const long b7 = b[3] >> MUL_DIGIT_BITS;

  // Combined multiples
  const long m_0_0 = a0 * b0;
  const long m_0_1 = a0 * b1;
  const long m_0_2 = a0 * b2;
  const long m_0_3 = a0 * b3;
  const long m_0_4 = a0 * b4;
  const long m_0_5 = a0 * b5;
  const long m_0_6 = a0 * b6;
  const long m_0_7 = a0 * b7;
  const long m_1_0 = a1 * b0;
  const long m_1_1 = a1 * b1;
  const long m_1_2 = a1 * b2;
  const long m_1_3 = a1 * b3;
  const long m_1_4 = a1 * b4;
  const long m_1_5 = a1 * b5;
  const long m_1_6 = a1 * b6;
  const long m_1_7 = a1 * b7;
  const long m_2_0 = a2 * b0;
  const long m_2_1 = a2 * b1;
  const long m_2_2 = a2 * b2;
  const long m_2_3 = a2 * b3;
  const long m_2_4 = a2 * b4;
  const long m_2_5 = a2 * b5;
  const long m_2_6 = a2 * b6;
  const long m_2_7 = a2 * b7;
  const long m_3_0 = a3 * b0;
  const long m_3_1 = a3 * b1;
  const long m_3_2 = a3 * b2;
  const long m_3_3 = a3 * b3;
  const long m_3_4 = a3 * b4;
  const long m_3_5 = a3 * b5;
  const long m_3_6 = a3 * b6;
  const long m_3_7 = a3 * b7;
  const long m_4_0 = a4 * b0;
  const long m_4_1 = a4 * b1;
  const long m_4_2 = a4 * b2;
  const long m_4_3 = a4 * b3;
  const long m_4_4 = a4 * b4;
  const long m_4_5 = a4 * b5;
  const long m_4_6 = a4 * b6;
  const long m_4_7 = a4 * b7;
  const long m_5_0 = a5 * b0;
  const long m_5_1 = a5 * b1;
  const long m_5_2 = a5 * b2;
  const long m_5_3 = a5 * b3;
  const long m_5_4 = a5 * b4;
  const long m_5_5 = a5 * b5;
  const long m_5_6 = a5 * b6;
  const long m_5_7 = a5 * b7;
  const long m_6_0 = a6 * b0;
  const long m_6_1 = a6 * b1;
  const long m_6_2 = a6 * b2;
  const long m_6_3 = a6 * b3;
  const long m_6_4 = a6 * b4;
  const long m_6_5 = a6 * b5;
  const long m_6_6 = a6 * b6;
  const long m_6_7 = a6 * b7;
  const long m_7_0 = a7 * b0;
  const long m_7_1 = a7 * b1;
  const long m_7_2 = a7 * b2;
  const long m_7_3 = a7 * b3;
  const long m_7_4 = a7 * b4;
  const long m_7_5 = a7 * b5;
  const long m_7_6 = a7 * b6;
  const long m_7_7 = a7 * b7;

  // Compute the combined product using 64-bit operations.
  const long d0 =
      m_0_0 + ((m_0_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_1_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS);
  const long c0 = d0 >> DIGIT_BITS;
  const long d1 =
      (m_0_1 >> MUL_DIGIT_BITS) + m_0_2 +
      ((m_0_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_1_0 >> MUL_DIGIT_BITS) + m_1_1 +
      ((m_1_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      m_2_0 + ((m_2_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_3_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c0;
  const long c1 = d1 >> DIGIT_BITS;
  const long d2 =
      (m_0_3 >> MUL_DIGIT_BITS) + m_0_4 +
      ((m_0_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_1_2 >> MUL_DIGIT_BITS) + m_1_3 +
      ((m_1_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_2_1 >> MUL_DIGIT_BITS) + m_2_2 +
      ((m_2_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_3_0 >> MUL_DIGIT_BITS) + m_3_1 +
      ((m_3_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      m_4_0 + ((m_4_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_5_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c1;
  const long c2 = d2 >> DIGIT_BITS;
  const long d3 =
      (m_0_5 >> MUL_DIGIT_BITS) + m_0_6 +
      ((m_0_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_1_4 >> MUL_DIGIT_BITS) + m_1_5 +
      ((m_1_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_2_3 >> MUL_DIGIT_BITS) + m_2_4 +
      ((m_2_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_3_2 >> MUL_DIGIT_BITS) + m_3_3 +
      ((m_3_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_4_1 >> MUL_DIGIT_BITS) + m_4_2 +
      ((m_4_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_5_0 >> MUL_DIGIT_BITS) + m_5_1 +
      ((m_5_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      m_6_0 + ((m_6_1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      ((m_7_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c2;
  const long c3 = d3 >> DIGIT_BITS;
  const long d4 =
      (m_0_7 >> MUL_DIGIT_BITS) +
      (m_1_6 >> MUL_DIGIT_BITS) + m_1_7 +
      (m_2_5 >> MUL_DIGIT_BITS) + m_2_6 +
      ((m_2_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_3_4 >> MUL_DIGIT_BITS) + m_3_5 +
      ((m_3_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_4_3 >> MUL_DIGIT_BITS) + m_4_4 +
      ((m_4_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_5_2 >> MUL_DIGIT_BITS) + m_5_3 +
      ((m_5_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_6_1 >> MUL_DIGIT_BITS) + m_6_2 +
      ((m_6_3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_7_0 >> MUL_DIGIT_BITS) + m_7_1 +
      ((m_7_2 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      c3;
  const long c4 = d4 >> DIGIT_BITS;
  const long d5 =
      (m_2_7 >> MUL_DIGIT_BITS) +
      (m_3_6 >> MUL_DIGIT_BITS) + m_3_7 +
      (m_4_5 >> MUL_DIGIT_BITS) + m_4_6 +
      ((m_4_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_5_4 >> MUL_DIGIT_BITS) + m_5_5 +
      ((m_5_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_6_3 >> MUL_DIGIT_BITS) + m_6_4 +
      ((m_6_5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_7_2 >> MUL_DIGIT_BITS) + m_7_3 +
      ((m_7_4 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
           c4;
  const long c5 = d5 >> DIGIT_BITS;
  const long d6 =
      (m_4_7 >> MUL_DIGIT_BITS) +
      (m_5_6 >> MUL_DIGIT_BITS) + m_5_7 +
      (m_6_5 >> MUL_DIGIT_BITS) + m_6_6 +
      ((m_6_7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
      (m_7_4 >> MUL_DIGIT_BITS) + m_7_5 +
      ((m_7_6 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
           c5;
  const long c6 = d6 >> DIGIT_BITS;
  const long d7 =
      (m_6_7 >> MUL_DIGIT_BITS) +
      (m_7_6 >> MUL_DIGIT_BITS) + m_7_7 +
      c6;

  // Modular reduction by a pseudo-mersenne prime of the form 2^n - c.

  // These are the n low-order
  const long l0_0 = d0 & DIGIT_MASK;
  const long l1_0 = d1 & DIGIT_MASK;
  const long l2_0 = d2 & DIGIT_MASK;
  const long l3_0 = d3 & HIGH_DIGIT_MASK;

  // Shift the high bits down into another n-bit number.
  const long h0_0 = ((d3 & DIGIT_MASK) >> HIGH_DIGIT_BITS) |
                    ((d4 & 0x000000000007ffffL) << 10);
  const long h1_0 = (d4 & 0x0000fffffff80000L) >> 19;
  const long h2_0 = ((d4 & 0x03ff000000000000L) >> HIGH_DIGIT_BITS) |
                    ((d5 & 0x000000000007ffffL) << 10);
  const long h3_0 = (d5 & 0x0000fffffff80000L) >> 19;
  const long h4_0 = ((d5 & 0x03ff000000000000L) >> HIGH_DIGIT_BITS) |
                    ((d6 & 0x000000000007ffffL) << 10);
  const long h5_0 = (d6 & 0x0000fffffff80000L) >> 19;
  const long h6_0 = ((d6 & 0x03ff000000000000L) >> HIGH_DIGIT_BITS) |
                    ((d7 & 0x000000000007ffffL) << 10);
  const long h7_0 = d7 >> 19;

  // Multiply by C
  const long hc0_0 = h0_0 * C_VAL;
  const long hc1_0 = h1_0 * C_VAL;
  const long hc2_0 = h2_0 * C_VAL;
  const long hc3_0 = h3_0 * C_VAL;
  const long hc4_0 = h4_0 * C_VAL;
  const long hc5_0 = h5_0 * C_VAL;
  const long hc6_0 = h6_0 * C_VAL;
  const long hc7_0 = h7_0 * C_VAL;

  const long hm0_0 = hc0_0 + ((hc1_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS);
  const long hmk0_0 = hm0_0 >> DIGIT_BITS;
  const long hm1_0 =
      (hc1_0 >> MUL_DIGIT_BITS) + hc2_0 +
      ((hc3_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + hmk0_0;
  const long hmk1_0 = hm1_0 >> DIGIT_BITS;
  const long hm2_0 =
      (hc3_0 >> MUL_DIGIT_BITS) + hc4_0 +
      ((hc5_0 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + hmk1_0;
  const long hmk2_0 = hm2_0 >> DIGIT_BITS;
  const long hm3_0 =
      (hc5_0 >> MUL_DIGIT_BITS) + hc6_0 +
      (hc7_0 << MUL_DIGIT_BITS) + hmk2_0;

  // Add h and l.
  const long kin_0 = hm3_0 >> HIGH_DIGIT_BITS;
  const long s0_0 = l0_0 + (hm0_0 & DIGIT_MASK) + (kin_0 * C_VAL);
  const long k0_0 = s0_0 >> DIGIT_BITS;
  const long s1_0 = l1_0 + (hm1_0 & DIGIT_MASK) + k0_0;
  const long k1_0 = s1_0 >> DIGIT_BITS;
  const long s2_0 = l2_0 + (hm2_0 & DIGIT_MASK) + k1_0;
  const long k2_0 = s2_0 >> DIGIT_BITS;
  const long s3_0 = l3_0 + (hm3_0 & HIGH_DIGIT_MASK) + k2_0;

  out[0] = s0_0 & DIGIT_MASK;
  out[1] = s1_0 & DIGIT_MASK;
  out[2] = s2_0 & DIGIT_MASK;
  out[3] = s3_0;
}

void mod_e222_m117_mul_small(const mod_e222_m117 a,
                             int b,
                             mod_e222_m117 out) {
  const long a0 = a[0] & MUL_DIGIT_MASK;
  const long a1 = a[0] >> MUL_DIGIT_BITS;
  const long a2 = a[1] & MUL_DIGIT_MASK;
  const long a3 = a[1] >> MUL_DIGIT_BITS;
  const long a4 = a[2] & MUL_DIGIT_MASK;
  const long a5 = a[2] >> MUL_DIGIT_BITS;
  const long a6 = a[3] & MUL_DIGIT_MASK;
  const long a7 = (a[3] & HIGH_DIGIT_MASK) >> MUL_DIGIT_BITS;

  const long m0 = a0 * b;
  const long m1 = a1 * b;
  const long m2 = a2 * b;
  const long m3 = a3 * b;
  const long m4 = a4 * b;
  const long m5 = a5 * b;
  const long m6 = a6 * b;
  const long m7 = a7 * b;

  const long cin = carry_out(a);
  const long d0 =
    m0 + ((m1 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) +
    (cin * C_VAL * b);
  const long c0 = d0 >> DIGIT_BITS;
  const long d1 =
    (m1 >> MUL_DIGIT_BITS) + m2 +
    ((m3 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c0;
  const long c1 = d1 >> DIGIT_BITS;
  const long d2 =
    (m3 >> MUL_DIGIT_BITS) + m4 +
    ((m5 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c1;
  const long c2 = d2 >> DIGIT_BITS;
  const long d3 =
    (m5 >> MUL_DIGIT_BITS) + m6 +
    ((m7 & MUL_DIGIT_MASK) << MUL_DIGIT_BITS) + c2;
  const long c3 = d3 >> HIGH_DIGIT_BITS;

  const long kin = ((m7 & 0xffffffffe0000000L) >> 19) + c3;
  const long s0 = (d0 & DIGIT_MASK) + (kin * C_VAL);
  const long k0 = s0 >> DIGIT_BITS;
  const long s1 = (d1 & DIGIT_MASK) + k0;
  const long k1 = s1 >> DIGIT_BITS;
  const long s2 = (d2 & DIGIT_MASK) + k1;
  const long k2 = s2 >> DIGIT_BITS;
  const long s3 = (d3 & HIGH_DIGIT_MASK) + k2;

  out[0] = s0 & DIGIT_MASK;
  out[1] = s1 & DIGIT_MASK;
  out[2] = s2 & DIGIT_MASK;
  out[3] = s3;
}

void mod_e222_m117_div(const mod_e222_m117 a,
                       const mod_e222_m117 b,
                       mod_e222_m117 c,
                       mod_e222_m117_scratchpad *scratch) {
  long *divisor = scratch->d2;

  mod_e222_m117_inv(b, divisor, scratch);
  mod_e222_m117_mul(a, divisor, c);
}

void mod_e222_m117_inv(const mod_e222_m117 in,
                       mod_e222_m117 out,
                       mod_e222_m117_scratchpad *scratch) {
  // First digit is 1.
  long *sqval = scratch->d0;

  memcpy(sqval, in, sizeof(mod_e222_m117));
  memmove(out, in, sizeof(mod_e222_m117));

  // Second and third digits are 0.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);

  // Fourth digit is 1.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(out, sqval, out);

  // Fifth, sixths, and seventh digits are 0.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);

  // All the remaining digits are 1.
  for(int i = 7; i < 222; i++) {
    mod_e222_m117_square(sqval, sqval);
    mod_e222_m117_mul(out, sqval, out);
  }
}

static void legendre_power(mod_e222_m117 digits,
                           mod_e222_m117_scratchpad *scratch) {
  // First digit is 1.
  long *sqval = scratch->d0;

  memcpy(sqval, digits, sizeof(mod_e222_m117));

  // Second digit is 0.
  mod_e222_m117_square(sqval, sqval);

  // Third digit is 1.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(digits, sqval, digits);

  // Fourth, fifth, and sixth digits are 0.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);

  // All the remaining digits are 1.
  for(int i = 6; i < 221; i++) {
    mod_e222_m117_square(sqval, sqval);
    mod_e222_m117_mul(digits, sqval, digits);
  }
}


int mod_e222_m117_leg(const mod_e222_m117 digits,
                      mod_e222_m117_scratchpad *scratch) {
  memcpy(scratch->d2, digits, sizeof(mod_e222_m117));

  legendre_power(scratch->d2, scratch);
  mod_e222_m117_norm(scratch->d2, scratch);

  const long low = (scratch->d2[0] << CARRY_BITS) >> CARRY_BITS;
  const unsigned char sign = (unsigned char)(low >> (DIGIT_BITS - 1));
  const unsigned char offset = (C_VAL * sign);
  const unsigned char result = (low + offset);

  return result;
}

int mod_e222_m117_is_zero(mod_e222_m117 digits,
                          mod_e222_m117_scratchpad *scratch) {
  mod_e222_m117_norm(digits, scratch);

  return mod_e222_m117_is_zero_norm(digits);
}

int mod_e222_m117_is_zero_norm(const mod_e222_m117 digits) {
  long out = 0;

  for(int i = 0; i < MOD_E222_M117_NDIGITS; i++) {
    out |= digits[i];
  }

  out |= out >> 32;
  out |= out >> 16;
  out |= out >> 8;
  out |= out >> 4;
  out |= out >> 2;
  out |= out >> 1;

  return (out & 0x1) ^ 0x1;
}

void mod_e222_m117_sqrt(const mod_e222_m117 in,
                        mod_e222_m117 out,
                        mod_e222_m117_scratchpad *scratch) {
  // First digit is 1.
  long *sqval = scratch->d0;

  memcpy(sqval, in, sizeof(mod_e222_m117));
  memmove(out, in, sizeof(mod_e222_m117));

  // Second digit is 1.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(out, sqval, out);

  // Third, fourth, and fifth digits are 0.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);

  // All remaining digits are 1.
  for (int i = 5; i < 220; i++) {
    mod_e222_m117_square(sqval, sqval);
    mod_e222_m117_mul(out, sqval, out);
  }
}

void mod_e222_m117_invsqrt(const mod_e222_m117 in,
                           mod_e222_m117 out,
                           mod_e222_m117_scratchpad *scratch) {
  // First digit is 1.
  long *sqval = scratch->d0;

  memcpy(sqval, in, sizeof(mod_e222_m117));
  memmove(out, in, sizeof(mod_e222_m117));

  // Second and third digits are 1.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(out, sqval, out);
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(out, sqval, out);

  // Fourth and fifth digits are 0.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_square(sqval, sqval);

  // Sixth digit is 1.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(out, sqval, out);

  // Seventh digit is 0.
  mod_e222_m117_square(sqval, sqval);

  // All digits up to 220 are 1.
  for(int i = 7; i < 220; i++) {
    mod_e222_m117_square(sqval, sqval);
    mod_e222_m117_mul(out, sqval, out);
  }

  // 220th digit is 0.
  mod_e222_m117_square(sqval, sqval);

  // Last digit is 1.
  mod_e222_m117_square(sqval, sqval);
  mod_e222_m117_mul(out, sqval, out);
}
