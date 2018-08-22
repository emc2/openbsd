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

#include <string.h>

#include "mod_e222_m117.h"
#include "e222.h"

static const int edwards_d = 160102;

static const mod_e222_m117 elligator_c =
  { 0x0181a93fab21d7a1L, 0x03c49e9124566f65L,
    0x03f3e2dc4c0dbf5dL, 0x00000ffebae73d9bL };

static const mod_e222_m117 elligator_s =
  { 0x00d463bd7f8d7a88L, 0x030567315d06dbd2L,
    0x012488b2d539972bL, 0x0000b6a1c2f545caL };

static const mod_e222_m117 elligator_r =
  { 0x0228f9a829825fc6L, 0x005d6e598a0655caL,
    0x02797e348510fb90L, 0x0000aa21670a0d7fL };

void e222_scale(e222 *p,
                mod_e222_m117_scratchpad *scratch) {
  mod_e222_m117_inv(p->z, p->z, scratch);
  mod_e222_m117_mul(p->x, p->z, p->x);
  mod_e222_m117_mul(p->y, p->z, p->y);
  memcpy(p->z, mod_e222_m117_zero, sizeof(mod_e222_m117));
}

bool e222_scaled_eq(e222 *a,
                    e222 *b,
                    mod_e222_m117_scratchpad *scratch) {
  return mod_e222_m117_eq(a->x, b->x, scratch) &&
         mod_e222_m117_eq(a->x, b->x, scratch);
}

bool e222_eq(e222 *a,
             e222 *b,
             mod_e222_m117_scratchpad *scratch) {
  e222_scale(a, scratch);
  e222_scale(b, scratch);

  return e222_scaled_eq(a, b, scratch);
}

void e222_from_edwards(const mod_e222_m117 x,
                       const mod_e222_m117 y,
                       e222 *p) {
  memcpy(p->x, x, sizeof(mod_e222_m117));
  memcpy(p->y, y, sizeof(mod_e222_m117));
  memcpy(p->z, mod_e222_m117_zero, sizeof(mod_e222_m117));
  mod_e222_m117_mul(x, y, p->t);
}

void e222_from_extended(const mod_e222_m117 x,
                        const mod_e222_m117 y,
                        const mod_e222_m117 t,
                        const mod_e222_m117 z,
                        e222 *p) {
  memcpy(p->x, x, sizeof(mod_e222_m117));
  memcpy(p->y, y, sizeof(mod_e222_m117));
  memcpy(p->t, t, sizeof(mod_e222_m117));
  memcpy(p->z, z, sizeof(mod_e222_m117));
}

void e222_copy(const e222 *from,
               e222 *to) {
  memcpy(to->x, from->x, sizeof(mod_e222_m117));
  memcpy(to->y, from->y, sizeof(mod_e222_m117));
  memcpy(to->t, from->t, sizeof(mod_e222_m117));
  memcpy(to->z, from->z, sizeof(mod_e222_m117));
}

void e222_add(const e222 *a,
              const e222 *b,
              e222 *c,
              e222_scratchpad *scratch) {
  /* Formula from
   * https://hyperelliptic.org/EFD/g1p/auto-twisted-extended.html#addition-add-2008-hwcd
   *
   * A = X1 * X2
   * B = Y1 * Y2
   * C = T1 * d * T2
   * D = Z1 * Z2
   * E = (X1 + Y1) * (X2 + Y2) - A - B
   * F = D - C
   * G = D + C
   * H = B - a * A
   * X3 = E * F
   * Y3 = G * H
   * T3 = E * H
   * Z3 = F * G
   *
   * Rewritten slightly to:
   *
   * A = X1 * X2
   * B = Y1 * Y2
   * C = T1 * d * T2
   * D = Z1 * Z2
   * S = X2 + Y2
   * E = (X1 + Y1) * S - A - B
   * H = B - A
   * F = D - C
   * G = D + C
   * X3 = E * F
   * Y3 = G * H
   * T3 = E * H
   * Z3 = F * G
   *
   * Manual register allocation produces the following assignments:
   *
   * r0 = A
   * r1 = B
   * r2 = C
   * r3 = D
   * r4 = S
   * r5 = E
   * r1.1 = H
   * r0.1 = F
   * r3.1 = G
   *
   * Final formula:
   *
   * r0 = X1 * X2
   * r1 = Y1 * Y2
   * r2 = T1 * d * T2
   * r3 = Z1 * Z2
   * r4 = X2 + Y2
   * r5 = (X1 + Y1) * r4 - r0 - r1
   * r1.1 = r1 - r0
   * r0.1 = r3 - r2
   * r3.1 = r3 + r2
   * X3 = r5 * r0.1
   * Y3 = r3.1 * r1.1
   * T3 = r5 * r1.1
   * Z3 = r0.1 * r3.1
   */

  /* r0 = X1 * X2 */
  mod_e222_m117_mul(a->x, b->x, scratch->r0);

  /* r1 = Y1 * Y2 */
  mod_e222_m117_mul(a->y, b->y, scratch->r1);

  /* r2 = T1 * d * T2 */
  mod_e222_m117_mul(a->t, b->t, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, edwards_d, scratch->r2);

  /* r3 = Z1 * Z2 */
  mod_e222_m117_mul(a->z, b->z, scratch->r3);

  /* r4 = X2 + Y2 */
  mod_e222_m117_add(b->x, b->y, scratch->r4);

  /* r5 = (X1 + Y1) * r4 - r0 - r1 */
  mod_e222_m117_add(a->x, a->y, scratch->r5);
  mod_e222_m117_mul(scratch->r5, scratch->r4, scratch->r5);
  mod_e222_m117_sub(scratch->r5, scratch->r0, scratch->r5);
  mod_e222_m117_sub(scratch->r5, scratch->r1, scratch->r5);

  /* r1.1 = r1 - r0 */
  mod_e222_m117_sub(scratch->r1, scratch->r0, scratch->r1);

  /* r0.1 = r3 - r2 */
  mod_e222_m117_sub(scratch->r0, scratch->r2, scratch->r0);

  /* r3.1 = r3 + r2 */
  mod_e222_m117_add(scratch->r0, scratch->r2, scratch->r0);

  /* X3 = r5 * r0.1 */
  mod_e222_m117_mul(scratch->r5, scratch->r0, c->x);

  /* Y3 = r3.1 * r1.1 */
  mod_e222_m117_mul(scratch->r3, scratch->r1, c->y);

  /* T3 = r5 * r1.1 */
  mod_e222_m117_mul(scratch->r5, scratch->r1, c->t);

  /* Z3 = r0.1 * r3.1 */
  mod_e222_m117_mul(scratch->r3, scratch->r0, c->t);
}

extern void e222_double(const e222 *a,
                        e222 *c,
                        e222_scratchpad *scratch) {
  /* Formula from
   * https://hyperelliptic.org/EFD/g1p/auto-twisted-extended.html#doubling-dbl-2008-hwcd
   *
   * A = X1^2
   * B = Y1^2
   * C = 2 * Z1^2
   * D = a * A
   * E = (X1 + Y1)^2 - A - B
   * G = D + B
   * F = G - C
   * H = D - B
   * X3 = E * F
   * Y3 = G * H
   * T3 = E * H
   * Z3 = F * G
   *
   * Rewritten slightly as
   *
   * A = X1^2
   * B = Y1^2
   * C = -2 * Z1^2
   * E = (X1 + Y1)^2 - A - B
   * G = A + B
   * F = G + C
   * H = A - B
   * X3 = E * F
   * Y3 = G * H
   * T3 = E * H
   * Z3 = F * G
   *
   * Manual register allocation produces the following substitutions:
   *
   * r0 = A
   * r1 = B
   * r2 = C
   * r3 = E
   * r4 = G
   * r2.1 = F
   * r0.1 = H
   *
   * Final formula:
   *
   * r0 = X1^2
   * r1 = Y1^2
   * r2 = -2 * Z1^2
   * r3 = (X1 + Y1)^2 - r0 - r1
   * r4 = r0 + r1
   * r2.1 = r4 + r2
   * r0.1 = r0 - r1
   * X3 = r3 * r2.1
   * Y3 = r4 * r0.1
   * T3 = r3 * r0.1
   * Z3 = r2.1 * r4
   */

  /* r0 = X1^2 */
  mod_e222_m117_square(a->x, scratch->r0);

  /* r1 = Y1^2 */
  mod_e222_m117_square(a->y, scratch->r1);

  /* r2 = Z1^2 * -2 */
  mod_e222_m117_square(a->z, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, -2, scratch->r2);

  /* r3 = (X1 + Y1)^2 - r0 - r1 */
  mod_e222_m117_add(a->x, a->y, scratch->r3);
  mod_e222_m117_square(scratch->r3, scratch->r3);
  mod_e222_m117_sub(scratch->r3, scratch->r0, scratch->r3);
  mod_e222_m117_sub(scratch->r3, scratch->r1, scratch->r3);

  /* r4 = r0 + r1 */
  mod_e222_m117_add(scratch->r0, scratch->r1, scratch->r4);

  /* r2.1 = r4 + r2,
   * r2 dead
   */
  mod_e222_m117_add(scratch->r2, scratch->r4, scratch->r2);

  /* r0.1 = r0 - r1,
   * r0, r1 dead
   */
  mod_e222_m117_sub(scratch->r0, scratch->r1, scratch->r0);

  /* X3 = r3 * r2.1 */
  mod_e222_m117_mul(scratch->r3, scratch->r2, c->x);

  /* Y3 = r4 * r0.1 */
  mod_e222_m117_mul(scratch->r4, scratch->r0, c->y);

  /* T3 = r3 * r0.1,
   * r3 dead
   */
  mod_e222_m117_mul(scratch->r3, scratch->r0, c->t);

  /* Z3 = r2.1 * r4,
   * r2.1, r4 dead
   */
  mod_e222_m117_mul(scratch->r2, scratch->r4, c->z);
}

extern void e222_mul_x(const e222 *p,
                       const mod_e222_m117 n,
                       mod_e222_m117 x);
extern void e222_mul(const e222 *p,
                     const mod_e222_m117 n,
                     e222 *out);

void e222_from_hash(const mod_e222_m117 r,
                    e222 *p,
                    e222_scratchpad *scratch) {
  /* Original formula from https://eprint.iacr.org/2013/325.pdf
   *
   * u = (1 - t) / (1 + t)
   * v = u^5 + (r^2 - 2) * u^3 + u
   * X = v.legendre * u
   * Y = (v.legendre * v)^((p+1)/4) * v.legendre *
   *     (u^2 + 1 / c^2).legendre
   * x = (c - 1) * s * X * (1 + X) / Y
   * y = (r * X - (1 + X)^2) / (r * X + (1 + X)^2)
   *
   * Rewritten slightly as
   *
   * u = (1 - t) / (1 + t)
   * r = elligatorR()
   * v = (u^4 + (r^2 - 2) * u^2 + 1) * u
   * c = elligatorC()
   * Y = (v.legendre * v).sqrt * v.legendre *
   *     (u^2 + 1 / c^2).legendre
   * X = v.legendre * u
   * x = (c - 1) * elligatorS() * X * (1 + X) / Y
   * y = (r * X - (1 + X)^2) / (r * X + (1 + X)^2)
   *
   * Manual common subexpression elimination produces the following:
   *
   * F = 1 + t
   * U = (1 - t) / F
   * U2 = U^2
   * C = elligatorC()
   * H = U2 + (1 / C^2)
   * l2 = H.legendre
   * R = elligatorR
   * G = (R^2 - 2) * U2
   * U4 = U2^2
   * V = (U4 + G + 1) * U
   * l1 = V.legendre
   * Y = (l1 * V).sqrt * l1 * l2
   * X = l1 * U
   * I = 1 + X
   * x = (C - 1) * elligatorS() * X * I / Y
   * J = I^2
   * K = R * X
   * L = K + J
   * y = (K - J) / L
   *
   * Manual register allocation produces the following assignments:
   *
   * r0 = F
   * r1 = U
   * r0.1 = U2
   * r2 = C
   * r3 = H
   * r3.1 = R
   * r4 = G
   * r0.2 = U4
   * r0.3 = V
   * r4.1 = Y
   * r1.1 = X
   * r0.4 = I
   * r2.1 = x
   * r0.5 = J
   * r3.2 = K
   * r1.2 = L
   * r3.3 = y
   *
   * Final formula:
   *
   * r0 = 1 + t
   * r1 = (1 - t) / r0
   * r0.1 = r1^2
   * r2 = elligatorC()
   * r3 = r0.1 + (1 / r2^2)
   * l2 = r3.legendre
   * r3.1 = elligatorR
   * r4 = (r3.1^2 - 2) * r0.1
   * r0.2 = r0.1^2
   * r0.3 = (r0.2 + r4 + 1) * r1
   * l1 = r0.3.legendre
   * r4.1 = (l1 * r0.3).sqrt * l1 * l2
   * r1.1 = l1 * r1
   * r0.4 = 1 + r1.1
   * r2.1 = (r2 - 1) * elligatorS() * r1.1 * r0.4 / r4.1
   * r0.5 = r0.4^2
   * r3.2 = r3.1 * r1.1
   * r1.2 = r3.2 + r0.5
   * r3.3 = (r3.2 - r0.5) / r1.2
   * x = r2.1
   * y = r3.3
   */

  /* r0 = 1 + t */
  mod_e222_m117_add_small(p->t, 1, scratch->r0);

  /* r1 = (1 - t) / r0 */
  mod_e222_m117_sub_small(p->t, 1, scratch->r1);
  mod_e222_m117_neg(scratch->r1, scratch->r1, &(stratch->inner));
  mod_e222_m117_div(scratch->r1, scratch->r0, scratch->r1);

  /* r0.1 = r1^2 */
  mod_e222_m117_square(scratch->r0, scratch->r1);

  /* r2 = elligatorC() */
  mod_e222_m117_copy(elligator_c, scratch->r2);

  /* r3 = r0.1 + (1 / r2^2) */
  mod_e222_m117_square(scratch->r2, scratch->r3);
  mod_e222_m117_inv(scratch->r3, scratch->r3, &(stratch->inner));
  mod_e222_m117_add(scratch->r3, scratch->r0, scratch->r3);

  /* l2 = r3.legendre */
  const int l2 = mod_e222_m117_leg(scratch->r3, &(scratch->inner));

  /* r3.1 = elligatorR */
  mod_e222_m117_copy(elligator_r, scratch->r3);

  /* r4 = (r3.1^2 - 2) * r0.1 */
  mod_e222_m117_square(scratch->r3, scratch->r4);
  mod_e222_m117_sub_small(scratch->r4, 2, scratch->r4);
  mod_e222_m117_mul(scratch->r4, scratch->r0, scratch->r4);

  /* r0.2 = r0.1^2 */
  mod_e222_m117_square(scratch->r0, scratch->r0);

  /* r0.3 = (r0.2 + r4 + 1) * r1 */
  mod_e222_m117_add(scratch->r0, scratch->r4, scratch->r0);
  mod_e222_m117_add_small(scratch->r0, 1, scratch->r0);
  mod_e222_m117_mul(scratch->r0, scratch->r1, scratch->r0);

  /* l1 = r0.3.legendre */
  const int l1 = mod_e222_m117_leg(scratch->r0, &(scratch->inner));

  /* r4.1 = (l1 * r0.3).sqrt * l1 * l2 */
  mod_e222_m117_mul_small(scratch->r0, l1, scratch->r4);
  mod_e222_m117_sqrt(scratch->r4, scratch->r4, &(scratch->inner));
  mod_e222_m117_mul_small(scratch->r4, l1 * l2, scratch->r4);

  /* r1.1 = l1 * r1 */
  mod_e222_m117_mul_small(scratch->r1, l1, scratch->r1);

  /* r0.4 = 1 + r1.1 */
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r0);

  /* r2.1 = (r2 - 1) * elligatorS() * r1.1 * r0.4 / r4.1 */
  mod_e222_m117_sub_small(scratch->r2, 1, scratch->r2);
  mod_e222_m117_mul(scratch->r2, elligator_s, scratch->r2);
  mod_e222_m117_mul(scratch->r2, scratch->r1, scratch->r2);
  mod_e222_m117_mul(scratch->r2, scratch->r0, scratch->r2);
  mod_e222_m117_div(scratch->r2, scratch->r4, scratch->r2);

  /* r0.5 = r0.4^2 */
  mod_e222_m117_square(scratch->r0, scratch->r0);

  /* r3.2 = r3.1 * r1.1 */
  mod_e222_m117_mul(scratch->r3, scratch->r1, scratch->r3);

  /* r1.2 = r3.2 + r0.5 */
  mod_e222_m117_add(scratch->r0, scratch->r3, scratch->r1);

  /* r3.3 = (r3.2 - r0.5) / r1.2 */
  mod_e222_m117_sub(scratch->r3, scratch->r0, scratch->r3);
  mod_e222_m117_sub(scratch->r3, scratch->r1, scratch->r3);

  /* x = r2.1 */
  /* y = r3.3 */
  e222_from_edwards(scratch->r2, scratch->r3, p);
}

void e222_to_hash(const e222 *p,
                  mod_e222_m117 r,
                  e222_scratchpad *scratch) {
  /* Formula from https://eprint.iacr.org/2013/325.pdf
   *
   * e = (y - 1) / (2 * (y + 1))
   * X = ((1 + e * r)^2 - 1)^((p+1)/4) - (1 + e * r)
   * z = ((c - 1) * s * X * (1 + X) * x * (X^2 + (1 / c^2)).legendre
   * u = z * X
   * t = (1 - u) / (1 + u)
   *
   * Rewritten slightly as, and taking absolute value of t, as t
   * and -t are equivalent under the inverse map:
   *
   * e = (y - 1) / (2 * (y + 1))
   * r = elligatorR()
   * X = ((1 + e * r)^2 - 1).sqrt - (1 + e * r)
   * c = elligatorC()
   * z = ((c - 1) * elligatorS() * X * (1 + X) * x *
   *     (X^2 + (1 / c^2))).legendre
   * u = z * X
   * t = ((1 - u) / (1 + u)).abs
   *
   * Manual common subexpression elimination produces the following:
   *
   * G = 2 * (y + 1)
   * E = (y - 1) / G
   * H = 1 + E * elligatorR()
   * X = (H^2 - 1).sqrt - H
   * C = elligatorC()
   * I = C - 1
   * J = 1 + X
   * K = X^2
   * Z = I * elligatorS * X * J * x * (K + (1 / C^2))
   * l1 = Z.legendre
   * U = l1 * X
   * L = 1 + U
   * t = ((1 - U) / L).abs
   *
   * Manual register allocation produces the following assignments:
   *
   * r0 = G
   * r1 = E
   * r0.1 = H
   * r1.1 = X
   * r0.2 = C
   * r2 = I
   * r3 = J
   * r4 = K
   * r0.3 = Z
   * r0.4 = U
   * r1.2 = L
   *
   * Final formula:
   *
   * r0 = 2 * (y + 1)
   * r1 = (y - 1) / r0
   * r0.1 = 1 + r1 * elligatorR()
   * r1.1 = (r0.1^2 - 1).sqrt - r0.1
   * r0.2 = elligatorC()
   * r2 = r0.2 - 1
   * r3 = 1 + r1.1
   * r4 = r1.1^2
   * r0.3 = r2 * elligatorS() * r1.1 * r3 * x * (r4 + (1 / r0.2^2))
   * l1 = r0.3.legendre
   * r0.4 = l1 * r1.1
   * r1.2 = 1 + r0.4
   * r0.5 = ((1 - r0.4) / r1.2).abs
   * t = r0.5
   */

  e222_scale(p, scratch);

  /* r0 = 2 * (y + 1) */
  mod_e222_m117_add_small(p->y, 1, scratch->r0);
  mod_e222_m117_mul_small(scratch->r0, 2, scratch->r0);

  /* r1 = (y - 1) / r0 */
  mod_e222_m117_sub_small(p->y, 1, scratch->r1);
  mod_e222_m117_div(scratch->r1, scratch->r0, scratch->r1);

  /* r0.1 = 1 + r1 * elligatorR() */
  mod_e222_m117_mul_small(scratch->r1, elligator_r, scratch->r0);
  mod_e222_m117_add_small(scratch->r0, 1, scratch->r0);

  /* r1.1 = (r0.1^2 - 1).sqrt - r0.1 */
  mod_e222_m117_square(scratch->r0, scratch->r1);
  mod_e222_m117_sub_small(scratch->r0, 1, scratch->r1);
  mod_e222_m117_sqrt(scratch->r1, scratch->r1, &(scratch->inner));
  mod_e222_m117_sub_small(scratch->r1, 1, scratch->r1);

  /* r0.2 = elligatorC() */
  mod_e222_m117_copy(elligator_c, scratch->r0);

  /* r2 = r0.2 - 1 */
  mod_e222_m117_sub_small(scratch->r0, 1, scratch->r2);

  /* r3 = 1 + r1.1 */
  mod_e222_m117_sub_small(scratch->r1, 1, scratch->r3);

  /* r4 = r1.1^2 */
  mod_e222_m117_square(scratch->r1, scratch->r4);

  /* r0.3 = r2 * elligatorS() * r1.1 * r3 * x * (r4 + (1 / r0.2^2)) */
  mod_e222_m117_square(scratch->r0, scratch->r0);
  mod_e222_m117_inv(scratch->r0, scratch->r0, &(scratch->inner));
  mod_e222_m117_add(scratch->r0, scratch->r4, scratch->r0);
  mod_e222_m117_mul(p->x, scratch->r0, scratch->r0);
  mod_e222_m117_mul(scratch->r3, scratch->r0, scratch->r0);
  mod_e222_m117_mul(scratch->r1, scratch->r0, scratch->r0);
  mod_e222_m117_mul(elligator_s, scratch->r0, scratch->r0);
  mod_e222_m117_mul(scratch->r2, scratch->r0, scratch->r0);

  /* l1 = r0.3.legendre */
  const int l1 = mod_e222_m117_leg(scratch->r0, &(scratch->inner));

  /* r0.4 = l1 * r1.1 */
  mod_e222_m117_mul_small(scratch->r1, l1, scratch->r1);

  /* r1.2 = 1 + r0.4 */
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r0);

  /* r0.5 = ((1 - r0.4) / r1.2).abs */
  mod_e222_m117_neg(scratch->r0, scratch->r0, &(scratch->inner));
  mod_e222_m117_add_small(scratch->r0, 1, scratch->r0);
  mod_e222_m117_mul(scratch->r0, scratch->r1, scratch->r0);
  mod_e222_m117_abs(scratch->r0, r, &(scratch->inner));
}

bool e222_can_hash(const e222 *p,
                   e222_scratchpad *scratch) {
  /* Criteria from https://eprint.iacr.org/2013/325.pdf
   *
   * e = (y - 1) / (2 * (y + 1))
   *
   * y + 1 != 0
   * ((1 + e * r)^2 - 1).legendre == 1
   * if e * r == -2 then 2 * s * (c - 1) * c.legendre / r
   *
   * Manual common subexpression elimination produces the following:
   *
   * R = elligatorR
   * F = y + 1
   * G = 2 * F
   * E = (y - 1) / G
   * H = E * R
   * I = (1 + H)^2 - 1
   * C = elligatorC
   * l1 = C.legendre
   * J = 2 * s * (C - 1) * l1 / R
   *
   * F != 0
   * I.legendre == 1
   * if H == -2 then x == J
   *
   * Manual register allocation produces the following assignments:
   *
   * r0 = R
   * r1 = F
   * r2 = G
   * r3 = E
   * r2.1 = H
   * r3.1 = I
   * r4 = C
   * r4.1 = J
   *
   * Final formula:
   *
   * r0 = elligatorR
   * r1 = y + 1
   * r2 = 2 * r1
   * r3 = (y - 1) / r2
   * r2.1 = r3 * r0
   * r3.1 = (1 + r2.1)^2 - 1
   * r4 = elligatorC
   * l1 = r4.legendre
   * r4.1 = 2 * s * (r4 - 1) * l1 / r0
   *
   * r1 != 0
   * r3.1.legendre == 1
   * if r2.1 == -2 then x == r4.1
   */

  e222_scale(p, scratch);

  /* r0 = elligatorR */
  mod_e222_m117_copy(elligator_r, scratch->r0);

  /* r1 = y + 1 */
  mod_e222_m117_add_small(p->y, 1, scratch->r1);

  /* r2 = 2 * r1 */
  mod_e222_m117_add_small(scratch->r1, 2, scratch->r2);

  /* r3 = (y - 1) / r2 */
  mod_e222_m117_sub_small(p->y, 1, scratch->r3);
  mod_e222_m117_div(scratch->r3, scratch->r2, scratch->r3);

  /* r2.1 = r3 * r0 */
  mod_e222_m117_div(scratch->r3, scratch->r0, scratch->r2);

  /* r3.1 = (1 + r2.1)^2 - 1 */
  mod_e222_m117_add_small(scratch->r2, 1, scratch->r3);
  mod_e222_m117_square(scratch->r3, scratch->r3);
  mod_e222_m117_sub_small(scratch->r3, 1, scratch->r3);

  /* r4 = elligatorC */
  mod_e222_m117_copy(elligator_c, scratch->r4);

  /* l1 = r4.legendre */
  const int l1 = mod_e222_m117_leg(scratch->r4, &(scratch->inner));

  /* r4.1 = 2 * s * (r4 - 1) * l1 / r0 */
  mod_e222_m117_add_small(scratch->r4, 1, scratch->r4);
  mod_e222_m117_mul_small(scratch->r4, 2 * l1, scratch->r4);
  mod_e222_m117_mul(scratch->r4, elligator_s, scratch->r4);
  mod_e222_m117_div(scratch->r4, scratch->r0, scratch->r4);

  /* r1 != 0 */
  /* r3.1.legendre == 1 */
  /* if r2.1 == -2 then x == r4.1 */
  mod_e222_m117_set(scratch->r0, -2);

  return mod_e222_m117_is_zero(scratch->r1, &(scratch->inner)) &&
         mod_e222_m117_leg(scratch->r3, &(scratch->inner)) == 1 &&
         (!mod_e222_m117_eq(scratch->r2, scratch->r0, &(scratch->inner)) ||
          mod_e222_m117_eq(p->x, scratch->r4, &(scratch->inner)));
}

bool e222_decaf_eq(const e222 *a,
                   const e222 *b,
                   e222_scratchpad *scratch) {
  mod_e222_m117_mul(a->x, b->y, scratch->r0);
  mod_e222_m117_mul(a->y, b->x, scratch->r1);

  return mod_e222_m117_eq(scratch->r0, scratch->r1, &(scratch->inner));
}

bool e222_decompress(const mod_e222_m117 s,
                     e222 *p,
                     e222_scratchpad *scratch) {
  /* Formula from https://eprint.iacr.org/2015/673.pdf
   *
   * Reject unless s.signum == 1
   *
   * X = 2 * s
   * Z = 1 + (a * s^2)
   * U = Z^2 - (4 * d * s^2)
   * V = 1 / (U * s^2).sqrt if (U * s^2).legendre == 1
   *     0 if (U * s^2).legendre == 0
   *     reject otherwise
   * V = -V if (U * V).signum == -1
   * W = V * s * (2 - Z)
   * W = W + 1 if s == 0
   * Y = W * Z
   * T = W * X
   *
   * Set a = 1 and rewritten as:
   *
   * Reject unless s.signum == 1
   *
   * X = 2 * s
   * SS = s^2
   * Z = 1 + SS
   * ZZ = Z^2
   * U = ZZ - (4 * d * SS)
   * C = U * SS
   * Reject if s.signum == -1 or C.legendre == -1
   * V = C.invsqrt * C.legendre
   * E = U * V
   * F = V * E.signum
   * H = 2 - Z
   * W = F * s * H
   * G = W + s.isZero
   * Y = G * Z
   * T = G * X
   *
   * Manual register allocation produces the following substitutions:
   *
   * r0 = SS
   * r1 = U
   * r2 = ZZ
   * r0.1 = C
   * r0.2 = V
   * r1.1 = E
   * r0.3 = F
   * r1.2 = H
   * r0.4 = W
   * r0.5 = G
   *
   * Final formula:
   *
   * X = 2 * s
   * r0 = s^2
   * Z = 1 + r0
   * r2 = Z^2
   * r1 = r2 - (4 * d * r0)
   * r0.1 = r1 * r0
   * i0 = r0.1.legendre
   * Reject if s.signum == -1 or i0 == -1
   * r0.2 = r0.1.invsqrt * i0
   * r1.1 = r1 * r0.2
   * r0.3 = r0.2 * r1.1.signum
   * r1.2 = 2 - Z
   * r0.4 = r0.3 * s * r1.2
   * r0.5 = r0.4 + s.isZero
   * Y = r0.5 * Z
   * T = r0.5 * X
   */

  /* X = 2 * s */
  mod_e222_m117_mul_small(s, 2, p->x);

  /* r0 = s^2 */
  mod_e222_m117_square(s, scratch->r0);

  /* Z = 1 + r0 */
  mod_e222_m117_mul_small(scratch->r0, 1, p->z);

  /* r2 = Z^2 */
  mod_e222_m117_square(p->z, scratch->r1);

  /* r1 = r2 - (4 * d * r0) */
  mod_e222_m117_mul_small(scratch->r0, -4 * edwards_d, scratch->r1);
  mod_e222_m117_add(scratch->r1, scratch->r2, scratch->r1);

  /* r0.1 = r1 * r0 */
  mod_e222_m117_add(scratch->r0, scratch->r1, scratch->r0);

  /* i0 = r0.1.legendre */
  const int i0 = mod_e222_m117_leg(scratch->r0, &(scratch->inner));

    /* Reject if s.signum == -1 or i0 == -1 */
  if (mod_e222_m117_signum(s, &(scratch->inner)) == -1 || i0 == -1) {
    return false;
  }

  /* r0.2 = r0.1.invsqrt * i0 */
  mod_e222_m117_invsqrt(scratch->r0, scratch->r0, &(scratch->inner));
  mod_e222_m117_mul_small(scratch->r0, i0, scratch->r0);

  /* r1.1 = r1 * r0.2 */
  mod_e222_m117_mul(scratch->r1, scratch->r0, scratch->r1);

  const int i1 = mod_e222_m117_signum(scratch->r1, &(scratch->inner));

  /* r0.3 = r0.2 * r1.1.signum */
  mod_e222_m117_mul_small(scratch->r0, i1, scratch->r2);

  /* r1.2 = 2 - Z */
  mod_e222_m117_set(scratch->r1, 2);
  mod_e222_m117_add(scratch->r1, p->z, scratch->r1);

  /* r0.4 = r0.3 * s * r1.2 */
  mod_e222_m117_mul(scratch->r0, scratch->r1, scratch->r0);
  mod_e222_m117_mul(scratch->r0, s, scratch->r0);

  const int i2 = mod_e222_m117_is_zero(s, &(scratch->inner));

  /* r0.5 = r0.4 + s.isZero */
  mod_e222_m117_add_small(scratch->r0, i2, scratch->r0);

  /* Y = r0.5 * Z */
  mod_e222_m117_mul(scratch->r0, p->z, y);

  /* T = r0.5 * X */
  mod_e222_m117_mul(scratch->r0, p->x, t);
}

void e222_compress(const e222 *p,
                   mod_e222_m117 s,
                   e222_scratchpad *scratch) {
  /* Formula from https://eprint.iacr.org/2015/673.pdf
   *
   * R = 1 / sqrt((a - d) * (Z + Y) * (Z - Y))
   * U = (a - d) * R
   * R = if (-2 * U * Z) negative then -R else R
   * S = abs(u * ((R * ((a * Z * X) - (d * Y * T))) + Y) / a)
   *
   * Set a = 1, slight rewrite to
   *
   * R = ((1 - d) * (Z + Y) * (Z - Y)).invsqrt
   * U = (1 - d) * R
   * Q = R * (-2 * U * Z).signum
   * S = abs(U * ((Q * ((Z * X) - (d * Y * T))) + Y))
   *
   * Manual common subexpression elimination produces the following:
   *
   * C = 1 - d
   * E = Z + Y
   * R = (C * E * (Z - Y)).invsqrt
   * U = C * R
   * G = -2 * U * Z
   * Q = R * G.signum
   * F = d * Y * T
   * S = abs(U * ((Q * ((Z * X) - F)) + Y))
   *
   * We will assume T requires a register assignment of its own.
   * Manual register allocation then produces the following
   * assignments:
   *
   * r0 = T
   * i0 = C
   * r1 = E
   * r2 = R
   * r1.1 = U
   * r3 = G
   * r2.1 = Q
   * r3.1 = F
   *
   * Final formula:
   *
   * i0 = 1 - d
   * r1 = Z + Y
   * r2 = (i0 * r1 * (Z - Y)).invsqrt
   * r1.1 = i0 * r2
   * r3 = -2 * r1.1 * Z
   * r2.1 = r2 * r3.signum
   * r3.1 = d * Y * T
   * r0.1 = abs(r1.1 * ((r2.1 * ((Z * X) - r3.1)) + Y))
   */

  /* i0 = 1 - d */
  const int i0 = 1 - d;

  /* r1 = Z + Y */
  mod_e222_m117_add(p->z, p->y, scratch->r1);

  /* r2 = (i0 * r1 * (Z - Y)).invsqrt */
  mod_e222_m117_sub(p->z, p->y, scratch->r2);
  mod_e222_m117_mul(scratch->r2, scratch->r1, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, i0, scratch->r2);
  mod_e222_m117_invsqrt(scratch->r2, scratch->r2, &(scratch->inner));

  /* r1.1 = i0 * r2 */
  mod_e222_m117_mul_small(scratch->r1, i0, scratch->r2);

  /* r3 = -2 * r1.1 * Z */
  mod_e222_m117_mul(scratch->r1, p->z, scratch->r3);
  mod_e222_m117_mul_small(scratch->r3, -2, scratch->r3);

  const int i1 = mod_e222_m117_signum(scratch->r3, &(scratch->inner));

  /* r2.1 = r2 * r3.signum */
  mod_e222_m117_mul_small(scratch->r2, i1, scratch->r2);

  /* r3.1 = d * Y * T */
  mod_e222_m117_mul(p->t, p->y, scratch->r3);
  mod_e222_m117_mul_small(scratch->r3, edwards_d, scratch->r3);

  /* S = abs(r1.1 * ((r2.1 * ((Z * X) - r3.1)) + Y)) */
  mod_e222_m117_mul(p->z, p->x, s);
  mod_e222_m117_sub(s, scratch->r3, s);
  mod_e222_m117_mul(s, scratch->r2, s);
  mod_e222_m117_add(s, y, s);
  mod_e222_m117_mul(s, scratch->r1, s);
  mod_e222_m117_abs(s, &(scratch->inner));
}

void e222_decaf_from_hash(const mod_e222_m117 r,
                          e222 *p,
                          e222_scratchpad *scratch) {
  /* Formula from https://eprint.iacr.org/2015/673.pdf
   *
   * n = nonresidue
   * r = n * r0^2
   * D = ((d * r) + a - d) * ((d * r) - (a * r) - d)
   * N = (r + 1) * (a - (2 * d))
   * c, e = if N * D is square
   *           then (1, 1 / sqrt (N * D)
   *           else (-1, (n * r0) / sqrt (n * N * D)
   * s = c * abs (N * e)
   * t = (-c * N * (r - 1) * ((a - (2 * d)) * e)^2) - 1
   *
   * Then apply the maps from the paper:
   *
   * x = (2 * s) / (1 + (a * s^2))
   * y = -(1 - (a * s^2)) / t
   *
   * Note that we can safely branch, because we are decoding a
   * hash; therefore, any attacker will know in advance whether
   * the branch is taken.
   *
   * Rewritten, renaming "r" to "q", "r0" to "r", dropping c,
   * setting a = 1:
   *
   * n = nonresidue
   * Q = n * R^2
   * D = ((d * Q) + 1 - d) * ((d * Q) - Q - d)
   * N = (Q + 1) * (1 - (2 * d))
   * c = (N * D).legendre
   * E = if c == 1
   *        then (N * D).invsqrt
   *        else (n * R) * (n * N * D).invsqrt
   * S = c * (N * E).abs
   * T = (-c * N * (Q - 1) * ((1 - (2 * d)) * E)^2) - 1
   * x = (2 * S) / (1 + S^2)
   * y = -(1 - S^2) / T
   *
   * Manual common subexpression elimination produces the following:
   *
   * n = nonresidue
   * Q = n * R^2
   * F = d * Q
   * G = F - Q - d
   * D = (F + 1 - d) * G
   * N = (Q + 1) * (1 - (2 * d))
   * H = N * D
   * c = H.legendre
   * E = if c == 1
   *        then H.invsqrt
   *        else {
   *          I = (n * H).invsqrt
   *          (n * Q) * I
   *        }
   * S = c * (N * E).abs
   * K = Q - 1
   * T = (-c * N * K * ((1 - (2 * d)) * E)^2) - 1
   * SS = S^2
   * J = 1 + SS
   * X = (2 * S) / J
   * Y = -(1 - SS) / T
   *
   * Manual register allocation produces the following assignments:
   *
   * Note: this assignment deliberately uses r3 for E and S over r0,
   * because decompression does not use r3.
   *
   * r0 = Q
   * r1 = F
   * r2 = G
   * r1.1 = D
   * r2.1 = N
   * r1.2 = H
   * r1.3 = I
   * r3 = E
   * r1.4 = S
   * r0.1 = K
   * r3.1 = T
   * r0.2 = SS
   * r2.2 = J
   * r1.5 = X
   * r0.3 = Y
   *
   * Final formula:
   *
   * n = nonresidue
   * r0 = n * R^2
   * r1 = d * r0
   * r2 = r1 - r0 - d
   * r1.1 = (r1 + 1 - d) * r2
   * r2.1 = (r0 + 1) * (1 - (2 * d))
   * r1.2 = r2.1 * r1.1
   * c = r1.2.legendre
   * r3 = if c == 1
   *        then r1.2.invsqrt
   *        else {
   *          r1.3 = (n * r1.2).invsqrt
   *          (n * r0) * r1.3
   *        }
   * r1.4 = c * (r2.1 * r3).abs
   * r0.1 = r0 - 1
   * r3.1 = (-c * r2.1 * r0.1 * ((1 - (2 * d)) * r3)^2) - 1
   * r0.2 = r1.4^2
   * r2.2 = 1 + r0.2
   * r1.5 = (2 * r1.4) / r2.2
   * r0.3 = -(1 - r0.2) / r3.1
   * X = r1.5
   * Y = r0.3
   */

  /* r0 = nonresidue * R^2 */
  mod_e222_m117_square(r, scratch->r0);
  mod_e222_m117_mul_small(scratch->r0, 2, scratch->r0);

  /* r1 = d * r0 */
  mod_e222_m117_mul_small(scratch->r0, edwards_d, scratch->r1);

  /* r2 = r1 - r0 - d */
  mod_e222_m117_sub(scratch->r1, scratch->r0, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, edwards_d, scratch->r2);

  /* r1.1 = (r1 + 1 - d) * r2 */
  mod_e222_m117_mul_small(scratch->r1, (1 - edwards_d), scratch->r1);
  mod_e222_m117_mul(scratch->r1, scratch->r2, scratch->r1);

  /* r2.1 = (r0 + 1) * (1 - (2 * d)) */
  mod_e222_m117_add_small(scratch->r0, 1, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, (1 - (2 * edwards_d)), scratch->r2);

  /* r1.2 = r2.1 * r1.1 */
  mod_e222_m117_mul(scratch->r1, scratch->r2, scratch->r1);

  /* c = r1.2.legendre */
  const int c = mod_e222_m117_leg(scratch->r1, &(scratch->inner));

  /* r3 = if r1.2.legendre == 1
   *         then r1.2.invsqrt
   *         else (nonresidue * R) * (nonresidue * r1.2).invsqrt
   */
  if (c == 1) {
    mod_e222_m117_invsqrt(scratch->r1, scratch->r3, &(scratch->inner));
  } else {
    mod_e222_m117_mul_small(scratch->r1, 2, scratch->r1);
    mod_e222_m117_invsqrt(scratch->r1, scratch->r1, &(scratch->inner));
    mod_e222_m117_mul_small(r, 2, scratch->r3);
    mod_e222_m117_mul(scratch->r3, scratch->r1, scratch->r3);
  }

  /* r1.4 = c * (r2.1 * r3).abs */
  mod_e222_m117_mul(scratch->r3, scratch->r2, scratch->r1);
  mod_e222_m117_abs(scratch->r1, scratch->r1, &(scratch->inner));
  mod_e222_m117_mul_small(scratch->r1, c, scratch->r1);

  /* r0.1 = r0 - 1 */
  mod_e222_m117_sub_small(scratch->r0, 1, scratch->r0);

  /* r3.1 = (-c * r2.1 * r0.1 * ((1 - (2 * d)) * r3)^2) - 1 */
  mod_e222_m117_mul_small(scratch->r3, (1 - (2 * edwards_d)), scratch->r3);
  mod_e222_m117_square(scratch->r3, scratch->r3);
  mod_e222_m117_mul(scratch->r3, scratch->r0, scratch->r3);
  mod_e222_m117_mul(scratch->r3, scratch->r2, scratch->r3);
  mod_e222_m117_mul_small(scratch->r3, -c, scratch->r3);
  mod_e222_m117_sub_small(scratch->r3, 1, scratch->r3);

  /* r0.2 = r1.4^2 */
  mod_e222_m117_square(scratch->r1, scratch->r0);

  /* r2.2 = 1 + r0.2 */
  mod_e222_m117_add_small(scratch->r2, 1, scratch->r0);

  /* r1.5 = (2 * r1.4) / r2.2 */
  mod_e222_m117_mul_small(scratch->r1, 2, scratch->r1);
  mod_e222_m117_div(scratch->r1, scratch->r2, scratch->r1);

  /* r0.3 = -(1 - r0.2) / r3.1 */
  mod_e222_m117_sub_small(scratch->r0, 1, scratch->r0);
  mod_e222_m117_div(scratch->r0, scratch->r3, scratch->r0);

  /* X = r1.5 */
  /* Y = r0.3 */
  e222_from_edwards(scratch->r1, scratch->r0, p);
}

void e222_decaf_to_hash(const e222 *p,
                        const mod_e222_m117 r,
                        e222_scratchpad *scratch) {
  /* Formula from https://eprint.iacr.org/2015/673.pdf
   * (This hashes Jacobi quartic points)
   *
   * n = nonresidue
   * c = s.signum
   * r = ((((2 * d) - a) * s^2) + (c * (t + 1))) /
   *     ((((2 * d) - a) * s^2) - (c * (t + 1)))
   * r0 = sqrt (r / n)
   * (r / n must be a square)
   *
   * Formula for Jacobi quartic points from the same source:
   *
   * s = (1 + (sqrt (1 - (a * x^2)))) / (a * x)
   * t = (2 * s * (sqrt (1 - (a * x^2)))) / (x * y)
   *
   * Combined, "r" renamed to "Q", "r0" renamed to "R", set a = 1:
   *
   * n = nonresidue
   * S = (1 + (sqrt (1 - x^2))) / x
   * T = (2 * S * (sqrt (1 - x^2))) / (x * y)
   * c = S.signum
   * Q = ((((2 * d) - 1) * S^2) + (c * (T + 1))) /
   *     ((((2 * d) - 1) * S^2) - (c * (T + 1)))
   * R = sqrt (Q / n)
   *
   * Manual common subexpression elimination produces the following:
   *
   * X = edwardsX
   * E = sqrt (1 - X^2)
   * S = (1 + E) / X
   * F = X * Y
   * T = (2 * S * E) / F
   * H = S.signum * (T + 1)
   * G = ((2 * d) - 1) * S^2
   * I = (G - H) * nonresidue
   * Q = (G + H) / I
   * R = sqrt (Q)
   *
   * Manual register allocation produces the following assignments:
   *
   * r0 = X
   * r1 = E
   * r2 = S
   * r0.1 = F
   * r1.1 = T
   * r1.2 = H
   * r2.1 = G
   * r0.2 = I
   * r2.2 = Q
   * r2.3 = R
   *
   * Final formula
   *
   * r0 = edwardsX
   * r1 = sqrt (1 - r0^2)
   * r2 = (1 + r1) / r0
   * r0.1 = r0 * Y
   * r1.1 = (2 * r2 * r1) / r0.1
   * r1.2 = c * (r1.1 + 1)
   * r2.1 = ((2 * d) - 1) * r2^2
   * r0.2 = (r2.1 - r1.2) * nonresidue
   * r2.2 = sqrt (r2.1 + r1.2) / r0.2
   * R = r2.2
   */

  e222_scale(p, scratch);

  /* r0 = edwardsX */
  mod_e222_m117_copy(p->x, scratch->r0);

  /* r1 = sqrt (1 - r0^2) */
  mod_e222_m117_square(scratch->r0, scratch->r1);
  mod_e222_m117_neg(scratch->r1, scratch->r1);
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r1);
  mod_e222_m117_sqrt(scratch->r1, scratch->r1);

  /* r2 = (1 + r1) / r0 */
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r2);
  mod_e222_m117_div(scratch->r2, scratch->r0, scratch->r2);

  /* r0.1 = r0 * Y */
  mod_e222_m117_mul(scratch->r0, p->y, scratch->r0);

  /* r1.1 = (2 * r2 * r1) / r0.1 */
  mod_e222_m117_mul(scratch->r2, scratch->r1, scratch->r1);
  mod_e222_m117_add_small(scratch->r1, 2, scratch->r1);
  mod_e222_m117_div(scratch->r1, scratch->r0, scratch->r1);

  const int i0 = mod_e222_m117_signum(scratch->r2, scratch);

  /* r1.2 = r2.signum * (r1.1 + 1) */
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r1);
  mod_e222_m117_mul_small(scratch->r1, i0, scratch->r1);

  /* r2.1 = ((2 * d) - 1) * r2^2 */
  mod_e222_m117_square(scratch->r2, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, (2 * edwards_d) - 1, scratch->r2);

  /* r0.2 = (r2.1 - r1.2) * nonresidue */
  mod_e222_m117_sub(scratch->r2, scratch->r1, scratch->r0);
  mod_e222_m117_mul_small(scratch->r0, 2, scratch->r0);

  /* r2.2 = sqrt ((r2.1 + r1.2) / r0.2) */
  mod_e222_m117_add(scratch->r2, scratch->r1, scratch->r2);
  mod_e222_m117_div(scratch->r2, scratch->r0, scratch->r2);
  mod_e222_m117_sqrt(scratch->r2, r);
}

bool e222_decaf_can_hash(const e222 *p,
                         e222_scratchpad *scratch) {
  /* Formula derived from encodeHash:
   *
   * n = nonresidue
   * X = edwardsX
   * E = sqrt (1 - X^2)
   * S = (1 + E) / X
   * F = X * Y
   * T = (2 * S * E) / F
   * H = S.signum * (T + 1)
   * G = ((2 * d) - 1) * S^2
   * I = (G - H)
   * Q = (G + H) / I
   * R = sqrt (Q / n)
   *
   * Q / n must be a square, therefore, use final formula for
   * encoding, with a modification:
   *
   * n = nonresidue
   * r0 = edwardsX
   * r1 = sqrt (1 - r0^2)
   * r2 = (1 + r1) / r0
   * r0.1 = r0 * Y
   * r1.1 = (2 * r2 * r1) / r0.1
   * r1.2 = c * (r1.1 + 1)
   * r2.1 = ((2 * d) - 1) * r2^2
   * r0.2 = (r2.1 - r1.2) * nonresidue
   * r2.2 = (r2.1 + r1.2) / r0.2
   *
   * r2.3.legendre == 1
   */

  e222_scale(p, scratch);

  /* r0 = edwardsX */
  mod_e222_m117_copy(p->x, scratch->r0);

  /* r1 = sqrt (1 - r0^2) */
  mod_e222_m117_square(scratch->r0, scratch->r1);
  mod_e222_m117_neg(scratch->r1, scratch->r1);
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r1);
  mod_e222_m117_sqrt(scratch->r1, scratch->r1);

  /* r2 = (1 + r1) / r0 */
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r2);
  mod_e222_m117_div(scratch->r2, scratch->r0, scratch->r2);

  /* r0.1 = r0 * Y */
  mod_e222_m117_mul(scratch->r0, p->y, scratch->r0);

  /* r1.1 = (2 * r2 * r1) / r0.1 */
  mod_e222_m117_mul(scratch->r2, scratch->r1, scratch->r1);
  mod_e222_m117_add_small(scratch->r1, 2, scratch->r1);
  mod_e222_m117_div(scratch->r1, scratch->r0, scratch->r1);

  const int i0 = mod_e222_m117_signum(scratch->r2, scratch);

  /* r1.2 = r2.signum * (r1.1 + 1) */
  mod_e222_m117_add_small(scratch->r1, 1, scratch->r1);
  mod_e222_m117_mul_small(scratch->r1, i0, scratch->r1);

  /* r2.1 = ((2 * d) - 1) * r2^2 */
  mod_e222_m117_square(scratch->r2, scratch->r2);
  mod_e222_m117_mul_small(scratch->r2, (2 * edwards_d) - 1, scratch->r2);

  /* r0.2 = (r2.1 - r1.2) * nonresidue */
  mod_e222_m117_sub(scratch->r2, scratch->r1, scratch->r0);
  mod_e222_m117_mul_small(scratch->r0, 2, scratch->r0);

  /* r2.2 = (r2.1 + r1.2) / r0.2 */
  mod_e222_m117_add(scratch->r2, scratch->r1, scratch->r2);
  mod_e222_m117_div(scratch->r2, scratch->r0, scratch->r2);

  return mod_e222_m117_leg(scratch->r2, &(scratch->inner)) == 1;
}

#endif
