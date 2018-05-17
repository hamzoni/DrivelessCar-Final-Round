/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2017 Yermalayeu Ihar.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "Simd/SimdMemory.h"
#include "Simd/SimdStore.h"
#include "Simd/SimdExtract.h"
#include "Simd/SimdArray.h"

namespace Simd
{
#ifdef SIMD_AVX512F_ENABLE    
    namespace Avx512f
    {
        SIMD_INLINE void AddProduct(float * ptr, __m512 value, __m512 alpha, __mmask16 mask)
        {
            _mm512_mask_storeu_ps(ptr, mask, _mm512_fmadd_ps(value, alpha, _mm512_maskz_loadu_ps(mask, ptr)));
        }

        static void Kernel4x48(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c01 = _mm512_setzero_ps();
            register __m512 c11 = _mm512_setzero_ps();
            register __m512 c21 = _mm512_setzero_ps();
            register __m512 c31 = _mm512_setzero_ps();
            register __m512 c02 = _mm512_setzero_ps();
            register __m512 c12 = _mm512_setzero_ps();
            register __m512 c22 = _mm512_setzero_ps();
            register __m512 c32 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            register __m512 b0, b1, b2, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                b2 = _mm512_loadu_ps(B + 2 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                c01 = _mm512_fmadd_ps(a0, b1, c01);
                c02 = _mm512_fmadd_ps(a0, b2, c02);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                c11 = _mm512_fmadd_ps(a0, b1, c11);
                c12 = _mm512_fmadd_ps(a0, b2, c12);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                c21 = _mm512_fmadd_ps(a0, b1, c21);
                c22 = _mm512_fmadd_ps(a0, b2, c22);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                c31 = _mm512_fmadd_ps(a0, b1, c31);
                c32 = _mm512_fmadd_ps(a0, b2, c32);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            AddProduct(C + 1 * F, _alpha, c01, mask[1]);
            AddProduct(C + 2 * F, _alpha, c02, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            AddProduct(C + 1 * F, _alpha, c11, mask[1]);
            AddProduct(C + 2 * F, _alpha, c12, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            AddProduct(C + 1 * F, _alpha, c21, mask[1]);
            AddProduct(C + 2 * F, _alpha, c22, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            AddProduct(C + 1 * F, _alpha, c31, mask[1]);
            AddProduct(C + 2 * F, _alpha, c32, mask[2]);
        }

        static void Kernel4x32(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c01 = _mm512_setzero_ps();
            register __m512 c11 = _mm512_setzero_ps();
            register __m512 c21 = _mm512_setzero_ps();
            register __m512 c31 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            register __m512 b0, b1, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                c01 = _mm512_fmadd_ps(a0, b1, c01);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                c11 = _mm512_fmadd_ps(a0, b1, c11);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                c21 = _mm512_fmadd_ps(a0, b1, c21);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                c31 = _mm512_fmadd_ps(a0, b1, c31);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            AddProduct(C + 1 * F, _alpha, c01, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            AddProduct(C + 1 * F, _alpha, c11, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            AddProduct(C + 1 * F, _alpha, c21, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            AddProduct(C + 1 * F, _alpha, c31, mask[1]);
        }

        static void Kernel4x16(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c0 = _mm512_setzero_ps();
            register __m512 c1 = _mm512_setzero_ps();
            register __m512 c2 = _mm512_setzero_ps();
            register __m512 c3 = _mm512_setzero_ps();
            const float * a0 = A + lda * 0;
            const float * a1 = A + lda * 1;
            const float * a2 = A + lda * 2;
            const float * a3 = A + lda * 3;
            register __m512 b0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B);
                c0 = _mm512_fmadd_ps(b0, _mm512_set1_ps(*a0++), c0);
                c1 = _mm512_fmadd_ps(b0, _mm512_set1_ps(*a1++), c1);
                c2 = _mm512_fmadd_ps(b0, _mm512_set1_ps(*a2++), c2);
                c3 = _mm512_fmadd_ps(b0, _mm512_set1_ps(*a3++), c3);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * ldc, _alpha, c0, mask[0]);
            AddProduct(C + 1 * ldc, _alpha, c1, mask[0]);
            AddProduct(C + 2 * ldc, _alpha, c2, mask[0]);
            AddProduct(C + 3 * ldc, _alpha, c3, mask[0]);
        }

        static void Kernel6x32(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            register __m512 c01 = _mm512_setzero_ps();
            register __m512 c11 = _mm512_setzero_ps();
            register __m512 c21 = _mm512_setzero_ps();
            register __m512 c31 = _mm512_setzero_ps();
            register __m512 c41 = _mm512_setzero_ps();
            register __m512 c51 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            register __m512 b0, b1, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                c01 = _mm512_fmadd_ps(a0, b1, c01);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                c11 = _mm512_fmadd_ps(a0, b1, c11);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                c21 = _mm512_fmadd_ps(a0, b1, c21);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                c31 = _mm512_fmadd_ps(a0, b1, c31);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                c41 = _mm512_fmadd_ps(a0, b1, c41);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                c51 = _mm512_fmadd_ps(a0, b1, c51);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            AddProduct(C + 1 * F, _alpha, c01, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            AddProduct(C + 1 * F, _alpha, c11, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            AddProduct(C + 1 * F, _alpha, c21, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            AddProduct(C + 1 * F, _alpha, c31, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            AddProduct(C + 1 * F, _alpha, c41, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
            AddProduct(C + 1 * F, _alpha, c51, mask[1]);
        }

        static void Kernel6x16(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            register __m512 b0, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
        }

        static void Kernel8x48(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c01 = _mm512_setzero_ps();
            register __m512 c02 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c11 = _mm512_setzero_ps();
            register __m512 c12 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c21 = _mm512_setzero_ps();
            register __m512 c22 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c31 = _mm512_setzero_ps();
            register __m512 c32 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c41 = _mm512_setzero_ps();
            register __m512 c42 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            register __m512 c51 = _mm512_setzero_ps();
            register __m512 c52 = _mm512_setzero_ps();
            register __m512 c60 = _mm512_setzero_ps();
            register __m512 c61 = _mm512_setzero_ps();
            register __m512 c62 = _mm512_setzero_ps();
            register __m512 c70 = _mm512_setzero_ps();
            register __m512 c71 = _mm512_setzero_ps();
            register __m512 c72 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            const float * A6 = A + lda * 6;
            const float * A7 = A + lda * 7;
            register __m512 b0, b1, b2, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                b2 = _mm512_loadu_ps(B + 2 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                c01 = _mm512_fmadd_ps(a0, b1, c01);
                c02 = _mm512_fmadd_ps(a0, b2, c02);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                c11 = _mm512_fmadd_ps(a0, b1, c11);
                c12 = _mm512_fmadd_ps(a0, b2, c12);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                c21 = _mm512_fmadd_ps(a0, b1, c21);
                c22 = _mm512_fmadd_ps(a0, b2, c22);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                c31 = _mm512_fmadd_ps(a0, b1, c31);
                c32 = _mm512_fmadd_ps(a0, b2, c32);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                c41 = _mm512_fmadd_ps(a0, b1, c41);
                c42 = _mm512_fmadd_ps(a0, b2, c42);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                c51 = _mm512_fmadd_ps(a0, b1, c51);
                c52 = _mm512_fmadd_ps(a0, b2, c52);
                a0 = _mm512_set1_ps(*A6++);
                c60 = _mm512_fmadd_ps(a0, b0, c60);
                c61 = _mm512_fmadd_ps(a0, b1, c61);
                c62 = _mm512_fmadd_ps(a0, b2, c62);
                a0 = _mm512_set1_ps(*A7++);
                c70 = _mm512_fmadd_ps(a0, b0, c70);
                c71 = _mm512_fmadd_ps(a0, b1, c71);
                c72 = _mm512_fmadd_ps(a0, b2, c72);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            AddProduct(C + 1 * F, _alpha, c01, mask[1]);
            AddProduct(C + 2 * F, _alpha, c02, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            AddProduct(C + 1 * F, _alpha, c11, mask[1]);
            AddProduct(C + 2 * F, _alpha, c12, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            AddProduct(C + 1 * F, _alpha, c21, mask[1]);
            AddProduct(C + 2 * F, _alpha, c22, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            AddProduct(C + 1 * F, _alpha, c31, mask[1]);
            AddProduct(C + 2 * F, _alpha, c32, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            AddProduct(C + 1 * F, _alpha, c41, mask[1]);
            AddProduct(C + 2 * F, _alpha, c42, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
            AddProduct(C + 1 * F, _alpha, c51, mask[1]);
            AddProduct(C + 2 * F, _alpha, c52, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c60, mask[0]);
            AddProduct(C + 1 * F, _alpha, c61, mask[1]);
            AddProduct(C + 2 * F, _alpha, c62, mask[2]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c70, mask[0]);
            AddProduct(C + 1 * F, _alpha, c71, mask[1]);
            AddProduct(C + 2 * F, _alpha, c72, mask[2]);
        }

        static void Kernel8x32(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c01 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c11 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c21 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c31 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c41 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            register __m512 c51 = _mm512_setzero_ps();
            register __m512 c60 = _mm512_setzero_ps();
            register __m512 c61 = _mm512_setzero_ps();
            register __m512 c70 = _mm512_setzero_ps();
            register __m512 c71 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            const float * A6 = A + lda * 6;
            const float * A7 = A + lda * 7;
            register __m512 b0, b1, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                c01 = _mm512_fmadd_ps(a0, b1, c01);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                c11 = _mm512_fmadd_ps(a0, b1, c11);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                c21 = _mm512_fmadd_ps(a0, b1, c21);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                c31 = _mm512_fmadd_ps(a0, b1, c31);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                c41 = _mm512_fmadd_ps(a0, b1, c41);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                c51 = _mm512_fmadd_ps(a0, b1, c51);
                a0 = _mm512_set1_ps(*A6++);
                c60 = _mm512_fmadd_ps(a0, b0, c60);
                c61 = _mm512_fmadd_ps(a0, b1, c61);
                a0 = _mm512_set1_ps(*A7++);
                c70 = _mm512_fmadd_ps(a0, b0, c70);
                c71 = _mm512_fmadd_ps(a0, b1, c71);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            AddProduct(C + 1 * F, _alpha, c01, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            AddProduct(C + 1 * F, _alpha, c11, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            AddProduct(C + 1 * F, _alpha, c21, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            AddProduct(C + 1 * F, _alpha, c31, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            AddProduct(C + 1 * F, _alpha, c41, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
            AddProduct(C + 1 * F, _alpha, c51, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c60, mask[0]);
            AddProduct(C + 1 * F, _alpha, c61, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c70, mask[0]);
            AddProduct(C + 1 * F, _alpha, c71, mask[1]);
        }

        static void Kernel8x16(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            register __m512 c60 = _mm512_setzero_ps();
            register __m512 c70 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            const float * A6 = A + lda * 6;
            const float * A7 = A + lda * 7;
            register __m512 b0, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                a0 = _mm512_set1_ps(*A6++);
                c60 = _mm512_fmadd_ps(a0, b0, c60);
                a0 = _mm512_set1_ps(*A7++);
                c70 = _mm512_fmadd_ps(a0, b0, c70);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c60, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c70, mask[0]);
        }

        static void Kernel12x32(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            register __m512 c01 = _mm512_setzero_ps();
            register __m512 c11 = _mm512_setzero_ps();
            register __m512 c21 = _mm512_setzero_ps();
            register __m512 c31 = _mm512_setzero_ps();
            register __m512 c41 = _mm512_setzero_ps();
            register __m512 c51 = _mm512_setzero_ps();
            register __m512 c60 = _mm512_setzero_ps();
            register __m512 c70 = _mm512_setzero_ps();
            register __m512 c80 = _mm512_setzero_ps();
            register __m512 c90 = _mm512_setzero_ps();
            register __m512 cA0 = _mm512_setzero_ps();
            register __m512 cB0 = _mm512_setzero_ps();
            register __m512 c61 = _mm512_setzero_ps();
            register __m512 c71 = _mm512_setzero_ps();
            register __m512 c81 = _mm512_setzero_ps();
            register __m512 c91 = _mm512_setzero_ps();
            register __m512 cA1 = _mm512_setzero_ps();
            register __m512 cB1 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            const float * A6 = A + lda * 6;
            const float * A7 = A + lda * 7;
            const float * A8 = A + lda * 8;
            const float * A9 = A + lda * 9;
            const float * AA = A + lda * 10;
            const float * AB = A + lda * 11;
            register __m512 b0, b1, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                c01 = _mm512_fmadd_ps(a0, b1, c01);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                c11 = _mm512_fmadd_ps(a0, b1, c11);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                c21 = _mm512_fmadd_ps(a0, b1, c21);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                c31 = _mm512_fmadd_ps(a0, b1, c31);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                c41 = _mm512_fmadd_ps(a0, b1, c41);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                c51 = _mm512_fmadd_ps(a0, b1, c51);
                a0 = _mm512_set1_ps(*A6++);
                c60 = _mm512_fmadd_ps(a0, b0, c60);
                c61 = _mm512_fmadd_ps(a0, b1, c61);
                a0 = _mm512_set1_ps(*A7++);
                c70 = _mm512_fmadd_ps(a0, b0, c70);
                c71 = _mm512_fmadd_ps(a0, b1, c71);
                a0 = _mm512_set1_ps(*A8++);
                c80 = _mm512_fmadd_ps(a0, b0, c80);
                c81 = _mm512_fmadd_ps(a0, b1, c81);
                a0 = _mm512_set1_ps(*A9++);
                c90 = _mm512_fmadd_ps(a0, b0, c90);
                c91 = _mm512_fmadd_ps(a0, b1, c91);
                a0 = _mm512_set1_ps(*AA++);
                cA0 = _mm512_fmadd_ps(a0, b0, cA0);
                cA1 = _mm512_fmadd_ps(a0, b1, cA1);
                a0 = _mm512_set1_ps(*AB++);
                cB0 = _mm512_fmadd_ps(a0, b0, cB0);
                cB1 = _mm512_fmadd_ps(a0, b1, cB1);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            AddProduct(C + 1 * F, _alpha, c01, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            AddProduct(C + 1 * F, _alpha, c11, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            AddProduct(C + 1 * F, _alpha, c21, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            AddProduct(C + 1 * F, _alpha, c31, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            AddProduct(C + 1 * F, _alpha, c41, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
            AddProduct(C + 1 * F, _alpha, c51, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c60, mask[0]);
            AddProduct(C + 1 * F, _alpha, c61, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c70, mask[0]);
            AddProduct(C + 1 * F, _alpha, c71, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c80, mask[0]);
            AddProduct(C + 1 * F, _alpha, c81, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c90, mask[0]);
            AddProduct(C + 1 * F, _alpha, c91, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, cA0, mask[0]);
            AddProduct(C + 1 * F, _alpha, cA1, mask[1]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, cB0, mask[0]);
            AddProduct(C + 1 * F, _alpha, cB1, mask[1]);
        }

        static void Kernel12x16(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
            register __m512 c00 = _mm512_setzero_ps();
            register __m512 c10 = _mm512_setzero_ps();
            register __m512 c20 = _mm512_setzero_ps();
            register __m512 c30 = _mm512_setzero_ps();
            register __m512 c40 = _mm512_setzero_ps();
            register __m512 c50 = _mm512_setzero_ps();
            register __m512 c60 = _mm512_setzero_ps();
            register __m512 c70 = _mm512_setzero_ps();
            register __m512 c80 = _mm512_setzero_ps();
            register __m512 c90 = _mm512_setzero_ps();
            register __m512 cA0 = _mm512_setzero_ps();
            register __m512 cB0 = _mm512_setzero_ps();
            const float * A0 = A + lda * 0;
            const float * A1 = A + lda * 1;
            const float * A2 = A + lda * 2;
            const float * A3 = A + lda * 3;
            const float * A4 = A + lda * 4;
            const float * A5 = A + lda * 5;
            const float * A6 = A + lda * 6;
            const float * A7 = A + lda * 7;
            const float * A8 = A + lda * 8;
            const float * A9 = A + lda * 9;
            const float * AA = A + lda * 10;
            const float * AB = A + lda * 11;
            register __m512 b0, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                a0 = _mm512_set1_ps(*A0++);
                c00 = _mm512_fmadd_ps(a0, b0, c00);
                a0 = _mm512_set1_ps(*A1++);
                c10 = _mm512_fmadd_ps(a0, b0, c10);
                a0 = _mm512_set1_ps(*A2++);
                c20 = _mm512_fmadd_ps(a0, b0, c20);
                a0 = _mm512_set1_ps(*A3++);
                c30 = _mm512_fmadd_ps(a0, b0, c30);
                a0 = _mm512_set1_ps(*A4++);
                c40 = _mm512_fmadd_ps(a0, b0, c40);
                a0 = _mm512_set1_ps(*A5++);
                c50 = _mm512_fmadd_ps(a0, b0, c50);
                a0 = _mm512_set1_ps(*A6++);
                c60 = _mm512_fmadd_ps(a0, b0, c60);
                a0 = _mm512_set1_ps(*A7++);
                c70 = _mm512_fmadd_ps(a0, b0, c70);
                a0 = _mm512_set1_ps(*A8++);
                c80 = _mm512_fmadd_ps(a0, b0, c80);
                a0 = _mm512_set1_ps(*A9++);
                c90 = _mm512_fmadd_ps(a0, b0, c90);
                a0 = _mm512_set1_ps(*AA++);
                cA0 = _mm512_fmadd_ps(a0, b0, cA0);
                a0 = _mm512_set1_ps(*AB++);
                cB0 = _mm512_fmadd_ps(a0, b0, cB0);
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            AddProduct(C + 0 * F, _alpha, c00, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c10, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c20, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c30, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c40, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c50, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c60, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c70, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c80, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, c90, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, cA0, mask[0]);
            C += ldc;
            AddProduct(C + 0 * F, _alpha, cB0, mask[0]);
        }

        static void KernelMx48(size_t M, size_t N, size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
#if SIMD_ZMM_COUNT == 32
            register __m512 c[8][3];
            register const float * a[8];
#else
            register __m512 c[4][3];
            register const float * a[4];
#endif
            for (size_t i = 0; i < M; ++i)
            {
                c[i][0] = _mm512_setzero_ps();
                c[i][1] = _mm512_setzero_ps();
                c[i][2] = _mm512_setzero_ps();
                a[i] = A + lda * i;
            }
            register __m512 b0, b1, b2, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                b2 = _mm512_loadu_ps(B + 2 * F);
                for (size_t i = 0; i < M; ++i)
                {
                    a0 = _mm512_set1_ps(*a[i]++);
                    c[i][0] = _mm512_add_ps(_mm512_mul_ps(b0, a0), c[i][0]);
                    c[i][1] = _mm512_add_ps(_mm512_mul_ps(b1, a0), c[i][1]);
                    c[i][2] = _mm512_add_ps(_mm512_mul_ps(b2, a0), c[i][2]);
                }
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            for (size_t i = 0; i < M; ++i)
            {
                AddProduct(C + 0 * F, _alpha, c[i][0], mask[0]);
                AddProduct(C + 1 * F, _alpha, c[i][1], mask[1]);
                AddProduct(C + 2 * F, _alpha, c[i][2], mask[2]);
                C += ldc;
            }
        }

        static void KernelMx32(size_t M, size_t N, size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
#if SIMD_ZMM_COUNT == 32
            register __m512 c[12][2];
            register const float * a[12];
#else
            register __m512 c[6][2];
            register const float * a[6];
#endif
            for (size_t i = 0; i < M; ++i)
            {
                c[i][0] = _mm512_setzero_ps();
                c[i][1] = _mm512_setzero_ps();
                a[i] = A + lda * i;
            }
            register __m512 b0, b1, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                b1 = _mm512_loadu_ps(B + 1 * F);
                for (size_t i = 0; i < M; ++i)
                {
                    a0 = _mm512_set1_ps(*a[i]++);
                    c[i][0] = _mm512_fmadd_ps(b0, a0, c[i][0]);
                    c[i][1] = _mm512_fmadd_ps(b1, a0, c[i][1]);
                }
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            for (size_t i = 0; i < M; ++i)
            {
                AddProduct(C + 0 * F, _alpha, c[i][0], mask[0]);
                AddProduct(C + 1 * F, _alpha, c[i][1], mask[1]);
                C += ldc;
            }
        }

        static void KernelMx16(size_t M, size_t N, size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask)
        {
#if SIMD_ZMM_COUNT == 32
            register __m512 c[12];
            register const float * a[12];
#elif SIMD_ZMM_COUNT == 16
            register __m512 c[6];
            register const float * a[6];
#else
            register __m512 c[4];
            register const float * a[4];
#endif
            for (size_t i = 0; i < M; ++i)
            {
                c[i] = _mm512_setzero_ps();
                a[i] = A + lda * i;
            }
            register __m512 b0, a0;
            for (size_t k = 0; k < K; k++)
            {
                b0 = _mm512_loadu_ps(B + 0 * F);
                for (size_t i = 0; i < M; ++i)
                {
                    a0 = _mm512_set1_ps(*a[i]++);
                    c[i] = _mm512_fmadd_ps(b0, a0, c[i]);
                }
                B += ldb;
            }
            __m512 _alpha = _mm512_set1_ps(alpha);
            for (size_t i = 0; i < M; ++i)
                AddProduct(C + i * ldc, _alpha, c[i], mask[0]);
        }

        SIMD_INLINE void ScaleC(float * ptr, __m512 value, __mmask16 mask = -1)
        {
            _mm512_mask_storeu_ps(ptr, mask, _mm512_mul_ps(_mm512_maskz_loadu_ps(mask, ptr), value));
        }

        static void ScaleC(size_t M, size_t N, float value, float * C, size_t ldc)
        {
            size_t NQF = AlignLo(N, QF);
            size_t NF = AlignLo(N, F);
            __m512 _value = _mm512_set1_ps(value);
            __mmask16 tail = TailMask16(N - NF);
            for (size_t i = 0; i < M; ++i)
            {
                size_t j = 0;
                for (; j < NQF; j += QF)
                {
                    ScaleC(C + j + F * 0, _value);
                    ScaleC(C + j + F * 1, _value);
                    ScaleC(C + j + F * 2, _value);
                    ScaleC(C + j + F * 3, _value);
                }
                for (; j < NF; j += F)
                    ScaleC(C + j, _value);
                if(j < N)
                    ScaleC(C + j, _value, tail);
                C += ldc;
            }
        }

        static void PackA(const float * src, size_t stride, size_t M, size_t K, size_t cell, float * dst)
        {
            size_t K4 = AlignLo(K, 4), K8 = AlignLo(K, 8);
            for (size_t i = 0; i < M; i += cell)
            {
                size_t m = Simd::Min(cell, M - i), k = 0;
                if (cell == 4 && m == 4)
                {
                    for (; k < K8; k += 8)
                    {
                        const float * ps = src + k;
                        __m256 s0 = _mm256_loadu_ps(ps + 0 * K);
                        __m256 s1 = _mm256_loadu_ps(ps + 1 * K);
                        __m256 s2 = _mm256_loadu_ps(ps + 2 * K);
                        __m256 s3 = _mm256_loadu_ps(ps + 3 * K);
                        __m256 s00 = _mm256_unpacklo_ps(s0, s2);
                        __m256 s01 = _mm256_unpacklo_ps(s1, s3);
                        __m256 s10 = _mm256_unpackhi_ps(s0, s2);
                        __m256 s11 = _mm256_unpackhi_ps(s1, s3);
                        __m256 d0 = _mm256_unpacklo_ps(s00, s01);
                        __m256 d1 = _mm256_unpackhi_ps(s00, s01);
                        __m256 d2 = _mm256_unpacklo_ps(s10, s11);
                        __m256 d3 = _mm256_unpackhi_ps(s10, s11);
                        _mm256_storeu_ps(dst + 0, _mm256_permute2f128_ps(d0, d1, 0x20));
                        _mm256_storeu_ps(dst + 8, _mm256_permute2f128_ps(d2, d3, 0x20));
                        _mm256_storeu_ps(dst + 16, _mm256_permute2f128_ps(d0, d1, 0x31));
                        _mm256_storeu_ps(dst + 24, _mm256_permute2f128_ps(d2, d3, 0x31));
                        dst += 32;
                    };
                    for (; k < K4; k += 4)
                    {
                        const float * ps = src + k;
                        __m128 s0 = _mm_loadu_ps(ps + 0 * stride);
                        __m128 s1 = _mm_loadu_ps(ps + 1 * stride);
                        __m128 s2 = _mm_loadu_ps(ps + 2 * stride);
                        __m128 s3 = _mm_loadu_ps(ps + 3 * stride);
                        __m128 s00 = _mm_unpacklo_ps(s0, s2);
                        __m128 s01 = _mm_unpacklo_ps(s1, s3);
                        __m128 s10 = _mm_unpackhi_ps(s0, s2);
                        __m128 s11 = _mm_unpackhi_ps(s1, s3);
                        _mm_storeu_ps(dst + 0, _mm_unpacklo_ps(s00, s01));
                        _mm_storeu_ps(dst + 4, _mm_unpackhi_ps(s00, s01));
                        _mm_storeu_ps(dst + 8, _mm_unpacklo_ps(s10, s11));
                        _mm_storeu_ps(dst + 12, _mm_unpackhi_ps(s10, s11));
                        dst += 16;
                    }
                }
                for (; k < K; ++k)
                {
                    for (size_t c = 0; c < m; ++c)
                        *(dst++) = src[c*stride + k];
                }  
                src += cell * stride;
            }
        }

        static void PackB(const float * B, size_t ldb, size_t K, size_t N, size_t microN, float * pB)
        {
            for (size_t j = 0; j < N; j += microN)
            {
                size_t n = Simd::Min(microN, N - j);
                if (microN == 1 * F)
                {
                    __mmask16 mask0 = TailMask16(n - 0 * F);
                    for (size_t k = 0; k < K; ++k)
                    {
                        const float * b = B + k * ldb;
                        _mm512_storeu_ps(pB + 0 * F, _mm512_maskz_loadu_ps(mask0, b + 0 * F));
                        pB += microN;
                    }
                }
                else if (microN == 2 * F)
                {
                    __mmask16 mask0 = TailMask16(n - 0 * F);
                    __mmask16 mask1 = TailMask16(n - 1 * F);
                    for (size_t k = 0; k < K; ++k)
                    {
                        const float * b = B + k * ldb;
                        _mm512_storeu_ps(pB + 0 * F, _mm512_maskz_loadu_ps(mask0, b + 0 * F));
                        _mm512_storeu_ps(pB + 1 * F, _mm512_maskz_loadu_ps(mask1, b + 1 * F));
                        pB += microN;
                    }
                }
                else if (microN == 3 * F)
                {
                    __mmask16 mask0 = TailMask16(n - 0 * F);
                    __mmask16 mask1 = TailMask16(n - 1 * F);
                    __mmask16 mask2 = TailMask16(n - 2 * F);
                    for (size_t k = 0; k < K; ++k)
                    {
                        const float * b = B + k * ldb;
                        _mm512_storeu_ps(pB + 0 * F, _mm512_maskz_loadu_ps(mask0, b + 0 * F));
                        _mm512_storeu_ps(pB + 1 * F, _mm512_maskz_loadu_ps(mask1, b + 1 * F));
                        _mm512_storeu_ps(pB + 2 * F, _mm512_maskz_loadu_ps(mask2, b + 2 * F));
                        pB += microN;
                    }
                }
                else
                {
                    for (size_t k = 0; k < K; ++k)
                    {
                        const float * b = B + k * ldb;
                        size_t c = 0;
                        for (; c < n; ++c)
                            *(pB++) = *(b++);
                        for (; c < microN; ++c)
                            *(pB++) = 0;
                    }
                }
                B += microN;
            }
        }

        class GemmNN
        {
            const size_t CACHE_L1_SIZE = 32 * 1024;
            const size_t CACHE_L2_SIZE = 256 * 1024;
            const size_t CACHE_L3_SIZE = 2*1024 * 1024;
        public:
            typedef void(*Main)(size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask);
            typedef void(*Tail)(size_t M, size_t N, size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float * C, size_t ldc, const __mmask16 * mask);
            typedef void(*ScaleC)(size_t M, size_t N, float beta, float * C, size_t ldc);
            typedef void(*PackB)(const float * B, size_t ldb, size_t K, size_t N, size_t microN, float * pB);

            GemmNN(size_t M, size_t N, size_t K)
                : _M(M)
                , _N(N)
                , _K(K)
                , _scaleC(Avx512f::ScaleC)
                , _packB(Avx512f::PackB)
            {
#if SIMD_ZMM_COUNT == 32
                if (K > 4024 && false)
                {
                    _microM = 12;
                    _microN = 32;
                    size_t tail = N - AlignLoAny(N, _microN);
                    _kernelMM = Kernel12x32;
                    _kernelMT = tail > F ? Kernel12x32 : Kernel12x16;
                    _kernelTM = KernelMx32;
                    _kernelTT = tail > F ? KernelMx32 : KernelMx16;
                }
                else
                {
                    _microM = 8;
                    _microN = 48;
                    size_t tail = N - AlignLoAny(N, _microN);
                    _kernelMM = Kernel8x48;
                    _kernelMT = tail > DF ? Kernel8x48 : (tail > F ? Kernel8x32 : Kernel8x16);
                    _kernelTM = KernelMx48;
                    _kernelTT = tail > DF ? KernelMx48 : (tail > F ? KernelMx32 : KernelMx16);
                }
#elif SIMD_ZMM_COUNT == 16
                if (K > 4024)
                {
                    _microM = 6;
                    _microN = 32;
                    size_t tail = N - AlignLoAny(N, _microN);
                    _kernelMM = Kernel6x32;
                    _kernelMT = tail > F ? Kernel6x32 : Kernel6x16;
                    _kernelTM = KernelMx32;
                    _kernelTT = tail > F ? KernelMx32 : KernelMx16;
                }
                else
                {
                    _microM = 4;
                    _microN = 48;
                    size_t tail = N - AlignLoAny(N, _microN);
                    _kernelMM = Kernel4x48;
                    _kernelMT = tail > DF ? Kernel4x48 : (tail > F ? Kernel4x32 : Kernel4x16);
                    _kernelTM = KernelMx48;
                    _kernelTT = tail > DF ? KernelMx48 : (tail > F ? KernelMx32 : KernelMx16);
                }
#else
                _microM = 4;
                _microN = 16;
                _kernelMM = Kernel4x16;
                _kernelMT = Kernel4x16;
                _kernelTM = KernelMx16;
                _kernelTT = KernelMx16;
#endif
                _macroK = CACHE_L2_SIZE / sizeof(float) / _microN;
                _macroM = AlignLoAny(CACHE_L3_SIZE / sizeof(float) / _macroK, _microM);
                _macroN = AlignLoAny(CACHE_L3_SIZE / sizeof(float) / _macroK, _microN);
                _pA.Resize(_macroM * _macroK);
                _pB.Resize(_macroN * _macroK);
                size_t NA = AlignLoAny(N, _microN);
                for (size_t j = 0; j < 3; ++j)
                {
                    _main[j] = __mmask16(-1);
                    _tail[j] = TailMask16(N - NA - F * j);
                }
            }

            void Run(const float * alpha, const float * A, size_t lda, const float * B, size_t ldb, const float * beta, float * C, size_t ldc)
            {
                for (size_t j = 0; j < _N; j += _macroN)
                {
                    size_t macroN = Simd::Min(_N, j + _macroN) - j;
                    for (size_t k = 0; k < _K; k += _macroK)
                    {
                        size_t macroK = Simd::Min(_K, k + _macroK) - k;
                        //PackA(A + i * lda, lda, macroM, K, _microM, _A.data);
                        for (size_t i = 0; i < _M; i += _macroM)
                        {
                            size_t macroM = Simd::Min(_M, i + _macroM) - i;
                            if (k == 0)
                                _scaleC(macroM, macroN, *beta, C + i * ldc + j, ldc);
                            MacroKernel(macroM, macroN, macroK, *alpha, A + i * lda + k, lda, B + k * ldb + j, ldb, *beta, C + i * ldc + j, ldc, i == 0);
                        }
                    }
                }
            }

        private:
            void MacroKernel(size_t M, size_t N, size_t K, float alpha, const float * A, size_t lda, const float * B, size_t ldb, float beta, float * C, size_t ldc, bool packB)
            {
                size_t MA = AlignLoAny(M, _microM);
                size_t NA = AlignLoAny(N, _microN);
                size_t j = 0;
                for (; j < NA; j += _microN)
                {
                    float * pB = _pB.data + j * _macroK;
                    if (packB)
                        _packB(B + j, ldb, K, _microN, _microN, pB);
                    size_t i = 0;
                    for (; i < MA; i += _microM)
                        _kernelMM(K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc, _main);
                    if (i < M)
                        _kernelTM(M - i, _microN, K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc, _main);
                }
                if (j < N)
                {
                    float * pB = _pB.data + j * _macroK;
                    if (packB)
                        _packB(B + j, ldb, K, N - j, _microN, pB);
                    size_t i = 0;
                    for (; i < MA; i += _microM)
                        _kernelMT(K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc, _tail);
                    if (i < M)
                        _kernelTT(M - i, NA - j, K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc, _tail);
                }
            }

            Array<float> _pA, _pB;
            size_t _M, _N, _K, _microM, _microN, _macroM, _macroN, _macroK;
            Main _kernelMM, _kernelMT;
            Tail _kernelTM, _kernelTT;
            ScaleC _scaleC;
            PackB _packB;
            __mmask16 _main[3], _tail[3];
        };

        void Gemm32fNN(size_t M, size_t N, size_t K, const float * alpha, const float * A, size_t lda, const float * B, size_t ldb, const float * beta, float * C, size_t ldc)
        {
            GemmNN gemmNN(M, N, K);
            gemmNN.Run(alpha, A, lda, B, ldb, beta, C, ldc);
        }
    }
#endif// SIMD_AVX512F_ENABLE
}
