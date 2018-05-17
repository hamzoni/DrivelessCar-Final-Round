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
#ifndef __SimdGemm_h__
#define __SimdGemm_h__

#include "Simd/SimdArray.h"

namespace Simd
{
    template <class T> class GemmNN
    {
    public:
        typedef void(*Main)(size_t K, T alpha, const T * A, size_t lda, const T * B, size_t ldb, T * C, size_t ldc);
        typedef void(*Tail)(size_t M, size_t N, size_t K, T alpha, const T * A, size_t lda, const T * B, size_t ldb, T * C, size_t ldc);
        typedef void(*ScaleC)(size_t M, size_t N, T beta, T * C, size_t ldc);
        typedef void(*PackB)(const T * B, size_t ldb, size_t K, size_t N, size_t microN, T * pB);

        GemmNN(Main kernelMM, Main kernelMT, Tail kernelTM, Tail kernelTT, ScaleC scaleC, PackB packB,
            size_t microM, size_t microN, size_t L1, size_t L2, size_t L3)
            : _kernelMM(kernelMM)
            , _kernelMT(kernelMT)
            , _kernelTM(kernelTM)
            , _kernelTT(kernelTT)
            , _scaleC(scaleC)
            , _packB(packB)
            , _microM(microM)
            , _microN(microN)
        {

            _macroK = L1 / sizeof(T) / _microN;
            _macroM = AlignLoAny(L2 / sizeof(T) / _macroK, _microM);
            _macroN = AlignLoAny(L3 / sizeof(T) / _macroK, _microN);
            _pA.Resize(_macroM * _macroK);
            _pB.Resize(_macroN * _macroK);
        }

        void Run(size_t M, size_t N, size_t K, const T * alpha, const T * A, size_t lda, const T * B, size_t ldb, const T * beta, T * C, size_t ldc)
        {
            for (size_t j = 0; j < N; j += _macroN)
            {
                size_t macroN = Simd::Min(N, j + _macroN) - j;
                for (size_t k = 0; k < K; k += _macroK)
                {
                    size_t macroK = Simd::Min(K, k + _macroK) - k;
                    //PackA(A + i * lda, lda, macroM, K, _microM, _A.data);
                    for (size_t i = 0; i < M; i += _macroM)
                    {
                        size_t macroM = Simd::Min(M, i + _macroM) - i;
                        if (k == 0)
                            _scaleC(macroM, macroN, *beta, C + i * ldc + j, ldc);
                        MacroKernel(macroM, macroN, macroK, *alpha, A + i * lda + k, lda, B + k * ldb + j, ldb, *beta, C + i * ldc + j, ldc, i == 0);
                    }
                }
            }
        }

    private:
        void MacroKernel(size_t M, size_t N, size_t K, T alpha, const T * A, size_t lda, const T * B, size_t ldb, T beta, T * C, size_t ldc, bool packB)
        {
            size_t MA = AlignLoAny(M, _microM);
            size_t NA = AlignLoAny(N, _microN);
            size_t j = 0;
            for (; j < NA; j += _microN)
            {
                T * pB = _pB.data + j * _macroK;
                if (packB)
                    _packB(B + j, ldb, K, _microN, _microN, pB);
                size_t i = 0;
                for (; i < MA; i += _microM)
                    _kernelMM(K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc);
                if (i < M)
                    _kernelTM(M - i, _microN, K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc);
            }
            if (j < N)
            {
                T * pB = _pB.data + j * _macroK;
                if (packB)
                    _packB(B + j, ldb, K, N - j, _microN, pB);
                size_t i = 0;
                for (; i < MA; i += _microM)
                    _kernelMT(K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc);
                if (i < M)
                    _kernelTT(M - i, NA - j, K, alpha, A + i * lda, lda, pB, _microN, C + i * ldc + j, ldc);
            }
        }

        Array<T> _pA, _pB;
        size_t _microM, _microN, _macroM, _macroN, _macroK;
        Main _kernelMM, _kernelMT;
        Tail _kernelTM, _kernelTT;
        ScaleC _scaleC;
        PackB _packB;
    };

#ifdef SIMD_AVX_ENABLE
    namespace Avx
    {
        void GemmScaleC(size_t M, size_t N, float beta, float * C, size_t ldc);

        void GemmPackB(const float * B, size_t ldb, size_t K, size_t N, size_t microN, float * pB);
    }
#endif//SIMD_AVX_ENABLE
}
#endif//__SimdGemm_h__
