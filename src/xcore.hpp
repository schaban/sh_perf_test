/*
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#define WIN32_LEAN_AND_MEAN 1
#define NOMINMAX
#define _WIN32_WINNT 0x0500
#include <tchar.h>
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <memory.h>
#include <malloc.h>
#include <intrin.h>

#include <new>

#include <d3d11.h>
#include <d3dcompiler.h>

#ifdef D3D_COMPILER_VERSION
#	if D3D_COMPILER_VERSION < 47
#		define XD_D3D_HAS_LINKER 0
#	else
#		define XD_D3D_HAS_LINKER 1
#	endif
#else
#	define XD_D3D_HAS_LINKER 0
#endif

#if defined(_MSC_VER)
#	pragma intrinsic(_BitScanForward)
#	pragma intrinsic(_BitScanReverse)
#	pragma intrinsic(_InterlockedIncrement)
#	pragma intrinsic(_InterlockedDecrement)
#	pragma intrinsic(_InterlockedExchangeAdd)
#	pragma intrinsic(_byteswap_ulong)
#endif

#ifndef XD_INLINE
#	define XD_INLINE __forceinline
#endif

#ifndef XD_NOINLINE
#	define XD_NOINLINE __declspec(noinline) 
#endif

#define XD_FOURCC(c1, c2, c3, c4) ((((uint8_t)(c4))<<24)|(((uint8_t)(c3))<<16)|(((uint8_t)(c2))<<8)|((uint8_t)(c1)))
#define XD_INCR_PTR(_ptr, _inc) ( &((uint8_t*)(_ptr))[_inc] )
#define XD_ARY_LEN(_arr) (sizeof((_arr)) / sizeof((_arr)[0]))
#define XD_ALIGN(_x, _n) ( ((intptr_t)(_x) + ((_n) - 1)) & (~((_n) - 1)) )

#define XD_BIT_ARY_SIZE(_storage_t, _nbits) ((((int)(_nbits)-1)/(sizeof(_storage_t)*8))+1)
#define XD_BIT_ARY_DECL(_storage_t, _name, _nbits) _name[XD_BIT_ARY_SIZE(storage_t, _nbits)]
#define XD_BIT_ARY_IDX(_storage_t, _no) ((_no) / (sizeof(_storage_t)*8))
#define XD_BIT_ARY_MASK(_storage_t, _no) (1 << ((_no)&((sizeof(_storage_t)*8) - 1)))
#define XD_BIT_ARY_ST(_storage_t, _ary, _no) (_ary)[XD_BIT_ARY_IDX(_storage_t, _no)] |= XD_BIT_ARY_MASK(_storage_t, _no)
#define XD_BIT_ARY_CL(_storage_t, _ary, _no) (_ary)[XD_BIT_ARY_IDX(_storage_t, _no)] &= ~XD_BIT_ARY_MASK(_storage_t, _no)
#define XD_BIT_ARY_SW(_storage_t, _ary, _no) (_ary)[XD_BIT_ARY_IDX(_storage_t, _no)] ^= XD_BIT_ARY_MASK(_storage_t, _no)
#define XD_BIT_ARY_CK(_storage_t, _ary, _no) (!!((_ary)[XD_BIT_ARY_IDX(_storage_t, _no)] & XD_BIT_ARY_MASK(_storage_t, _no)))

class cxWorkBrigade;
class cxJobQueue;

typedef int32_t xt_int;
typedef float xt_float;

struct xt_float2 {
	float x, y;

	void set(float xval, float yval) { x = xval; y = yval; }
	void fill(float val) { x = val; y = val; }
	void scl(float s) { x *= s; y *= s; }

	operator float* () { return reinterpret_cast<float*>(this); }
	operator const float* () const { return reinterpret_cast<const float*>(this); }
};

struct xt_float3 {
	float x, y, z;

	void set(float xval, float yval, float zval) { x = xval; y = yval; z = zval; }
	void fill(float val) { x = val; y = val; z = val; }
	void scl(float s) { x *= s; y *= s; z *= s; }

	operator float* () { return reinterpret_cast<float*>(this); }
	operator const float* () const { return reinterpret_cast<const float*>(this); }
};

struct xt_float4 {
	float x, y, z, w;

	void set(float xval, float yval, float zval, float wval) { x = xval; y = yval; z = zval; w = wval; }
	void fill(float val) { x = val; y = val; z = val; w = val; }
	void scl(float s) { x *= s; y *= s; z *= s; w *= s; }

	void set_at(int at, float val) {
		switch (at) {
			case 0: x = val; break;
			case 1: y = val; break;
			case 2: z = val; break;
			case 3: w = val; break;
			default: break;
		}
	}

	operator float* () { return reinterpret_cast<float*>(this); }
	operator const float* () const { return reinterpret_cast<const float*>(this); }
};

struct xt_mtx {
	float m[4][4];

	void identity() {
		__m128i* pV = reinterpret_cast<__m128i*>(this);
		__m128i v = _mm_insert_epi16(_mm_setzero_si128(), 0x3F80, 1);
		_mm_storeu_si128(pV, v);
		v = _mm_slli_si128(v, 4);
		_mm_storeu_si128(&pV[1], v);
		v = _mm_slli_si128(v, 4);
		_mm_storeu_si128(&pV[2], v);
		v = _mm_slli_si128(v, 4);
		_mm_storeu_si128(&pV[3], v);
	}

	operator float* () { return reinterpret_cast<float*>(this); }
	operator const float* () const { return reinterpret_cast<const float*>(this); }
};

struct xt_wmtx {
	float m[3][4];

	void identity() {
		__m128i* pV = reinterpret_cast<__m128i*>(this);
		__m128i v = _mm_insert_epi16(_mm_setzero_si128(), 0x3F80, 1);
		_mm_storeu_si128(pV, v);
		v = _mm_slli_si128(v, 4);
		_mm_storeu_si128(&pV[1], v);
		v = _mm_slli_si128(v, 4);
		_mm_storeu_si128(&pV[2], v);
	}

	operator float* () { return reinterpret_cast<float*>(this); }
	operator const float* () const { return reinterpret_cast<const float*>(this); }
};

struct xt_int2 {
	int32_t x, y;

	void set(int32_t xval, int32_t yval) { x = xval; y = yval; }
	void fill(int32_t val) { x = val; y = val; }
};

struct xt_int3 {
	int32_t x, y, z;

	void set(int32_t xval, int32_t yval, int32_t zval) { x = xval; y = yval; z = zval; }
	void fill(int32_t val) { x = val; y = val; z = val; }
};

struct xt_int4 {
	int32_t x, y, z, w;

	void set(int32_t xval, int32_t yval, int32_t zval, int32_t wval) { x = xval; y = yval; z = zval; w = wval; }
	void fill(int32_t val) { x = val; y = val; z = val; w = val; }

	void set_at(int at, int32_t val) {
		switch (at) {
			case 0: x = val; break;
			case 1: y = val; break;
			case 2: z = val; break;
			case 3: w = val; break;
			default: break;
		}
	}
};

struct xt_half {
	uint16_t x;

	void set(float f);
	float get() const;
};

struct xt_half2 {
	uint16_t x, y;

	void set(float fx, float fy);
	void set(xt_float2 fv) { set(fv.x, fv.y); }
	xt_float2 get() const;
};

struct xt_half3 {
	uint16_t x, y, z;

	void set(float fx, float fy, float fz);
	void set(xt_float3 fv) { set(fv.x, fv.y, fv.z); }
	xt_float3 get() const;
};

struct xt_half4 {
	uint16_t x, y, z, w;

	void set(float fx, float fy, float fz, float fw);
	void set(xt_float4 fv) { set(fv.x, fv.y, fv.z, fv.w); }
	xt_float4 get() const;
};

struct xt_texcoord {
	float u, v;

	void set(float u, float v) { this->u = u; this->v = v; }
	void fill(float val) { u = val; v = val; }
	void flip_v() { v = 1.0f - v; }
	void negate_v() { v = -v; }
	void scroll(const xt_texcoord& step);
	void encode_half(uint16_t* pDst) const;
};

union uxVal32 {
	int32_t i;
	uint32_t u;
	float f;
	uint8_t b[4];
};

union uxVal64 {
	int64_t i;
	uint64_t u;
	double f;
	uint8_t b[8];
};

#define XD_QSTR(_str) ([]() { static const uxQStr qs = {_str}; return qs; } )()
#define XD_QSTR_EQ(qs1, qs2) (_mm_movemask_epi8(_mm_cmpeq_epi8((qs1).mIQ, (qs2).mIQ)) == 0xFFFF)

union uxQStr {
	char mChr[16];
	__m128i mIQ;

	void load(const char* pStr, int n = 16) {
		mIQ = _mm_setzero_si128();
		if (n <= 0) return;
		if (n > 16) n = 16;
		for (int i = 0; i < n; ++i) {
			char c = pStr[i];
			if (!c) break;
			mChr[i] = c;
		}
	}

	bool operator == (const uxQStr& s) const { return XD_QSTR_EQ(*this, s); }
	bool operator != (const uxQStr& s) const { return !XD_QSTR_EQ(*this, s); }
};

struct sxGfxDevice {
	ID3D11Device* mpDev;
	ID3D11DeviceContext* mpCtx;

	sxGfxDevice() : mpDev(nullptr), mpCtx(nullptr) {}

	bool is_valid() const { return (mpDev && mpCtx); }

	void relese() {
		if (mpCtx) {
			mpCtx->Release();
		}
		if (mpDev) {
			mpDev->Release();
		}
		mpCtx = nullptr;
		mpDev = nullptr;
	}

	IDXGISwapChain* create_swap_chain(DXGI_SWAP_CHAIN_DESC& desc) const;
	bool is_dx11_level() const;
};

struct sxHLSLCompilerIfc {
	HMODULE mHandle;
	pD3DCompile mpD3DCompile;
	HRESULT(WINAPI * mpD3DReflect)(LPCVOID pCode, SIZE_T codeSize, REFIID refIfc, void** ppReflector);
	pD3DDisassemble mpD3DDisassemble;
#if XD_D3D_HAS_LINKER
	HRESULT(WINAPI * mpD3DCreateLinker)(ID3D11Linker** ppLinker);
	HRESULT(WINAPI * mpD3DLoadModule)(LPCVOID pSrcData, SIZE_T dataSize, ID3D11Module** ppModule);
	HRESULT(WINAPI * mpD3DCreateFunctionLinkingGraph)(UINT uFlags, ID3D11FunctionLinkingGraph** ppFunctionLinkingGraph);
	HRESULT(WINAPI * mpD3DReflectLibrary)(LPCVOID pSrcData, SIZE_T dataSize, REFIID refIfx, void** ppReflector);

	bool has_linker() const { return mpD3DCreateLinker != nullptr; }
#else
	bool has_linker() const { return false; }
#endif

	bool is_loaded() const { return mHandle != nullptr; }
	bool reflect(const void* pCode, size_t codeSize, ID3D11ShaderReflection** ppReflector);
};

namespace nxCore {

void* mem_alloc(size_t size);
void mem_free(void* pMem);
void dbg_msg(const char* fmt, ...);
void attach_console();
void set_exe_cwd();
void set_thread_name(const char* pName);
void fpu_init();
void cpu_serialize();
int64_t get_timestamp();
double get_perf_freq();
void* bin_load(const char* pPath, size_t* pSize = nullptr, bool appendPath = false);
void bin_unload(void* pMem);
void bin_save(const char* pPath, const void* pMem, size_t size);
uint32_t str_hash32(const char* pStr);
uint16_t str_hash16(const char* pStr);
bool str_eq(const char* pStrA, const char* pStrB);
bool str_starts_with(const char* pStr, const char* pPrefix);
bool str_ends_with(const char* pStr, const char* pPostfix);
char* str_dup(const char* pSrc);

inline void prefetch(const void* pMem) { _mm_prefetch((const char*)(pMem), _MM_HINT_T0); }
inline void prefetch_nt(const void* pMem) { _mm_prefetch((const char*)(pMem), _MM_HINT_NTA); }

template<typename T> inline void ctor(T* pMem, int n = 1) {
	T* pObj = pMem;
	for (int i = 0; i < n; ++i) {
		::new((void*)pObj) T;
		++pObj;
	}
}

template<typename T> inline void dtor(T* pMem, int n = 1) {
	T* pObj = pMem + (n - 1);
	for (int i = 0; i < n; ++i) {
		pObj->~T();
		--pObj;
	}
}

template<typename T> inline T* obj_alloc(int n = 1) {
	T* pObj = nullptr;
	size_t memsize = sizeof(T) * n;
	void* pMem = mem_alloc(memsize);
	if (pMem) {
		::ZeroMemory(pMem, memsize);
		pObj = reinterpret_cast<T*>(pMem);
		ctor(pObj, n);
	}
	return pObj;
}

template<typename T> inline void obj_free(T* pMem, int n = 1) {
	if (pMem) {
		dtor(pMem, n);
		mem_free(pMem);
	}
}

inline uint32_t bit_scan_fwd(uint32_t mask) {
	unsigned long idx;
	::_BitScanForward(&idx, mask);
	return (uint32_t)idx;
}

inline uint32_t bit_scan_rev(uint32_t mask) {
	unsigned long idx;
	::_BitScanReverse(&idx, mask);
	return (uint32_t)idx;
}

inline uint32_t clz32(uint32_t x) {
	if (x) return 31 - bit_scan_rev(x);
	return 32;
}

inline uint32_t ctz32(uint32_t x) {
	if (x) return bit_scan_fwd(x);
	return 32;
}

inline uint32_t bit_len32(uint32_t x) {
	return 32 - clz32(x);
}

inline uint32_t pop_count32(uint32_t x) {
#if 1
	return (uint32_t)_mm_popcnt_u32(x);
#else
	uint32_t n = 0;
	while (x) {
		x &= x - 1;
		++n;
	}
	return n;
#endif
}

float f32_set_bits(uint32_t x);
uint32_t f32_get_bits(float x);
inline uint32_t f32_get_exp_bits(float x) { return (f32_get_bits(x) >> 23) & 0xFF; }
inline int f32_get_exp(float x) { return (int)f32_get_exp_bits(x) - 127; }
inline float f32_mk_nan() { return f32_set_bits(0xFFC00000); }

uint32_t fetch_bits32(uint8_t* pTop, uint32_t org, uint32_t len);
uint32_t fetch_bits32_loop(uint8_t* pTop, uint32_t org, uint32_t len);

sxGfxDevice create_def_gfx_device();
sxGfxDevice create_gfx_device_sc(DXGI_SWAP_CHAIN_DESC& desc, IDXGISwapChain** ppSwapChain);
sxHLSLCompilerIfc* get_hlsl_compiler();
void free_hlsl_compiler();

int get_display_freq();

} // nxCore

class cxSharedScratch {
protected:
	char mName[64];
	void* mpMem;
	size_t mMemSize;
	long mCursor;

public:
	cxSharedScratch() : mpMem(nullptr), mMemSize(0), mCursor(0) { set_name("<unnamed>"); }
	~cxSharedScratch() { release(); }

	void set_name(const char* pName);

	void create(size_t memSize);
	void release();

	void purge() { mCursor = 0; }
	void* get_mem(int size);
};

class cxMutex {
protected:
	CRITICAL_SECTION mCS;

public:
	cxMutex() { ::InitializeCriticalSection(&mCS);  }
	~cxMutex() { ::DeleteCriticalSection(&mCS); }

	void enter() { ::EnterCriticalSection(&mCS); }
	void leave() { ::LeaveCriticalSection(&mCS); }
	bool try_enter() { return !!::TryEnterCriticalSection(&mCS); }

	struct LOCK {
		cxMutex* mpMutex;
		LOCK(cxMutex& m) {
			mpMutex = &m;
			m.enter();
		}
		~LOCK() {
			mpMutex->leave();
		}
	};
};

class cxWorker {
protected:
	char mName[64];
	HANDLE mhThread;
	HANDLE mhExec;
	HANDLE mhDone;
	DWORD mTID;
	bool mEndFlg;

	void set_name(const char* pName);

public:
	cxWorker() : mhThread(nullptr), mhExec(nullptr), mhDone(nullptr), mTID(0), mEndFlg(false) {
		mName[0] = 0;
	}

	uint32_t get_thread_id() const { return (uint32_t)mTID; }

	const char* get_name() const { return mName; }
	HANDLE get_done_handle() const { return mhDone; }

	void init(const char* pName = "cxWorker");
	void start() {
		::ResumeThread(mhThread);
		wait();
	}
	void exec() {
		::ResetEvent(mhDone);
		::SetEvent(mhExec);
	}
	void stop() {
		mEndFlg = true;
		exec();
		::WaitForSingleObject(mhThread, INFINITE);
	}
	void reset() {
		::CloseHandle(mhExec);
		::CloseHandle(mhDone);
		::CloseHandle(mhThread);
		mhExec = nullptr;
		mhDone = nullptr;
		mhThread = nullptr;
	}
	void wait() { ::WaitForSingleObject(mhDone, INFINITE); }
	void loop();
	virtual void main() {}
};

struct sxDispatch {
	uint8_t mState[4];

	void clear() { for (int i = 0; i < XD_ARY_LEN(mState); ++i) { mState[i] = 0; } }
	uint8_t operator [](int i) const { return mState[i]; }
	uint8_t& operator [](int i) { return mState[i]; }
};

class cxJob {
protected:
	char mName[64];
	bool mEnable;

public:
	sxDispatch mDispatch;
	uint32_t mTimer;

	struct Context {
		cxJobQueue* mpQueue;
		cxWorkBrigade* mpBrigade;
		int mWorkerId;
	};

public:
	cxJob();
	virtual ~cxJob() {}

	virtual void exec(const Context& ctx) {};

	void enable(bool flg) { mEnable = flg; }
	bool is_enabled() const { return mEnable; }

	void set_name(const char* pName);
	const char* get_name() const { return mName; }

	void set_dispatch(uint8_t v0) {
		mDispatch[0] = v0;
	}
	void set_dispatch(uint8_t v0, uint8_t v1) {
		mDispatch[0] = v0;
		mDispatch[1] = v1;
	}
	void set_dispatch(uint8_t v0, uint8_t v1, uint8_t v2) {
		mDispatch[0] = v0;
		mDispatch[1] = v1;
		mDispatch[2] = v2;
	}
	void set_dispatch(uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3) {
		mDispatch[0] = v0;
		mDispatch[1] = v1;
		mDispatch[2] = v2;
		mDispatch[3] = v3;
	}
	void set_timer(uint16_t val) { mTimer = val; }
};

class cxJobQueue {
protected:
	cxJob** mpJobSlots;
	int32_t mSlotsNum;
	int32_t mPutIdx;
	long mAccessIdx;

public:
	cxJobQueue() : mpJobSlots(nullptr), mSlotsNum(0) {
		clear();
	}

	~cxJobQueue() {
		free();
	}

	void alloc(int num);
	void free();
	void reset() { mAccessIdx = 0; }
	void clear() { mPutIdx = 0; reset(); }
	void put(cxJob* pJob);
	cxJob* get_next_job();
	void exec(cxWorkBrigade* pBigade = nullptr);
};

class cxWorkBrigade {
public:
	class Worker : public cxWorker {
	protected:
		cxWorkBrigade* mpBrigade;
		int mId;
		int mJobsDone;

		friend class cxWorkBrigade;

		Worker() : mpBrigade(nullptr), mId(-1), mJobsDone(0) {}

	public:
		void main();

		int get_id() const { return mId; }
		cxWorkBrigade* get_brigade() const { return mpBrigade; }
		int get_num_jobs_done() const { return mJobsDone; }
	};

protected:
	Worker* mpWrkAry;
	int mWrkNum;
	cxJobQueue* mpQue;

public:
	static const int MAX_WORKERS = 16;

	cxWorkBrigade() : mpWrkAry(nullptr), mWrkNum(0), mpQue(nullptr) {}
	~cxWorkBrigade() { reset();  }

	void init(int wrkNum);
	void reset();
	bool is_valid() const { return (mWrkNum > 0 && mpWrkAry); }
	int get_wrk_num() const { return mWrkNum; }
	Worker* get_worker(int wrkIdx) const { return (uint32_t)wrkIdx < (uint32_t)mWrkNum ? &mpWrkAry[wrkIdx] : nullptr; }
	cxJobQueue* get_queue() const { return mpQue; }
	void set_queue(cxJobQueue* pQue) { mpQue = pQue; }
	void exec() const;
	void wait() const;
};


class cxMouse {
public:
	enum class eBtn {
		LEFT,
		MIDDLE,
		RIGHT,
		X,
		Y
	};

	struct State {
		uint32_t mBtnOld;
		uint32_t mBtnNow;
		int32_t mNowX;
		int32_t mNowY;
		int32_t mOldX;
		int32_t mOldY;
		int32_t mWheel;

		void reset() { ::memset(this, 0, sizeof(State)); }
		bool ck_now(eBtn id) const { return !!(mBtnNow & (1 << (uint32_t)id)); }
		bool ck_old(eBtn id) const { return !!(mBtnOld & (1 << (uint32_t)id)); }
		bool ck_trg(eBtn id) const { return !!((mBtnNow & (mBtnNow ^ mBtnOld)) & (1 << (uint32_t)id)); }
		bool ck_chg(eBtn id) const { return !!((mBtnNow ^ mBtnOld) & (1 << (uint32_t)id)); }
	};

protected:
	State mState;

public:
	cxMouse() {
		mState.reset();
	}
	virtual ~cxMouse() {}

	virtual void operator ()() {}

	State* get_state() { return &mState; }
	void update(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
	virtual void exec() {}
};

class cxAppLoop {
public:
	virtual void operator ()() {}
	virtual void end() {}
};

class cxWindow {
protected:
	HINSTANCE mhInstance;
	HWND mhWnd;
	int mWidth;
	int mHeight;

public:
	cxWindow() {}
	virtual ~cxWindow() {}

	void init(HINSTANCE hInstance, int w, int h, const char* pTitle = nullptr);
	HWND get_hwnd() const { return mhWnd; }
	int get_width() const { return mWidth; }
	int get_height() const { return mHeight; }
	virtual LRESULT wnd_proc(UINT msg, WPARAM wParam, LPARAM lParam);
};

namespace nxWindow {

ATOM register_class(HINSTANCE hInstance);
void unregister_class(HINSTANCE hInstance);
void msg_loop(cxAppLoop& loop);

} // nxWindow



