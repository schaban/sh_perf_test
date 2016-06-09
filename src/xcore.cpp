/*
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#include "xcore.hpp"

#define XD_USE_DXPKV 0
#if XD_USE_DXPKV
#	include <DirectXPackedVector.h>
#endif

#define _XD_SET_XYZW(_x, _y, _z, _w) _mm_set_ps(_w, _z, _y, _x)

namespace nxCore {

static HMODULE s_hD3D11 = nullptr;
static PFN_D3D11_CREATE_DEVICE s_pD3D11CreateDevice = nullptr;
static PFN_D3D11_CREATE_DEVICE_AND_SWAP_CHAIN s_pD3D11CreateDeviceAndSwapChain = nullptr;

void* mem_alloc(size_t size) {
	return ::_aligned_malloc(size, 0x10);
}

void mem_free(void* pMem) {
	::_aligned_free(pMem);
}

void dbg_msg(const char* fmt, ...) {
	char msg[1024 * 2];
	va_list marker;
	va_start(marker, fmt);
	::vsprintf_s(msg, sizeof(msg), fmt, marker);
	va_end(marker);
	::OutputDebugStringA(msg);
}

void attach_console() {
	::AllocConsole();
	::AttachConsole(::GetCurrentProcessId());
	FILE* pConFile;
	::freopen_s(&pConFile, "CON", "w", stdout);
	::freopen_s(&pConFile, "CON", "w", stderr);
}

void set_exe_cwd() {
	LPTSTR pCmd;
	TCHAR path[MAX_PATH];
	TCHAR fullPath[MAX_PATH];
	::memset(path, 0, sizeof(path));
	pCmd = ::GetCommandLine();
	while (*pCmd == '"' || *pCmd == ' ') ++pCmd;
	LPTSTR pOrg = pCmd;
	while (*pCmd && !::iswspace(*pCmd)) ++pCmd;
	LPTSTR pEnd = ::_tcsrchr(pOrg, '\\');
	::_tcsncpy_s(path, XD_ARY_LEN(path), pOrg, pEnd - pOrg);
	::GetFullPathName(path, XD_ARY_LEN(fullPath), fullPath, NULL);
	BOOL res = ::SetCurrentDirectory(fullPath);
	if (!res) {
		dbg_msg("!cwd\n");
	}
}

void set_thread_name(const char* pName) {
	struct {
		DWORD  type;
		LPCSTR pName;
		DWORD  tid;
		DWORD  flg;
	} info;
	info.type = 0x1000;
	info.pName = pName;
	info.tid = -1;
	info.flg = 0;
	__try {
		RaiseException(0x406D1388, 0, sizeof(info) / sizeof(DWORD), (ULONG_PTR*)&info);
	} __except (EXCEPTION_CONTINUE_EXECUTION) {}
}

void fpu_init() {
#if !defined(_WIN64)
#	if defined(_MSC_VER) || defined(__INTEL_COMPILER)
		__asm {
			push eax
			fnstcw word ptr[esp]
			pop eax
			and ah, NOT 3
			push eax
			fldcw word ptr[esp]
			pop eax
		}
#	elif defined (__GNUC__)
		uint16_t fcw;
		asm("fnstcw %w0" : "=m" (fcw));
		fcw &= ~(3 << 8);
		asm("fldcw %w0" : : "m" (fcw));
#	endif
#endif
}

void cpu_serialize() {
#if defined (__GNUC__) || defined(_WIN64)
	int dummy[4];
	__cpuid(dummy, 0);
#else
	__asm {
		sub eax, eax
			cpuid
	}
#endif
}

int64_t get_timestamp() {
	LARGE_INTEGER ctr;
	cpu_serialize();
	::QueryPerformanceCounter(&ctr);
	cpu_serialize();
	return ctr.QuadPart;
}

double get_perf_freq() {
	LARGE_INTEGER freq;
	::QueryPerformanceFrequency(&freq);
	return (double)freq.QuadPart;
}

void* bin_load(const char* pPath, size_t* pSize, bool appendPath) {
	FILE* f;
	void* pData = nullptr;
	size_t size = 0;
	if (0 == ::fopen_s(&f, pPath, "rb") && f) {
		size_t pathLen = ::strlen(pPath);
		long len = 0;
		long old = ::ftell(f);
		if (0 == ::fseek(f, 0, SEEK_END)) {
			len = ::ftell(f);
		}
		::fseek(f, old, SEEK_SET);
		size_t memsize = len;
		if (appendPath) {
			memsize += pathLen + 1;
		}
		pData = mem_alloc(memsize);
		if (pData) {
			::fread(pData, len, 1, f);
		}
		::fclose(f);
		if (appendPath) {
			::strcpy_s(&((char*)pData)[len], pathLen + 1, pPath);
		}
		size = len;
	}
	if (pSize) {
		*pSize = size;
	}
	return pData;
}

void bin_unload(void* pMem) {
	mem_free(pMem);
}

void bin_save(const char* pPath, const void* pMem, size_t size) {
	if (!pPath || !pMem || !size) return;
	FILE* f;
	::fopen_s(&f, pPath, "w+b");
	if (f) {
		::fwrite(pMem, 1, size, f);
		::fclose(f);
	}
}

// http://www.isthe.com/chongo/tech/comp/fnv/index.html
uint32_t str_hash32(const char* pStr) {
	uint32_t h = 2166136261;
	while (true) {
		uint8_t c = *pStr++;
		if (!c) break;
		h *= 16777619;
		h ^= c;
	}
	return h;
}

uint16_t str_hash16(const char* pStr) {
	uint32_t h = str_hash32(pStr);
	return (uint16_t)((h >> 16) ^ (h & 0xFFFF));
}

bool str_eq(const char* pStrA, const char* pStrB) {
	bool res = false;
	if (pStrA && pStrB) {
		res = ::strcmp(pStrA, pStrB) == 0;
	}
	return res;
}

bool str_starts_with(const char* pStr, const char* pPrefix) {
	if (pStr && pPrefix) {
		size_t len = ::strlen(pPrefix);
		for (size_t i = 0; i < len; ++i) {
			char ch = pStr[i];
			if (0 == ch || ch != pPrefix[i]) return false;
		}
		return true;
	}
	return false;
}

bool str_ends_with(const char* pStr, const char* pPostfix) {
	if (pStr && pPostfix) {
		size_t lenStr = ::strlen(pStr);
		size_t lenPost = ::strlen(pPostfix);
		if (lenStr < lenPost) return false;
		for (size_t i = 0; i < lenPost; ++i) {
			if (pStr[lenStr - lenPost + i] != pPostfix[i]) return false;
		}
		return true;
	}
	return false;
}

char* str_dup(const char* pSrc) {
	char* pDst = nullptr;
	if (pSrc) {
		size_t len = ::strlen(pSrc) + 1;
		pDst = reinterpret_cast<char*>(mem_alloc(len));
		::memcpy(pDst, pSrc, len);
	}
	return pDst;
}


float f32_set_bits(uint32_t x) {
	uxVal32 v;
	v.u = x;
	return v.f;
}

uint32_t f32_get_bits(float x) {
	uxVal32 v;
	v.f = x;
	return v.u;
}


uint32_t fetch_bits32(uint8_t* pTop, uint32_t org, uint32_t len) {
	uint32_t res;
	uint32_t idx = org >> 3;
	__m128i tmp = _mm_cvtsi32_si128(pTop[idx]);
	tmp = _mm_or_si128(tmp, _mm_sll_epi64(_mm_cvtsi32_si128(pTop[idx + 1]), _mm_cvtsi32_si128(8 * 1)));
	tmp = _mm_or_si128(tmp, _mm_sll_epi64(_mm_cvtsi32_si128(pTop[idx + 2]), _mm_cvtsi32_si128(8 * 2)));
	tmp = _mm_or_si128(tmp, _mm_sll_epi64(_mm_cvtsi32_si128(pTop[idx + 3]), _mm_cvtsi32_si128(8 * 3)));
	tmp = _mm_or_si128(tmp, _mm_sll_epi64(_mm_cvtsi32_si128(pTop[idx + 4]), _mm_cvtsi32_si128(8 * 4)));
	tmp = _mm_srl_epi64(tmp, _mm_cvtsi32_si128(org & 7));
	res = _mm_cvtsi128_si32(tmp);
	return res & ((1 << len) - 1);
}

uint32_t fetch_bits32_loop(uint8_t* pTop, uint32_t org, uint32_t len) {
	int i;
	uint32_t res;
	uint32_t idx = org >> 3;
	int cnt = len + 8;
	int n = (cnt >> 3) + ((cnt & 7) ? 1 : 0);
	__m128i tmp = _mm_setzero_si128();
	for (i = 0; i < n; ++i) {
		__m128i next = _mm_cvtsi32_si128(pTop[idx + i]);
		next = _mm_sll_epi64(next, _mm_cvtsi32_si128(8 * i));
		tmp = _mm_or_si128(tmp, next);
	}
	tmp = _mm_srl_epi64(tmp, _mm_cvtsi32_si128(org & 7));
	res = _mm_cvtsi128_si32(tmp);
	return res & ((1 << len) - 1);
}

static void d3d11_init() {
	if (!s_hD3D11) {
		s_hD3D11 = ::LoadLibraryW(L"d3d11.dll");
		if (s_hD3D11) {
			*(FARPROC*)&s_pD3D11CreateDevice = ::GetProcAddress(s_hD3D11, "D3D11CreateDevice");
			*(FARPROC*)&s_pD3D11CreateDeviceAndSwapChain = ::GetProcAddress(s_hD3D11, "D3D11CreateDeviceAndSwapChain");
		}
	}
}

sxGfxDevice create_def_gfx_device() {
	sxGfxDevice dev;
	dev.mpDev = nullptr;
	dev.mpCtx = nullptr;
	d3d11_init();
	if (s_pD3D11CreateDevice) {
		HRESULT hres = s_pD3D11CreateDevice(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, NULL, 0, D3D11_SDK_VERSION, &dev.mpDev, NULL, &dev.mpCtx);
		if (FAILED(hres)) {
			dev.mpDev = nullptr;
			dev.mpCtx = nullptr;
		}
	}
	return dev;
}

sxGfxDevice create_gfx_device_sc(DXGI_SWAP_CHAIN_DESC& desc, IDXGISwapChain** ppSwapChain) {
	sxGfxDevice dev;
	dev.mpDev = nullptr;
	dev.mpCtx = nullptr;
	d3d11_init();
	if (s_pD3D11CreateDeviceAndSwapChain) {
		HRESULT hres = s_pD3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, NULL, 0, D3D11_SDK_VERSION, &desc, ppSwapChain, &dev.mpDev, NULL, &dev.mpCtx);
		if (FAILED(hres)) {
			dev.mpDev = nullptr;
			dev.mpCtx = nullptr;
		}
	}
	return dev;
}

int get_display_freq() {
	int freq = 60;
	DEVMODE devmode;
	::ZeroMemory(&devmode, sizeof(DEVMODE));
	devmode.dmSize = sizeof(DEVMODE);
	if (::EnumDisplaySettings(nullptr, ENUM_CURRENT_SETTINGS, &devmode)) {
		freq = devmode.dmDisplayFrequency;
	}
	return freq;
}

} // nxCore

static void set_obj_name(char* pDst, size_t dstSize, const char* pName) {
	::memset(pDst, 0, dstSize);
	if (!pName) return;
	size_t slen = ::strlen(pName);
	if (!slen) return;
	const size_t cmax = dstSize - 1;
	if (slen > cmax) slen = cmax;
	::memcpy(pDst, pName, slen);
}


static inline float tex_scroll(float val, float add) {
	val += add;
	if (val < 0.0f) val += 1.0f;
	if (val > 1.0f) val -= 1.0f;
	return val;
}

void xt_texcoord::scroll(const xt_texcoord& step) {
	u = tex_scroll(u, step.u);
	v = tex_scroll(v, step.v);
}

void xt_texcoord::encode_half(uint16_t* pDst) const {
#if XD_USE_DXPKV
	DirectX::PackedVector::XMHALF2 h(u, v);
	pDst[0] = h.x;
	pDst[1] = h.y;
#else
	int h = _mm_cvtsi128_si32(_mm_cvtps_ph(_XD_SET_XYZW(u, v, u, v), 0 /* round */));
	pDst[0] = h & 0xFFFF;
	pDst[1] = (h >> 16) & 0xFFFF;
#endif
}


IDXGISwapChain* sxGfxDevice::create_swap_chain(DXGI_SWAP_CHAIN_DESC& desc) const {
	IDXGISwapChain* pSwp = nullptr;
	if (is_valid()) {
		IDXGIDevice* pDXGIDev = nullptr;
		HRESULT hres = mpDev->QueryInterface(__uuidof(IDXGIDevice), (void**)&pDXGIDev);
		if (SUCCEEDED(hres)) {
			IDXGIAdapter* pAdapter = nullptr;
			hres = pDXGIDev->GetAdapter(&pAdapter);
			if (SUCCEEDED(hres)) {
				IDXGIFactory* pFct = nullptr;
				hres = pAdapter->GetParent(__uuidof(IDXGIFactory), (void**)&pFct);
				if (SUCCEEDED(hres)) {
					hres = pFct->CreateSwapChain(mpDev, &desc, &pSwp);
					if (FAILED(hres)) {
						nxCore::dbg_msg("Can't create swap chain.\n");
						pSwp = nullptr;
					}
				}
			}
		}
	}
	return pSwp;
}


void cxSharedScratch::create(size_t memSize) {
	if (memSize > 0) {
		memSize = XD_ALIGN(memSize, 0x10);
		mpMem = nxCore::mem_alloc(memSize);
		if (mpMem) {
			mMemSize = memSize;
		}
	}
	mCursor = 0;
}

void cxSharedScratch::release() {
	if (mpMem) {
		nxCore::mem_free(mpMem);
		mpMem = nullptr;
	}
	mMemSize = 0;
	mCursor = 0;
}

void cxSharedScratch::set_name(const char* pName) {
	set_obj_name(mName, sizeof(mName), pName);
}

void* cxSharedScratch::get_mem(int size) {
	void* pMem = nullptr;
	if (mpMem && size > 0) {
		size = XD_ALIGN(size, 0x10);
		int cur = ::_InterlockedExchangeAdd(&mCursor, size);
		if ((size_t)cur + size <= mMemSize) {
			pMem = XD_INCR_PTR(mpMem, cur);
		} else {
			::_InterlockedAdd(&mCursor, -size);
			nxCore::dbg_msg("cxSharedScratch[%s]: out of memory.\n", mName);
		}
	}
	return pMem;
}


static DWORD APIENTRY wrk_entry(void* pData) {
	nxCore::fpu_init();
	cxWorker* pWrk = reinterpret_cast<cxWorker*>(pData);
	pWrk->loop();
	return 0;
}

void cxWorker::set_name(const char* pName) {
	set_obj_name(mName, sizeof(mName), pName);
}

void cxWorker::init(const char* pName) {
	set_name(pName);
	mhThread = ::CreateThread(NULL, 0, wrk_entry, this, CREATE_SUSPENDED, &mTID);
	mhExec = ::CreateEvent(NULL, FALSE, FALSE, NULL);
	mhDone = ::CreateEvent(NULL, FALSE, FALSE, NULL);
	mEndFlg = false;
}

void cxWorker::loop() {
	nxCore::set_thread_name(mName);
	::SetEvent(mhDone);
	while (!mEndFlg) {
		if (::WaitForSingleObject(mhExec, INFINITE) == WAIT_OBJECT_0) {
			::ResetEvent(mhExec);
			main();
			::SetEvent(mhDone);
		}
	}
}


cxJob::cxJob() : mEnable(true) {
	char name[64];
	::sprintf_s(name, sizeof(name), "cxJob@%p", this);
	set_name(name);
	set_timer(0);
}

void cxJob::set_name(const char* pName) {
	set_obj_name(mName, sizeof(mName), pName);
}


void cxJobQueue::alloc(int num) {
	if (num > 0) {
		int memsize = sizeof(cxJob*) * num;
		cxJob** pSlots = reinterpret_cast<cxJob**>(nxCore::mem_alloc(memsize));
		if (pSlots) {
			::ZeroMemory(pSlots, memsize);
			mpJobSlots = pSlots;
			mSlotsNum = num;
		}
		clear();
	}
}

void cxJobQueue::free() {
	if (mpJobSlots) {
		nxCore::mem_free(mpJobSlots);
		mpJobSlots = nullptr;
		mSlotsNum = 0;
		clear();
	}
}

void cxJobQueue::put(cxJob* pJob) {
	if (mpJobSlots) {
		if (mPutIdx < mSlotsNum) {
			mpJobSlots[mPutIdx] = pJob;
			++mPutIdx;
		} else {
			nxCore::dbg_msg("cxJobQueue::put() - out of free slots.\n");
		}
	}
}

cxJob* cxJobQueue::get_next_job() {
	cxJob* pJob = nullptr;
	int count = mPutIdx;
	if (count > 0) {
		long* pIdx = &mAccessIdx;
		if (*pIdx < count) {
			long idx = ::_InterlockedIncrement(pIdx);
			if (idx <= count) {
				pJob = mpJobSlots[idx - 1];
			}
		}
	}
	return pJob;
}

void cxJobQueue::exec(cxWorkBrigade* pBigade) {
	if (pBigade && pBigade->is_valid()) {
		pBigade->set_queue(this);
		pBigade->exec();
		pBigade->wait();
	} else {
		cxJob::Context ctx;
		ctx.mpQueue = this;
		ctx.mpBrigade = nullptr;
		ctx.mWorkerId = -1;
		int num = mPutIdx;
		for (int i = 0; i < num; ++i) {
			cxJob* pJob = get_next_job();
			if (!pJob) break;
			if (pJob->is_enabled()) {
				pJob->exec(ctx);
			}
		}
	}
	reset();
}


void cxWorkBrigade::Worker::main() {
	if (!mpBrigade) return;
	cxJobQueue* pQue = mpBrigade->get_queue();
	if (!pQue) return;
	cxJob::Context ctx;
	ctx.mpQueue = pQue;
	ctx.mpBrigade = mpBrigade;
	ctx.mWorkerId = mId;
	while (true) {
		cxJob* pJob = pQue->get_next_job();
		if (!pJob) break;
		if (pJob->is_enabled()) {
			pJob->exec(ctx);
		}
		++mJobsDone;
	}
}

void cxWorkBrigade::init(int wrkNum) {
	if (wrkNum < 1) wrkNum = 1;
	if (wrkNum > MAX_WORKERS) wrkNum = MAX_WORKERS;
	size_t memsize = wrkNum * sizeof(Worker);
	Worker* pWrk = reinterpret_cast<Worker*>(nxCore::mem_alloc(memsize));
	if (pWrk) {
		for (int i = 0; i < wrkNum; ++i) {
			::new(&pWrk[i]) Worker;
		}
		mWrkNum = wrkNum;
		mpWrkAry = pWrk;
		for (int i = 0; i < wrkNum; ++i) {
			mpWrkAry[i].init();
		}
		for (int i = 0; i < wrkNum; ++i) {
			mpWrkAry[i].mpBrigade = this;
			mpWrkAry[i].mId = i;
		}
		for (int i = 0; i < wrkNum; ++i) {
			mpWrkAry[i].start();
		}
	}
}

void cxWorkBrigade::reset() {
	if (is_valid()) {
		for (int i = 0; i < mWrkNum; ++i) {
			mpWrkAry[i].stop();
		}
		for (int i = 0; i < mWrkNum; ++i) {
			mpWrkAry[i].reset();
		}
		nxCore::mem_free(mpWrkAry);
		mWrkNum = 0;
		mpWrkAry = nullptr;
	}
}

void cxWorkBrigade::exec() const {
	if (is_valid()) {
		for (int i = 0; i < mWrkNum; ++i) {
			mpWrkAry[i].mJobsDone = 0;
		}
		for (int i = 0; i < mWrkNum; ++i) {
			mpWrkAry[i].exec();
		}
	}
}

void cxWorkBrigade::wait() const {
	if (is_valid()) {
		HANDLE hlist[MAX_WORKERS];
		for (int i = 0; i < mWrkNum; ++i) {
			hlist[i] = mpWrkAry[i].get_done_handle();
		}
		::WaitForMultipleObjects(mWrkNum, hlist, TRUE, INFINITE);
	}
}


void cxMouse::update(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	static UINT mouseMsg[] = {
		WM_LBUTTONDOWN, WM_LBUTTONUP, WM_LBUTTONDBLCLK,
		WM_MBUTTONDOWN, WM_MBUTTONUP, WM_MBUTTONDBLCLK,
		WM_RBUTTONDOWN, WM_RBUTTONUP, WM_RBUTTONDBLCLK,
		WM_XBUTTONDOWN, WM_XBUTTONUP, WM_XBUTTONDBLCLK,
		WM_MOUSEWHEEL, WM_MOUSEMOVE
	};
	bool mouseFlg = false;
	for (int i = 0; i < XD_ARY_LEN(mouseMsg); ++i) {
		if (mouseMsg[i] == msg) {
			mouseFlg = true;
			break;
		}
	}
	if (mouseFlg) {
		int32_t posX = (int32_t)LOWORD(lParam);
		int32_t posY = (int32_t)HIWORD(lParam);
		mState.mWheel = 0;
		if (WM_MOUSEWHEEL == msg) {
			POINT wpt;
			wpt.x = posX;
			wpt.y = posY;
			::ScreenToClient(hWnd, &wpt);
			mState.mWheel = (int16_t)HIWORD(wParam);
		}
		uint32_t btnMask = LOWORD(wParam);
		static uint32_t mouseMask[] = {
			MK_LBUTTON, MK_MBUTTON, MK_RBUTTON, MK_XBUTTON1, MK_XBUTTON2
		};
		uint32_t mask = 0;
		for (int i = 0; i < XD_ARY_LEN(mouseMask); ++i) {
			if (btnMask & mouseMask[i]) {
				mask |= 1 << i;
			}
		}
		mState.mBtnOld = mState.mBtnNow;
		mState.mBtnNow = mask;
		mState.mOldX = mState.mNowX;
		mState.mOldY = mState.mNowY;
		mState.mNowX = posX;
		mState.mNowY = posY;
	}
}


LRESULT cxWindow::wnd_proc(UINT msg, WPARAM wParam, LPARAM lParam) {
	LRESULT res = 0;
	HWND hWnd = get_hwnd();

	switch (msg) {
		case WM_DESTROY:
			::PostQuitMessage(0);
			break;
		default:
			res = ::DefWindowProc(hWnd, msg, wParam, lParam);
			break;
	}

	return res;
}


namespace nxWindow {

static const TCHAR* s_className = _T("cxWindow");

static LRESULT CALLBACK wnd_proc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	LRESULT res = 0;

	cxWindow* pWnd = nullptr;
	if (msg == WM_CREATE) {
		pWnd = reinterpret_cast<cxWindow*>(((CREATESTRUCT*)lParam)->lpCreateParams);
		::SetWindowLongPtr(hWnd, 0, (LONG_PTR)pWnd);
	} else {
		pWnd = reinterpret_cast<cxWindow*>(::GetWindowLongPtr(hWnd, 0));
	}

	if (pWnd && pWnd->get_hwnd() == hWnd) {
		res = pWnd->wnd_proc(msg, wParam, lParam);
	} else {
		res = ::DefWindowProc(hWnd, msg, wParam, lParam);
	}

	return res;
}

ATOM register_class(HINSTANCE hInstance) {
	WNDCLASSEX wc;
	::ZeroMemory(&wc, sizeof(WNDCLASSEX));
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.style = CS_VREDRAW | CS_HREDRAW;
	wc.hInstance = hInstance;
	wc.hCursor = ::LoadCursor(0, IDC_ARROW);
	wc.hbrBackground = (HBRUSH)::GetStockObject(BLACK_BRUSH);
	wc.lpszClassName = s_className;
	wc.lpfnWndProc = wnd_proc;
	wc.cbWndExtra = 0x10;
	ATOM atom = ::RegisterClassEx(&wc);
	return atom;
}

void unregister_class(HINSTANCE hInstance) {
	::UnregisterClass(s_className, hInstance);
}

void msg_loop(cxAppLoop& loop) {
	MSG msg;
	bool done = false;
	while (!done) {
		if (::PeekMessage(&msg, 0, 0, 0, PM_NOREMOVE)) {
			if (::GetMessage(&msg, NULL, 0, 0)) {
				::TranslateMessage(&msg);
				::DispatchMessage(&msg);
			} else {
				done = true;
				loop.end();
				break;
			}
		} else {
			loop();
		}
	}
}

} // nxWindow


void cxWindow::init(HINSTANCE hInstance, int w, int h, const char* pTitle) {
	mhInstance = hInstance;
	mWidth = w;
	mHeight = h;

	int style = WS_CLIPCHILDREN | WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_GROUP;
	RECT rect;
	rect.left = 0;
	rect.top = 0;
	rect.right = w;
	rect.bottom = h;
	::AdjustWindowRect(&rect, style, FALSE);
	int wndW = rect.right - rect.left;
	int wndH = rect.bottom - rect.top;

	TCHAR title[128];
	::ZeroMemory(title, sizeof(title));
	if (pTitle) {
		const char* pSrc = pTitle;
		TCHAR* pDst = title;
		while (true) {
			TCHAR ch = *pSrc++;
			if (ch) {
				*pDst++ = ch;
			} else {
				break;
			}
		}
	} else {
		::_stprintf_s(title, XD_ARY_LEN(title), _T("%s: build %s"), _T("XWND"), _T(__DATE__));
	}
	mhWnd = ::CreateWindowEx(0, nxWindow::s_className, title, style, 0, 0, wndW, wndH, NULL, NULL, hInstance, this);
	if (mhWnd) {
		::ShowWindow(mhWnd, SW_SHOW);
		::UpdateWindow(mhWnd);
	}
}



