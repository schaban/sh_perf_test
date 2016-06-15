/*
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <intrin.h>

#include <DirectXMath.h>
using namespace DirectX;

#define XD_CALC_OPT 1
#ifndef XD_CALC_OPT
#	define XD_CALC_OPT 0
#endif

#define XD_EXT_CPU 1
#ifndef XD_EXT_CPU
#	define XD_EXT_CPU 0
#endif

#ifndef XD_INLINE
#	define XD_INLINE __forceinline
#endif

#ifndef XD_NOINLINE
#	define XD_NOINLINE __declspec(noinline) 
#endif

#define XD_MIX_MASK(_ix0, _iy0, _iz1, _iw1) ( (_ix0)|((_iy0)<<2)|((_iz1)<<4)|((_iw1)<<6) )
#define XD_ELEM_MASK(_idx) XD_MIX_MASK(_idx, _idx, _idx, _idx)
#define XD_SHUF(_v, _ix, _iy, _iz, _iw) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128((_v)), XD_MIX_MASK(_ix, _iy, _iz, _iw)))

#define XD_DEG2RAD(_deg) XMConvertToRadians(_deg)
#define XD_RAD2DEG(_rad) XMConvertToDegrees(_rad)

#define XD_PI ((float)M_PI)

class cxVec;
class cxMtx;
class cxQuat;
class cxPlane;
class cxSphere;
class cxAABB;

enum class exTransformOrd : uint8_t {
	SRT = 0,
	STR = 1,
	RST = 2,
	RTS = 3,
	TSR = 4,
	TRS = 5
};

enum class exRotOrd : uint8_t {
	XYZ = 0,
	XZY = 1,
	YXZ = 2,
	YZX = 3,
	ZXY = 4,
	ZYX = 5
};

enum class exAxis : uint8_t {
	PLUS_X  = 0,
	MINUS_X = 1,
	PLUS_Y  = 2,
	MINUS_Y = 3,
	PLUS_Z  = 4,
	MINUS_Z = 5
};

namespace nxCalc {

template<typename T> inline T min(T x, T y) { return (x < y ? x : y); }
template<typename T> inline T min(T x, T y, T z) { return min(x, min(y, z)); }
template<typename T> inline T min(T x, T y, T z, T w) { return min(x, min(y, z, w)); }
template<typename T> inline T max(T x, T y) { return (x > y ? x : y); }
template<typename T> inline T max(T x, T y, T z) { return max(x, max(y, z)); }
template<typename T> inline T max(T x, T y, T z, T w) { return max(x, max(y, z, w)); }

template<typename T> inline T clamp(T x, T lo, T hi) { return max(min(x, hi), lo); }

inline float min(float x, float y) { return _mm_cvtss_f32(_mm_min_ss(_mm_set_ss(x), _mm_set_ss(y))); }
inline float max(float x, float y) { return _mm_cvtss_f32(_mm_max_ss(_mm_set_ss(x), _mm_set_ss(y))); }
inline double min(double x, double y) { return _mm_cvtsd_f64(_mm_min_sd(_mm_set_sd(x), _mm_set_sd(y))); }
inline double max(double x, double y) { return _mm_cvtsd_f64(_mm_max_sd(_mm_set_sd(x), _mm_set_sd(y))); }

#if XD_CALC_OPT && XD_EXT_CPU
inline float floor(float x) {
	__m128 t = _mm_set_ss(x);
	return _mm_cvtss_f32(_mm_round_ss(t, t, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC));
}

inline float ceil(float x) {
	__m128 t = _mm_set_ss(x);
	return _mm_cvtss_f32(_mm_round_ss(t, t, _MM_FROUND_TO_POS_INF | _MM_FROUND_NO_EXC));
}

inline float round(float x) {
	__m128 t = _mm_set_ss(x);
	return _mm_cvtss_f32(_mm_round_ss(t, t, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
}

inline float trunc(float x) {
	__m128 t = _mm_set_ss(x);
	return _mm_cvtss_f32(_mm_round_ss(t, t, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC));
}
#else
inline float floor(float x) { return ::floorf(x); }
inline float ceil(float x) { return ::ceilf(x); }
inline float round(float x) { return ::roundf(x); }
inline float trunc(float x) { return ::truncf(x); }
#endif

inline float clamp(float x, float lo, float hi) { return max(min(x, hi), lo); }
inline float saturate(float x) { return clamp(x, 0.0f, 1.0f); }

inline float mod_pi(float x) { return XMScalarModAngle(x); }

inline float sinc(float x) {
	if (::fabsf(x) < 1.0e-4f) {
		return 1.0f;
	}
	return (::sinf(x) / x);
}

inline int sq(int x) { return x*x; }
inline float sq(float x) { return x*x; }
inline float cb(float x) { return x*x*x; }

inline float div0(float x, float y) {
	XMVECTOR vy = _mm_set_ss(y);
	XMVECTOR m = _mm_cmpneq_ss(vy, _mm_setzero_ps());
	XMVECTOR d = _mm_div_ss(_mm_set_ss(x), vy);
	return _mm_cvtss_f32(_mm_and_ps(d, m));
}

inline float rcp0(float x) { return div0(1.0f, x); }

inline float lerp(float a, float b, float t) { return a + (b - a)*t; }

float hermite(float p0, float m0, float p1, float m1, float t);
float bezier01(float t, float p1, float p2);
float fit(float val, float oldMin, float oldMax, float newMin, float newMax);

float calc_fovy(float focal, float aperture, float aspect);

} // nxCalc


namespace nxVec {

#if XD_CALC_OPT
inline float dot4(XMVECTOR v0, XMVECTOR v1) {
	XMVECTOR d;
# if XD_EXT_CPU
	d = _mm_dp_ps(v0, v1, 0xF1);
# else
	d = _mm_mul_ps(v0, v1);
	d = _mm_hadd_ps(d, d);
	d = _mm_hadd_ps(d, d);
# endif
	return _mm_cvtss_f32(d);
}
#else /* XD_CALC_OPT */
inline float dot4(XMVECTOR v0, XMVECTOR v1) {
	float res;
	XMStoreFloat(&res, XMVector4Dot(v0, v1));
	return res;
}
#endif /* XD_CALC_OPT */

inline double dot4_d(XMVECTOR v0, XMVECTOR v1) {
	__m128d d0 = _mm_mul_pd(_mm_cvtps_pd(v0), _mm_cvtps_pd(v1));
	__m128d d1 = _mm_mul_pd(_mm_cvtps_pd(_mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(v0), 8))),
		_mm_cvtps_pd(_mm_castsi128_ps(_mm_srli_si128(_mm_castps_si128(v1), 8))));
	__m128d d = _mm_hadd_pd(d0, d1);
	d = _mm_hadd_pd(d, d);
	return _mm_cvtsd_f64(d);
}

inline XMVECTOR div0(XMVECTOR x, XMVECTOR y) {
	XMVECTOR m = _mm_cmpneq_ps(y, _mm_setzero_ps());
	XMVECTOR d = _mm_div_ps(x, y);
	return _mm_and_ps(d, m);
}

inline XMVECTOR rcp0(XMVECTOR x) {
	return div0(XMVectorReplicate(1.0f), x);
}

inline void transpose3(XMVECTOR* pDst, const XMVECTOR* pSrc) {
	__m128 r0 = pSrc[0];
	__m128 r1 = pSrc[1];
	__m128 r2 = pSrc[2];
	__m128 r3 = _mm_setzero_ps();
	__m128 t0 = _mm_unpacklo_ps(r0, r1);
	__m128 t1 = _mm_unpackhi_ps(r0, r1);
	__m128 t2 = _mm_unpacklo_ps(r2, r3);
	__m128 t3 = _mm_unpackhi_ps(r2, r3);
	pDst[0] = _mm_movelh_ps(t0, t2);
	pDst[1] = _mm_movehl_ps(t2, t0);
	pDst[2] = _mm_movelh_ps(t1, t3);
}

float dist2(const cxVec& pos0, const cxVec& pos1);
float dist(const cxVec& pos0, const cxVec& pos1);

cxVec get_axis(exAxis axis);
cxVec basis_xform(const cxVec& ax, const cxVec& ay, const cxVec& az, const cxVec& vec);
cxVec from_polar_uv(float u, float v);

} // nxVec


class cxVec : public XMFLOAT3 {
protected:
#if XD_CALC_OPT
	/* x0yz */
	XMVECTOR xload() const { return _mm_loadh_pi(_mm_load_ss(&this->x), (__m64 const*)(&this->y)); }
	void xstore(XMVECTOR xv) { _mm_store_ss(&this->x, xv); _mm_storeh_pi((__m64*)(&this->y), xv); }
#else
	/* xyz0 */
	XMVECTOR xload() const { return XMLoadFloat3(this); }
	void xstore(XMVECTOR xv) { XMStoreFloat3(this, xv); }
#endif

public:
	cxVec() {}
	cxVec(float x, float y, float z) : XMFLOAT3(x, y, z) {}
	cxVec(float s) { fill(s); }
	cxVec(XMVECTOR xv) { set_xv(xv); }

#if 1
	cxVec(const cxVec& v) { xstore(v.xload()); }

	cxVec& operator=(const cxVec& v) {
		xstore(v.xload());
		return *this;
	}
#else
	cxVec(const cxVec& v) = default;
	cxVec& operator=(const cxVec& v) = default;
#endif

	void set(float x, float y, float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	void fill(float s) { xstore(XMVectorReplicate(s)); }
	void zero() { xstore(XMVectorZero()); }

	void parse(const char* pStr);
	void from_mem(const float* pSrc) { set(pSrc[0], pSrc[1], pSrc[2]); }
	void to_mem(float* pDst) {
		pDst[0] = x;
		pDst[1] = y;
		pDst[2] = z;
	}

	float get_at(int i) const { return XMVectorGetByIndex(get_xv(), i); }
	void set_at(int i, float val) { set_xv(XMVectorSetByIndex(get_xv(), val, i)); }
	float operator [](int i) const { return get_at(i); }

	XMVECTOR get_xv() const { return XMLoadFloat3(this); }
	void set_xv(XMVECTOR xv) { XMStoreFloat3(this, xv); }
	XMVECTOR get_xv_pnt() const { return XMVectorSet(x, y, z, 1.0f); }

	void add(const cxVec& v) { xstore(xload() + v.xload()); }
	void sub(const cxVec& v) { xstore(xload() - v.xload()); }
	void mul(const cxVec& v) { xstore(xload() * v.xload()); }
	void div(const cxVec& v) { xstore(xload() / v.xload()); }
	void scl(float s) { xstore(xload() * s); }
	void neg() { xstore(-xload()); }
	void abs() { xstore(XMVectorAbs(xload())); }

	cxVec neg_val() const {
		cxVec v = *this;
		v.neg();
		return v;
	}

	cxVec abs_val() const {
		cxVec v = *this;
		v.abs();
		return v;
	}

	cxVec& operator += (const cxVec& v) { add(v); return *this; }
	cxVec& operator -= (const cxVec& v) { sub(v); return *this; }
	cxVec& operator *= (const cxVec& v) { mul(v); return *this; }
	cxVec& operator /= (const cxVec& v) { div(v); return *this; }

	cxVec& operator *= (float s) { scl(s); return *this; }
	cxVec& operator /= (float s) { scl(1.0f / s); return *this; }

	void div0(const cxVec& v) {
		XMVECTOR a = xload();
		XMVECTOR b = v.xload();
		XMVECTOR m = _mm_cmpneq_ps(b, _mm_setzero_ps());
		XMVECTOR d = a / b;
		xstore(_mm_and_ps(d, m));
	}

	void div0(float s) { scl(s != 0.0f ? 1.0f / s : 0.0f); }

	cxVec inv_val() const {
		cxVec v(1.0f);
		v.div0(*this);
		return v;
	}
	void inv() { *this = inv_val(); }

	float min_elem(bool absFlg) const {
		XMVECTOR xv = XMLoadFloat3(this);
		if (absFlg) xv = XMVectorAbs(xv);
		float res;
		xv = _mm_min_ss(xv, _mm_min_ss(XMVectorSplatY(xv), XMVectorSplatZ(xv)));
		XMStoreFloat(&res, xv);
		return res;
	}

	float max_elem(bool absFlg) const {
		XMVECTOR xv = XMLoadFloat3(this);
		if (absFlg) xv = XMVectorAbs(xv);
		float res;
		xv = _mm_max_ss(xv, _mm_max_ss(XMVectorSplatY(xv), XMVectorSplatZ(xv)));
		XMStoreFloat(&res, xv);
		return res;
	}

	float mag2() const { return dot(*this);	}
	float mag_fast() const { return ::sqrtf(mag2()); }
	float mag() const;
	float length() const { return mag(); }

	void normalize(const cxVec& v);
	void normalize() { normalize(*this); }
	cxVec get_normalized() const {
		cxVec n;
		n.normalize(*this);
		return n;
	}

	float azimuth() const { return ::atan2f(x, z); }
	float elevation() const { return -::atan2f(y, ::hypotf(x, z)); }

	void min(const cxVec& v) { xstore(XMVectorMin(xload(), v.xload())); }
	void min(const cxVec& v1, const cxVec& v2) { xstore(XMVectorMin(v1.xload(), v2.xload())); }
	void max(const cxVec& v) { xstore(XMVectorMax(xload(), v.xload())); }
	void max(const cxVec& v1, const cxVec& v2) { xstore(XMVectorMax(v1.xload(), v2.xload())); }

	void clamp(const cxVec& v, const cxVec& vmin, const cxVec& vmax) { xstore(XMVectorClamp(v.xload(), vmin.xload(), vmax.xload())); }
	void clamp(const cxVec& vmin, const cxVec& vmax) { xstore(XMVectorClamp(xload(), vmin.xload(), vmax.xload())); }
	void saturate(const cxVec& v) { xstore(XMVectorSaturate(v.xload())); }
	void saturate() { xstore(XMVectorSaturate(xload())); }

	cxVec get_clamped(const cxVec& vmin, const cxVec& vmax) const {
		cxVec v;
		v.clamp(*this, vmin, vmax);
		return v;
	}

	cxVec get_saturated() const {
		cxVec v;
		v.saturate(*this);
		return v;
	}

#if XD_CALC_OPT
	float dot(const cxVec& v) const {
		float res;
		XMVECTOR d;
		XMVECTOR a = xload();
		XMVECTOR b = v.xload();
# if XD_EXT_CPU
		d = _mm_dp_ps(a, b, 0xD1);
# else
		d = _mm_mul_ps(a, b);
		d = _mm_hadd_ps(d, d);
		d = _mm_hadd_ps(d, d);
# endif
		XMStoreFloat(&res, d);
		return res;
	}
#else /* XD_CALC_OPT */
	float dot(const cxVec& v) const {
		float res;
		XMStoreFloat(&res, XMVector3Dot(xload(), v.xload()));
		return res;
	}
#endif /* XD_CALC_OPT */

	double dot_d(const cxVec& v) const { return nxVec::dot4_d(xload(), v.xload()); }

#if XD_CALC_OPT
	void cross(const cxVec& v1, const cxVec& v2) {
		XMVECTOR a = v1.xload();
		XMVECTOR b = v2.xload();
		XMVECTOR c = _mm_sub_ps(
				_mm_mul_ps(XD_SHUF(a, 2, 1, 3, 0), XD_SHUF(b, 3, 1, 0, 2)),
				_mm_mul_ps(XD_SHUF(a, 3, 1, 0, 2), XD_SHUF(b, 2, 1, 3, 0)));
		xstore(c);
	}
#else
	void cross(const cxVec& v1, const cxVec& v2) { xstore(XMVector3Cross(v1.xload(), v2.xload())); }
#endif


	void lerp(const cxVec& v1, const cxVec& v2, float t) {
		xstore(XMVectorLerp(v1.xload(), v2.xload(), t));
	}

	XMFLOAT2 encode_octa() const;
	void decode_octa(const XMFLOAT2& oct);
};

inline cxVec operator + (const cxVec& v1, const cxVec& v2) { cxVec v = v1; v.add(v2); return v; }
inline cxVec operator - (const cxVec& v1, const cxVec& v2) { cxVec v = v1; v.sub(v2); return v; }
inline cxVec operator * (const cxVec& v1, const cxVec& v2) { cxVec v = v1; v.mul(v2); return v; }
inline cxVec operator / (const cxVec& v1, const cxVec& v2) { cxVec v = v1; v.div(v2); return v; }
inline cxVec operator * (const cxVec& v0, float s) { cxVec v = v0; v.scl(s); return v; }
inline cxVec operator * (float s, const cxVec& v0) { cxVec v = v0; v.scl(s); return v; }

namespace nxVec {

inline cxVec cross(const cxVec& v1, const cxVec& v2) {
	cxVec cv;
	cv.cross(v1, v2);
	return cv;
}

inline float scalar_triple(const cxVec& v0, const cxVec& v1, const cxVec& v2) {
	return cross(v0, v1).dot(v2);
}

inline cxVec lerp(const cxVec& v1, const cxVec& v2, float t) {
	cxVec v;
	v.lerp(v1, v2, t);
	return v;
}

inline cxVec min(const cxVec& v1, const cxVec& v2) {
	cxVec v;
	v.min(v1, v2);
	return v;
}

inline cxVec max(const cxVec& v1, const cxVec& v2) {
	cxVec v;
	v.max(v1, v2);
	return v;
}

} // nxVec


class cxMtx : public XMMATRIX {
protected:
	void xset(const XMMATRIX& xm) { *(XMMATRIX*)this = xm; }
	XMMATRIX xget() const { return *(XMMATRIX*)this; }

public:
	cxMtx() {}
	cxMtx(const XMMATRIX& xm) { xset(xm); }

#if XD_CALC_OPT
	void identity() {
		__m128i* pV = reinterpret_cast<__m128i*>(this);
		__m128i v = _mm_insert_epi16(_mm_setzero_si128(), 0x3F80, 1);
		pV[0] = v;
		v = _mm_slli_si128(v, 4);
		pV[1] = v;
		v = _mm_slli_si128(v, 4);
		pV[2] = v;
		v = _mm_slli_si128(v, 4);
		pV[3] = v;
	}

	void identity_sr() {
		__m128i* pV = reinterpret_cast<__m128i*>(this);
		__m128i v = _mm_insert_epi16(_mm_setzero_si128(), 0x3F80, 1);
		pV[0] = v;
		v = _mm_slli_si128(v, 4);
		pV[1] = v;
		v = _mm_slli_si128(v, 4);
		pV[2] = v;
	}
#else
	void identity() { xset(XMMatrixIdentity()); }

	void identity_sr() {
		r[0] = g_XMIdentityR0;
		r[1] = g_XMIdentityR1;
		r[2] = g_XMIdentityR2;
	}
#endif

	void from_mem(const float* pSrc) { ::memcpy(this, pSrc, sizeof(cxMtx)); }
	void to_mem(float* pDst) const { ::memcpy(pDst, this, sizeof(cxMtx)); }

	XMVECTOR get_row(int i) const { return r[i]; }
	XMVECTOR* get_row_ptr(int i) { return &r[i]; }

	float get_at(int row, int column) const { return XMVectorGetByIndex(get_row(row), column); }
	void set_at(int row, int column, float val) { r[row] = XMVectorSetByIndex(get_row(row), val, column); }

	void cpy_sr(const cxMtx& m) {
		r[0] = m.r[0];
		r[1] = m.r[1];
		r[2] = m.r[2];
	}

	void from_quat(const cxQuat& q);
	void from_quat_and_pos(const cxQuat& qrot, const cxVec& vtrans);
	cxQuat to_quat() const;

	void transpose() { xset(XMMatrixTranspose(xget())); }
	void transpose(const cxMtx& m) { xset(XMMatrixTranspose(m.xget())); }
	void transpose_sr() { nxVec::transpose3(r, r); }
	void transpose_sr(const cxMtx& m) { nxVec::transpose3(r, m.r); r[3] = m.r[3]; }
	void invert();
	void invert(const cxMtx& m);

	void mul(const cxMtx& m);
	void mul(const cxMtx& m1, const cxMtx& m2);

	void add(const cxMtx& m);
	void add(const cxMtx& m1, const cxMtx& m2);
	void sub(const cxMtx& m);
	void sub(const cxMtx& m1, const cxMtx& m2);

	cxVec get_translation() const {
		cxVec tv(get_row(3));
		return tv;
	}

	void set_translation(float tx, float ty, float tz) { r[3] = XMVectorSet(tx, ty, tz, 1.0f); }
	void set_translation(const cxVec& tv) { r[3] = tv.get_xv_pnt(); }
	void mk_translation(const cxVec& tv) { identity_sr(); set_translation(tv); }

	cxMtx get_transposed() const {
		cxMtx m;
		m.transpose(*this);
		return m;
	}

	cxMtx get_transposed_sr() const {
		cxMtx m;
		m.transpose_sr(*this);
		return m;
	}

	cxMtx get_inverted() const {
		cxMtx m;
		m.invert(*this);
		return m;
	}

	void set_rot_frame(const cxVec& axisX, const cxVec& axisY, const cxVec& axisZ) {
		r[0] = axisX.get_xv();
		r[1] = axisY.get_xv();
		r[2] = axisZ.get_xv();
		r[3] = g_XMIdentityR3;
	}

	void set_rot(const cxVec& axis, float ang);
	void set_rot_x(float rx);
	void set_rot_y(float ry);
	void set_rot_z(float rz);
	void set_rot_xyz(float rx, float ry, float rz);
	void set_rot_xzy(float rx, float ry, float rz);
	void set_rot_yxz(float rx, float ry, float rz);
	void set_rot_yzx(float rx, float ry, float rz);
	void set_rot_zxy(float rx, float ry, float rz);
	void set_rot_zyx(float rx, float ry, float rz);
	void set_rot(float rx, float ry, float rz, exRotOrd ord = exRotOrd::XYZ);
	void set_rot_degrees(const cxVec& r, exRotOrd ord = exRotOrd::XYZ);

	bool is_valid_rot(float tol = 1.0e-3f) const;
	cxVec get_rot(exRotOrd ord = exRotOrd::XYZ) const;
	cxVec get_rot_deg(exRotOrd ord = exRotOrd::XYZ) const { return get_rot(ord) * XD_RAD2DEG(1.0f); }

	void mk_scl(float sx, float sy, float sz) { xset(XMMatrixScaling(sx, sy, sz)); }
	void mk_scl(const cxVec& sv) { mk_scl(sv.x, sv.y, sv.z); }
	void mk_scl(float s) { mk_scl(s, s, s); }

	void calc_xform(const cxMtx& mtxT, const cxMtx& mtxR, const cxMtx& mtxS, exTransformOrd ord = exTransformOrd::SRT);

	void mk_view(const cxVec& pos, const cxVec& tgt, const cxVec& upvec);
	void mk_proj(float fovy, float aspect, float znear, float zfar);

	cxVec calc_vec(const cxVec& v) const { return XMVector4Transform(v.get_xv(), *this); }
	cxVec calc_pnt(const cxVec& v) const { return XMVector4Transform(v.get_xv_pnt(), *this); }
	XMVECTOR apply(XMVECTOR xv) const { return XMVector4Transform(xv, *this); }

	void sprintf_py(char* pBuf, size_t bufSize) const;
};

inline cxMtx operator * (const cxMtx& m1, const cxMtx& m2) { cxMtx m = m1; m.mul(m2); return m; }

namespace nxMtx {

inline cxMtx mk_rot_frame(const cxVec& axisX, const cxVec& axisY, const cxVec& axisZ) {
	cxMtx m;
	m.set_rot_frame(axisX, axisY, axisZ);
	return m;
}

} // nxMtx


class cxQuat : public XMFLOAT4A {
protected:
	XMVECTOR xload() const { return XMLoadFloat4A(this); }
	void xstore(XMVECTOR xv) { XMStoreFloat4A(this, xv); }

public:
	cxQuat() {}
	cxQuat(float x, float y, float z, float w) : XMFLOAT4A(x, y, z, w) {}
	cxQuat(const cxQuat& q) { xstore(q.xload()); }

	void identity() { xstore(XMQuaternionIdentity()); }

	float get_at(int i) const { return XMVectorGetByIndex(xload(), i); }
	void set_at(int i, float val) { xstore(XMVectorSetByIndex(xload(), val, i)); }

	void from_mtx(const cxMtx& m);
	cxMtx to_mtx() const;
	float get_axis_ang(cxVec* pAxis) const;
	cxVec get_scaled_axis() const;
	void from_scaled_axis(const cxVec& sa);

	int get_zero_mask() const {
		return _mm_movemask_ps(_mm_cmpeq_ps(xload(), _mm_setzero_ps()));
	}

	float mag2() const {
		float res;
		XMStoreFloat(&res, XMQuaternionLengthSq(xload()));
		return res;
	}

	float mag() const {
		float res;
		XMStoreFloat(&res, XMQuaternionLength(xload()));
		return res;
	}

	float dot(const cxQuat& q) const { return nxVec::dot4(xload(), q.xload()); }
	double dot_d(const cxQuat& q) const { return nxVec::dot4_d(xload(), q.xload()); }

	void mul(const cxQuat& q) { xstore(XMQuaternionMultiply(q.xload(), xload())); }
	void mul(const cxQuat& q1, const cxQuat& q2) { xstore(XMQuaternionMultiply(q2.xload(), q1.xload())); }

	void normalize() { xstore(XMQuaternionNormalize(xload())); }
	void normalize(const cxQuat& q) { xstore(XMQuaternionNormalize(q.xload())); }
	cxQuat get_normalized() const { cxQuat q; q.normalize(*this); return q; }

	void conjugate() { xstore(XMQuaternionConjugate(xload())); }
	void conjugate(const cxQuat& q) { xstore(XMQuaternionConjugate(q.xload())); }
	cxQuat get_conjugated() const { cxQuat q; q.conjugate(*this); return q; }

	void invert() { xstore(XMQuaternionInverse(xload())); }
	void invert(const cxQuat& q) { xstore(XMQuaternionInverse(q.xload())); }
	cxQuat get_inverted() const { cxQuat q; q.invert(*this); return q; }

	void set_rot(const cxVec& axis, float ang);
	void set_rot_x(float rx);
	void set_rot_y(float ry);
	void set_rot_z(float rz);
	void set_rot_xyz(float rx, float ry, float rz);
	void set_rot_xzy(float rx, float ry, float rz);
	void set_rot_yxz(float rx, float ry, float rz);
	void set_rot_yzx(float rx, float ry, float rz);
	void set_rot_zxy(float rx, float ry, float rz);
	void set_rot_zyx(float rx, float ry, float rz);
	void set_rot(float rx, float ry, float rz, exRotOrd ord = exRotOrd::XYZ);
	void set_rot_degrees(const cxVec& r, exRotOrd ord = exRotOrd::XYZ);

	void lerp(const cxQuat& q1, const cxQuat& q2, float t) {
		xstore(XMVectorLerp(q1.xload(), q2.xload(), t));
		normalize();
	}

	void slerp(const cxQuat& q1, const cxQuat& q2, float t);

	cxVec apply(const cxVec& v) const { return XMVector3Rotate(v.get_xv(), xload()); }

	double diff_estimate(const cxQuat& a, const cxQuat& b) const { return 1.0 - nxCalc::min(1.0, ::fabs(a.dot_d(b))); }
};

inline cxQuat operator * (const cxQuat& q1, const cxQuat& q2) { cxQuat q = q1; q.mul(q2); return q; }

namespace nxQuat {

inline cxQuat slerp(const cxQuat& q1, const cxQuat& q2, float t) {
	cxQuat q;
	q.slerp(q1, q2, t);
	return q;
}

} // nxQuat


class cxColor {
public:
	union {
		struct { float r, g, b, a; };
		XMVECTOR mRGBA;
	};

public:
	cxColor() {}
	cxColor(float val) { set(val); }
	cxColor(float r, float g, float b, float a = 1.0f) { set(r, g, b, a); }

	void set(float r, float g, float b, float a = 1.0f) { mRGBA = XMVectorSet(r, g, b, a); }
	void set(float val) { set(val, val, val); }

	float get_r() const { return XMVectorGetByIndex(mRGBA, 0); }
	float get_g() const { return XMVectorGetByIndex(mRGBA, 1); }
	float get_b() const { return XMVectorGetByIndex(mRGBA, 2); }
	float get_a() const { return XMVectorGetByIndex(mRGBA, 3); }
	void set_r(float r) { mRGBA = XMVectorSetByIndex(mRGBA, r, 0); }
	void set_g(float g) { mRGBA = XMVectorSetByIndex(mRGBA, g, 1); }
	void set_b(float b) { mRGBA = XMVectorSetByIndex(mRGBA, b, 2); }
	void set_a(float a) { mRGBA = XMVectorSetByIndex(mRGBA, a, 3); }

	float luma() const;
	float luminance() const;

	void scl_rgb(float s) {
		r *= s;
		g *= s;
		b *= s;
	}

	void make_linear();
	void make_sRGB();

	XMVECTOR to_HSV() const;
	void from_HSV(XMVECTOR hsv);
};


class cxTrackball {
protected:
	cxQuat mSpin;
	cxQuat mQuat;
	float mRadius;

public:
	cxTrackball() : mRadius(0.8f) {
		reset();
	}

	void reset() {
		mSpin.identity();
		mQuat.identity();
	}

	void set_radius(float r) { mRadius = nxCalc::clamp(r, 0.001f, 1.0f); }
	cxVec apply(const cxVec& v) { return mQuat.apply(v); }
	void update(float x0, float y0, float x1, float y1);
};

namespace nxGeom {

inline cxVec tri_normal_cw(const cxVec& v0, const cxVec& v1, const cxVec& v2) {
	return nxVec::cross(v0 - v1, v2 - v1).get_normalized();
}

inline cxVec tri_normal_ccw(const cxVec& v0, const cxVec& v1, const cxVec& v2) {
	return nxVec::cross(v1 - v0, v2 - v0).get_normalized();
}

inline float signed_tri_area_2d(float ax, float ay, float bx, float by, float cx, float cy) {
	return (ax - cx)*(by - cy) - (ay - cy)*(bx - cx);
}

bool seg_seg_overlap_2d(float s0x0, float s0y0, float s0x1, float s0y1, float s1x0, float s1y0, float s1x1, float s1y1);
bool seg_quad_intersect_cw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, cxVec* pHitPos = nullptr, cxVec* pHitNrm = nullptr);
bool seg_quad_intersect_cw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, const cxVec& nrm, cxVec* pHitPos = nullptr);
bool seg_quad_intersect_ccw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, cxVec* pHitPos = nullptr, cxVec* pHitNrm = nullptr);
bool seg_quad_intersect_ccw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, const cxVec& nrm, cxVec* pHitPos = nullptr);
bool seg_polyhedron_intersect(const cxVec& p0, const cxVec& p1, const cxPlane* pPln, int plnNum, float* pFirst = nullptr, float* pLast = nullptr);
bool seg_cylinder_intersect(const cxVec& p0, const cxVec& p1, const cxVec& cp0, const cxVec& cp1, float cr, cxVec* pHitPos = nullptr, float* pHitDist = nullptr);
bool seg_tri_intersect_cw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& nrm, cxVec* pHitPos = nullptr);
bool seg_tri_intersect_cw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, cxVec* pHitPos = nullptr, cxVec* pHitNrm = nullptr);
bool seg_tri_intersect_ccw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& nrm, cxVec* pHitPos = nullptr);
bool seg_tri_intersect_ccw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, cxVec* pHitPos = nullptr, cxVec* pHitNrm = nullptr);
cxVec barycentric(const cxVec& pos, const cxVec& v0, const cxVec& v1, const cxVec& v2);
float quad_dist2(const cxVec& pos, const cxVec vtx[4]);
void update_nrm_newell(cxVec* pNrm, cxVec* pVtxI, cxVec* pVtxJ);
cxVec poly_normal_cw(cxVec* pVtx, int vtxNum);
cxVec poly_normal_ccw(cxVec* pVtx, int vtxNum);

int quad_convex_ck(const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3);
inline bool is_quad_convex(const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3) { return quad_convex_ck(v0, v1, v2, v3) == 3; }

inline bool aabb_overlap(const cxVec& min0, const cxVec& max0, const cxVec& min1, const cxVec& max1) {
	XMVECTOR vmin0 = min0.get_xv();
	XMVECTOR vmax0 = max0.get_xv();
	XMVECTOR vmin1 = min1.get_xv();
	XMVECTOR vmax1 = max1.get_xv();
	int m = _mm_movemask_ps(_mm_and_ps(_mm_cmple_ps(vmin0, vmax1), _mm_cmpnlt_ps(vmax0, vmin1)));
	return ((m & 7) == 7);
}

inline bool pnt_in_aabb(const cxVec& pos, const cxVec& min, const cxVec& max) {
	XMVECTOR vpos = pos.get_xv();
	XMVECTOR vmin = min.get_xv();
	XMVECTOR vmax = max.get_xv();
	return ((_mm_movemask_ps(_mm_and_ps(_mm_cmple_ps(vpos, vmax), _mm_cmpnlt_ps(vpos, vmin))) & 7) == 7);
}

} // nxGeom

class cxLineSeg {
protected:
	cxVec mPos0;
	cxVec mPos1;

public:
	cxLineSeg() {}
	cxLineSeg(const cxVec& p0, const cxVec& p1) { set(p0, p1); }

	cxVec get_pos0() const { return mPos0; }
	cxVec* get_pos0_ptr() { return &mPos0; }
	void set_pos0(const cxVec& p0) { mPos0 = p0;  }
	cxVec get_pos1() const { return mPos1; }
	cxVec* get_pos1_ptr() { return &mPos1; }
	cxVec get_inner_pos(float t) const { return nxVec::lerp(mPos0, mPos1, nxCalc::saturate(t)); }
	void set_pos1(const cxVec& p1) { mPos1 = p1; }
	void set(const cxVec& p0, const cxVec& p1) { mPos0 = p0; mPos1 = p1; }
	cxVec get_center() const { return (mPos0 + mPos1) * 0.5f; }
	cxVec get_dir() const { return mPos1 - mPos0; }
	cxVec get_dir_nrm() const { return get_dir().get_normalized(); }
	float len2() const { return nxVec::dist2(mPos0, mPos1); }
	float len() const { return nxVec::dist(mPos0, mPos1); }
};

class cxPlane {
protected:
	XMVECTOR mABCD;

public:
	cxPlane() {}

	void calc(const cxVec& pos, const cxVec& nrm) { mABCD = XMVectorSetW(nrm.get_xv(), pos.dot(nrm)); }
	cxVec get_normal() const { cxVec n; n.set_xv(mABCD); return n; }
	float get_D() const { return XMVectorGetW(mABCD); }
	float signed_dist(const cxVec& pos) const { return pos.dot(get_normal()) - get_D(); }
	float dist(const cxVec& pos) const { return ::fabsf(signed_dist(pos)); }
	bool pnt_in_front(const cxVec& pos) const { return signed_dist(pos) >= 0.0f; }
};

class cxSphere {
protected:
	XMVECTOR mSph;

public:
	cxSphere() {}
	cxSphere(const cxVec& c, float r) { mSph = XMVectorSetW(c.get_xv(), r); }
	cxSphere(float x, float y, float z, float r) { mSph = XMVectorSet(x, y, z, r); }

	float get_radius() const { return XMVectorGetW(mSph); }
	void set_radius(float r) { mSph = XMVectorSetW(mSph, r); }

	cxVec get_center() const { cxVec c; c.set_xv(mSph); return c; }
	void set_center(const cxVec& c) { float r = get_radius(); mSph = XMVectorSetW(c.get_xv(), r); }

	float volume() const { return (4.0f*XM_PI / 3.0f) * nxCalc::cb(get_radius()); }
	float surf_area() const { return 4.0f*XM_PI * nxCalc::sq(get_radius()); }

	bool contains(const cxVec& pos, float* pSqDist = nullptr) const {
		float dd = nxVec::dist2(pos, get_center());
		float rr = nxCalc::sq(get_radius());
		if (pSqDist) {
			*pSqDist = dd;
		}
		return dd <= rr;
	}

	bool overlap(const cxSphere& sph) const;
	bool overlap(const cxAABB& box) const;
};

class cxAABB {
protected:
	cxVec mMin;
	cxVec mMax;

public:
	cxAABB() {}
	cxAABB(const cxVec& p0, const cxVec& p1) { set(p0, p1); }

	void init() {
		mMin.fill(FLT_MAX);
		mMax.fill(-FLT_MAX);
	}

	void set(const cxVec& p0, const cxVec& p1) {
		mMin.min(p0, p1);
		mMax.max(p0, p1);
	}

	cxVec get_min_pos() const { return mMin; }
	cxVec get_max_pos() const { return mMax; }
	cxVec get_center() const { return ((mMin + mMax) * 0.5f); }
	cxVec get_size_vec() const { return mMax - mMin; }
	cxVec get_radius_vec() const { return get_size_vec() * 0.5f; }
	float get_bounding_radius() const { return get_radius_vec().mag(); }

	void add_pnt(const cxVec& p) {
		mMin.min(p);
		mMax.max(p);
	}

	void merge(const cxAABB& box) {
		mMin.min(box.mMin);
		mMax.max(box.mMax);
	}

	void transform(const cxAABB& box, const cxMtx& mtx);
	void transform(const cxMtx& mtx) { transform(*this, mtx); }

	void from_seg(const cxLineSeg& seg) {
		set(seg.get_pos0(), seg.get_pos1());
	}

	bool contains(const cxVec& pos) const { return nxGeom::pnt_in_aabb(pos, mMin, mMax); }

	bool seg_ck(const cxVec& p0, const cxVec& p1) const;
	bool seg_ck(const cxLineSeg& seg) const { return seg_ck(seg.get_pos0(), seg.get_pos1()); }

	bool overlap(const cxSphere& sph) const { return sph.overlap(*this); }
	bool overlap(const cxAABB& box) const { return nxGeom::aabb_overlap(mMin, mMax, box.mMin, box.mMax); }
};

class cxFrustum {
protected:
	cxVec mPnt[8];
	cxVec mNrm[6];
	cxPlane mPlane[6];

	void calc_normals();
	void calc_planes();

public:
	cxFrustum() {}

	void init(const cxMtx& mtx, float fovy, float aspect, float znear, float zfar);

	bool cull(const cxSphere& sph) const;
	bool overlap(const cxSphere& sph) const;
	bool cull(const cxAABB& box) const;
	bool overlap(const cxAABB& box) const;
};

namespace nxGeom {

inline cxAABB mk_empty_bbox() {
	cxAABB bbox;
	bbox.init();
	return bbox;
}

} // nxGeom


class cxBasicIK {
public:
	struct ChainInfo {
		cxVec mTopOffs;
		cxVec mRotOffs;
		cxVec mEndOffs;
		cxVec mExtOffs;
		exAxis mDirAxis;
		exAxis mUpAxis;
		union Flags {
			struct {
				uint32_t mCalcEnd : 1;
				uint32_t mRevFoot : 1;
			};
			uint32_t mAll;
		} mFlags;

		ChainInfo() { reset(); }

		void reset();

		void set_axis_leg() {
			mDirAxis = exAxis::MINUS_Y;
			mUpAxis = exAxis::PLUS_Z;
		}

		void set_axis_arm_l() {
			mDirAxis = exAxis::PLUS_X;
			mUpAxis = exAxis::MINUS_Z;
		}

		void set_axis_arm_r() {
			mDirAxis = exAxis::MINUS_X;
			mUpAxis = exAxis::MINUS_Z;
		}

		bool is_calc_end() const { return !!mFlags.mCalcEnd; }
		bool is_rev_foot() const { return !!mFlags.mRevFoot; }
	};

	struct ChainPose {
		cxMtx mRootW;
		cxMtx mParentW;
		cxQuat mTopRot;
		cxQuat mEndRot;
		cxQuat mExtRot;
		cxVec mEffPos;
		bool mIsEffPosW;

		ChainPose() : mIsEffPosW(false) {}

		cxVec get_eff_pos_w() const { return mIsEffPosW ? mEffPos : mRootW.calc_pnt(mEffPos); }
	};

	struct ChainResult {
		cxMtx mTop;
		cxMtx mRot;
		cxMtx mEnd;
		cxMtx mExt;
	};

protected:
	ChainInfo mInfo;
	float mLen0;
	float mLen1;

public:
	cxBasicIK() {
		mInfo.reset();
	}

	bool is_rev_foot() const { return mInfo.is_rev_foot(); }
	void init(const ChainInfo& info);
	void solve(ChainResult* pRes, const ChainPose& pose);
};


namespace nxSH {

inline int calc_coefs_num(int order) { return order < 1 ? 0 : nxCalc::sq(order); }
inline int calc_ary_idx(int l, int m) { return l*(l+1) + m; }
inline int band_idx_from_ary_idx(int idx) { return (int)::sqrtf((float)idx); }
inline int func_idx_from_ary_band(int idx, int l) { return idx - l*(l + 1); }
double calc_K(int l, int m);

#define XD_SHEVAL_F64 0

#define _XD_SHEVAL_FUNC(_ord_) void eval##_ord_(float* pCoef, float x, float y, float z);
_XD_SHEVAL_FUNC(2)
_XD_SHEVAL_FUNC(3)
_XD_SHEVAL_FUNC(4)
_XD_SHEVAL_FUNC(5)
_XD_SHEVAL_FUNC(6)
_XD_SHEVAL_FUNC(7)
_XD_SHEVAL_FUNC(8)
_XD_SHEVAL_FUNC(9)
_XD_SHEVAL_FUNC(10)

void eval(int order, float* pCoef, float x, float y, float z);

#if XD_SHEVAL_F64
#undef _XD_SHEVAL_FUNC
#define _XD_SHEVAL_FUNC(_ord_) void eval##_ord_(double* pCoef, double x, double y, double z);
_XD_SHEVAL_FUNC(2)
_XD_SHEVAL_FUNC(3)
_XD_SHEVAL_FUNC(4)
_XD_SHEVAL_FUNC(5)
_XD_SHEVAL_FUNC(6)
_XD_SHEVAL_FUNC(7)
_XD_SHEVAL_FUNC(8)
_XD_SHEVAL_FUNC(9)
_XD_SHEVAL_FUNC(10)

void eval(int order, double* pCoef, double x, double y, double z);
#endif

void project_polar_map(int order, float* pCoefR, float* pCoefG, float* pCoefB, cxColor* pMap, int w, int h, float* pTmp = nullptr);
void calc_weights(float* pWgt, int order, float s, float scl = 1.0f);
void apply_weights(float* pDst, int order, const float* pSrc, const float* pWgt);
float dot(int order, float* pA, float* pB);

} // nxSH

struct sxSHIdx {
	uint16_t l, m;

	sxSHIdx() : l(0), m(0) {}
	sxSHIdx(int band, int func) : l(band), m(func) {}

	int get_band_idx() const { return l; }
	int get_func_idx() const { return m; }
	int get_ary_idx() const { return nxSH::calc_ary_idx(l, m); }
	void from_ary_idx(int idx) {
		l = nxSH::band_idx_from_ary_idx(idx);
		m = nxSH::func_idx_from_ary_band(idx, l);
	}

	double calc_K() const { return nxSH::calc_K(l, m); }
};




