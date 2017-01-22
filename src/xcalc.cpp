/*
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#include <string.h>
#include <stdio.h>

#include "xcalc.hpp"

namespace nxCalc {

float hermite(float p0, float m0, float p1, float m1, float t) {
	float tt = t*t;
	float ttt = tt*t;
	float tt3 = tt*3.0f;
	float tt2 = tt + tt;
	float ttt2 = tt2 * t;
	float h00 = ttt2 - tt3 + 1.0f;
	float h10 = ttt - tt2 + t;
	float h01 = -ttt2 + tt3;
	float h11 = ttt - tt;
	return (h00*p0 + h10*m0 + h01*p1 + h11*m1);
}

float bezier01(float t, float p1, float p2) {
	static XMMATRIX bm = {
		{ -1.0f, 3.0f, -3.0f, 1.0f },
		{ 3.0f, -6.0f, 3.0f, 0.0f },
		{ -3.0f, 3.0f, 0.0f, 0.0f },
		{ 1.0f, 0.0f, 0.0f, 0.0f }
	};
	float tt = t*t;
	float ttt = tt*t;
	XMVECTOR tv = XMVectorSet(ttt, tt, t, 1.0f);
	tv = XMVector4Transform(tv, bm);
	float val = p1*XMVectorGetY(tv) + p2*XMVectorGetZ(tv) + XMVectorGetW(tv);
	return val;
}

float fit(float val, float oldMin, float oldMax, float newMin, float newMax) {
	float rel = (val - oldMin) / (oldMax - oldMin);
	return lerp(newMin, newMax, rel);
}

float calc_fovy(float focal, float aperture, float aspect) {
	float zoom = ((2.0f * focal) / aperture) * aspect;
	float fovy = 2.0f * ::atan2f(1.0f, zoom);
	return fovy;
}

} // nxCalc


namespace nxVec {

float dist2(const cxVec& pos0, const cxVec& pos1) { return (pos1 - pos0).mag2(); }
float dist(const cxVec& pos0, const cxVec& pos1) { return (pos1 - pos0).mag(); }

cxVec get_axis(exAxis axis) {
	int i = (int)axis;
	cxVec v(0.0f);
	float* pV = (float*)&v;
	pV[(i >> 1) & 3] = (i & 1) ? -1.0f : 1.0f;
	return v;
}

cxVec basis_xform(const cxVec& ax, const cxVec& ay, const cxVec& az, const cxVec& vec) {
	XMVECTOR frame[3];
	frame[0] = ax.get_xv();
	frame[1] = ay.get_xv();
	frame[2] = ax.get_xv();
	nxVec::transpose3(frame, frame);
	XMVECTOR xv = vec.get_xv();
	return XMVectorPermute<0, 1, 4, 4>(XMVectorMergeXY(XMVector3Dot(xv, frame[0]), XMVector3Dot(xv, frame[1])), XMVector3Dot(xv, frame[2]));
}

cxVec from_polar_uv(float u, float v) {
#if 0
	float azimuth = u * 2.0f * XD_PI;
	float sinAzi = ::sinf(azimuth);
	float cosAzi = ::cosf(azimuth);
	float elevation = (v - 1.0f) * XD_PI;
	float sinEle = ::sinf(elevation);
	float cosEle = ::cosf(elevation);
	float nx = cosAzi*sinEle;
	float ny = cosEle;
	float nz = sinAzi*sinEle;
	return cxVec(nx, ny, nz);
#else
	XMVECTOR vsin;
	XMVECTOR vcos;
	float azimuth = u * 2.0f * XD_PI;
	float elevation = (v - 1.0f) * XD_PI;
	XMVectorSinCos(&vsin, &vcos, XMVectorSet(azimuth, elevation, azimuth, elevation));
	float sinAzi = XMVectorGetX(vsin);
	float cosAzi = XMVectorGetX(vcos);
	float sinEle = XMVectorGetY(vsin);
	float cosEle = XMVectorGetY(vcos);
	float nx = cosAzi*sinEle;
	float ny = cosEle;
	float nz = sinAzi*sinEle;
	return cxVec(nx, ny, nz);
#endif
}

cxVec reflect(const cxVec& vec, const cxVec& nrm) {
	return vec - nrm*vec.dot(nrm)*2.0f;
}

} // nxVec

void cxVec::parse(const char* pStr) {
	float val[3];
	::memset(val, 0, sizeof(val));
	::sscanf_s(pStr, "[%f,%f,%f]", &val[0], &val[1], &val[2]);
	set(val[0], val[1], val[2]);
}

// http://www.insomniacgames.com/mike-day-vector-length-and-normalization-difficulties/
float cxVec::mag() const {
	cxVec v = *this;
	float m = v.max_abs_elem();
	v /= m;
	float l = v.mag_fast() * m;
	_mm_store_ss(&l, _mm_and_ps(_mm_set_ss(l), _mm_cmpneq_ss(_mm_set_ss(m), _mm_setzero_ps())));
	return l;
}

// see ref above
void cxVec::normalize(const cxVec& v) {
	cxVec n = v;
	float m = v.max_abs_elem();
	if (m > 0.0f) {
		n.scl(1.0f / m);
		n.scl(1.0f / n.mag_fast());
	}
	*this = n;
}

// http://lgdv.cs.fau.de/publications/publication/Pub.2010.tech.IMMD.IMMD9.onfloa/
// http://jcgt.org/published/0003/02/01/
XD_NOINLINE XMFLOAT2 cxVec::encode_octa() const {
	XMFLOAT2 oct;
	cxVec av = abs_val();
	float d = nxCalc::rcp0(av.x + av.y + av.z);
	float ox = x * d;
	float oy = y * d;
	if (z < 0.0f) {
		float tx = (1.0f - ::fabsf(oy)) * (ox < 0.0f ? -1.0f : 1.0f);
		float ty = (1.0f - ::fabsf(ox)) * (oy < 0.0f ? -1.0f : 1.0f);
		ox = tx;
		oy = ty;
	}
	oct.x = ox;
	oct.y = oy;
	return oct;
}

void cxVec::decode_octa(const XMFLOAT2& oct) {
	float ox = oct.x;
	float oy = oct.y;
	float ax = ::fabsf(ox);
	float ay = ::fabsf(oy);
	z = 1.0f - ax - ay;
	if (z < 0.0f) {
		x = (1.0f - ay) * (ox < 0.0f ? -1.0f : 1.0f);
		y = (1.0f - ax) * (oy < 0.0f ? -1.0f : 1.0f);
	} else {
		x = ox;
		y = oy;
	}
	normalize();
}


void cxMtx::from_quat(const cxQuat& q) {
	*this = q.to_mtx();
}

void cxMtx::from_quat_and_pos(const cxQuat& qrot, const cxVec& vtrans) {
	from_quat(qrot);
	set_translation(vtrans);
}

cxQuat cxMtx::to_quat() const {
	cxQuat q;
	q.from_mtx(*this);
	return q;
}

XD_NOINLINE void cxMtx::invert() {
	XMVECTOR det;
	xset(XMMatrixInverse(&det, xget()));
}

XD_NOINLINE void cxMtx::invert(const cxMtx& m) {
	XMVECTOR det;
	xset(XMMatrixInverse(&det, m.xget()));
}

#if XD_EXT_CPU
static inline void mtx_mul_AVX(XMMATRIX* pDst, const XMMATRIX* pSrcA, const XMMATRIX* pSrcB) {
	__m256* pRA = (__m256*)pSrcA;
	__m128* pRB = (__m128*)pSrcB;
	__m256* pD = (__m256*)pDst;
	__m256 rA0A1 = pRA[0];
	__m256 rA2A3 = pRA[1];
	__m256 rB0 = _mm256_broadcast_ps(&pRB[0]);
	__m256 rB1 = _mm256_broadcast_ps(&pRB[1]);
	__m256 rB2 = _mm256_broadcast_ps(&pRB[2]);
	__m256 rB3 = _mm256_broadcast_ps(&pRB[3]);
	pD[0] = _mm256_add_ps(
		_mm256_add_ps(
			_mm256_mul_ps(_mm256_shuffle_ps(rA0A1, rA0A1, XD_ELEM_MASK(0)), rB0),
			_mm256_mul_ps(_mm256_shuffle_ps(rA0A1, rA0A1, XD_ELEM_MASK(1)), rB1)
		),
		_mm256_add_ps(
			_mm256_mul_ps(_mm256_shuffle_ps(rA0A1, rA0A1, XD_ELEM_MASK(2)), rB2),
			_mm256_mul_ps(_mm256_shuffle_ps(rA0A1, rA0A1, XD_ELEM_MASK(3)), rB3)
		)
	);
	pD[1] = _mm256_add_ps(
		_mm256_add_ps(
			_mm256_mul_ps(_mm256_shuffle_ps(rA2A3, rA2A3, XD_ELEM_MASK(0)), rB0),
			_mm256_mul_ps(_mm256_shuffle_ps(rA2A3, rA2A3, XD_ELEM_MASK(1)), rB1)
		),
		_mm256_add_ps(
			_mm256_mul_ps(_mm256_shuffle_ps(rA2A3, rA2A3, XD_ELEM_MASK(2)), rB2),
			_mm256_mul_ps(_mm256_shuffle_ps(rA2A3, rA2A3, XD_ELEM_MASK(3)), rB3)
		)
	);
}

void cxMtx::mul(const cxMtx& m) {
	mtx_mul_AVX(this, this, &m);
}

void cxMtx::mul(const cxMtx& m1, const cxMtx& m2) {
	mtx_mul_AVX(this, &m1, &m2);
}
#else

void cxMtx::mul(const cxMtx& m) {
	xset(XMMatrixMultiply(xget(), m.xget()));
}

void cxMtx::mul(const cxMtx& m1, const cxMtx& m2) {
	xset(XMMatrixMultiply(m1.xget(), m2.xget()));
}
#endif

static inline void mtx_add(XMMATRIX* pDst, const XMMATRIX* pSrcA, const XMMATRIX* pSrcB) {
#if XD_EXT_CPU
	__m256* pRA = (__m256*)pSrcA;
	__m256* pRB = (__m256*)pSrcB;
	__m256* pD = (__m256*)pDst;
	pD[0] = _mm256_add_ps(pRA[0], pRB[0]);
	pD[1] = _mm256_add_ps(pRA[1], pRB[1]);
#else
	pDst->r[0] = pSrcA->r[0] + pSrcB->r[0];
	pDst->r[1] = pSrcA->r[1] + pSrcB->r[1];
	pDst->r[2] = pSrcA->r[2] + pSrcB->r[2];
	pDst->r[3] = pSrcA->r[3] + pSrcB->r[3];
#endif
}

void cxMtx::add(const cxMtx& m) {
	mtx_add(this, this, &m);
}

void cxMtx::add(const cxMtx& m1, const cxMtx& m2) {
	mtx_add(this, &m1, &m2);
}

static inline void mtx_sub(XMMATRIX* pDst, const XMMATRIX* pSrcA, const XMMATRIX* pSrcB) {
#if XD_EXT_CPU
	__m256* pRA = (__m256*)pSrcA;
	__m256* pRB = (__m256*)pSrcB;
	__m256* pD = (__m256*)pDst;
	pD[0] = _mm256_sub_ps(pRA[0], pRB[0]);
	pD[1] = _mm256_sub_ps(pRA[1], pRB[1]);
#else
	pDst->r[0] = pSrcA->r[0] - pSrcB->r[0];
	pDst->r[1] = pSrcA->r[1] - pSrcB->r[1];
	pDst->r[2] = pSrcA->r[2] - pSrcB->r[2];
	pDst->r[3] = pSrcA->r[3] - pSrcB->r[3];
#endif
}

void cxMtx::sub(const cxMtx& m) {
	mtx_sub(this, this, &m);
}

void cxMtx::sub(const cxMtx& m1, const cxMtx& m2) {
	mtx_sub(this, &m1, &m2);
}

void cxMtx::set_rot(const cxVec& axis, float ang) {
	xset(XMMatrixRotationAxis(axis.get_xv(), ang));
}

void cxMtx::set_rot_x(float rx) {
	xset(XMMatrixRotationX(rx));
}

void cxMtx::set_rot_y(float ry) {
	xset(XMMatrixRotationY(ry));
}

void cxMtx::set_rot_z(float rz) {
	xset(XMMatrixRotationZ(rz));
}

void cxMtx::set_rot_xyz(float rx, float ry, float rz) {
	cxMtx m;
	set_rot_x(rx);
	m.set_rot_y(ry);
	mul(m);
	m.set_rot_z(rz);
	mul(m);
}

void cxMtx::set_rot_xzy(float rx, float ry, float rz) {
	cxMtx m;
	set_rot_x(rx);
	m.set_rot_z(rz);
	mul(m);
	m.set_rot_y(ry);
	mul(m);
}

void cxMtx::set_rot_yxz(float rx, float ry, float rz) {
	cxMtx m;
	set_rot_y(ry);
	m.set_rot_x(rx);
	mul(m);
	m.set_rot_z(rz);
	mul(m);
}

void cxMtx::set_rot_yzx(float rx, float ry, float rz) {
	cxMtx m;
	set_rot_y(ry);
	m.set_rot_z(rz);
	mul(m);
	m.set_rot_x(rx);
	mul(m);
}

void cxMtx::set_rot_zxy(float rx, float ry, float rz) {
	cxMtx m;
	set_rot_z(rz);
	m.set_rot_x(rx);
	mul(m);
	m.set_rot_y(ry);
	mul(m);
}

void cxMtx::set_rot_zyx(float rx, float ry, float rz) {
	cxMtx m;
	set_rot_z(rz);
	m.set_rot_y(ry);
	mul(m);
	m.set_rot_x(rx);
	mul(m);
}

void cxMtx::set_rot(float rx, float ry, float rz, exRotOrd ord) {
	switch (ord) {
		case exRotOrd::XYZ:
			set_rot_xyz(rx, ry, rz);
			break;
		case exRotOrd::XZY:
			set_rot_xzy(rx, ry, rz);
			break;
		case exRotOrd::YXZ:
			set_rot_yxz(rx, ry, rz);
			break;
		case exRotOrd::YZX:
			set_rot_yzx(rx, ry, rz);
			break;
		case exRotOrd::ZXY:
			set_rot_zxy(rx, ry, rz);
			break;
		case exRotOrd::ZYX:
			set_rot_zyx(rx, ry, rz);
			break;
		default:
			identity();
			break;
	}
}

void cxMtx::set_rot_degrees(const cxVec& r, exRotOrd ord) {
	cxVec rr = r * XD_DEG2RAD(1.0f);
	set_rot(rr.x, rr.y, rr.z, ord);
}

bool cxMtx::is_valid_rot(float tol) const {
	cxVec r0 = get_row(0);
	cxVec r1 = get_row(1);
	cxVec r2 = get_row(2);
	float v = ::fabsf(nxVec::scalar_triple(r0, r1, r2));
	if (::fabsf(v - 1.0f) > tol) return false;

	v = r0.dot(r0);
	if (::fabsf(v - 1.0f) > tol) return false;
	v = r1.dot(r1);
	if (::fabsf(v - 1.0f) > tol) return false;
	v = r2.dot(r2);
	if (::fabsf(v - 1.0f) > tol) return false;

	v = r0.dot(r1);
	if (::fabsf(v) > tol) return false;
	v = r1.dot(r2);
	if (::fabsf(v) > tol) return false;
	v = r0.dot(r2);
	if (::fabsf(v) > tol) return false;

	return true;
}

static inline float limit_pi(float rad) {
	//return nxCalc::mod_pi(rad);
	rad = ::fmodf(rad, XD_PI*2);
	if (::fabsf(rad) > XD_PI) {
		if (rad < 0.0f) {
			rad = XD_PI*2 + rad;
		} else {
			rad = rad - XD_PI*2;
		}
	}
	return rad;
}

cxVec cxMtx::get_rot(exRotOrd ord) const {
	cxVec rv(0.0f);
	cxQuat q = to_quat();
	float qw = nxCalc::clamp(q.w, -1.0f, 1.0f);
	int axisMask = q.get_zero_mask();
	bool singleAxis = false;
	switch (axisMask) {
		case 6: /* 0110 -> X */
			rv.x = ::acosf(qw) * 2.0f;
			singleAxis = true;
			break;
		case 5: /* 0101 -> Y */
			rv.y = ::acosf(qw) * 2.0f;
			singleAxis = true;
			break;
		case 3: /* 0011 -> Z */
			rv.z = ::acosf(qw) * 2.0f;
			singleAxis = true;
			break;
		case 7: /* 0111 -> identity */
			singleAxis = true;
			break;
	}
	if (singleAxis) {
		return rv;
	}
	static struct { uint8_t i0, i1, i2, s; } tbl[] = {
		/* XYZ */ { 0, 1, 2, 1 },
		/* XZY */ { 0, 2, 1, 0 },
		/* YXZ */ { 1, 0, 2, 0 },
		/* YZX */ { 1, 2, 0, 1 },
		/* ZXY */ { 2, 0, 1, 1 },
		/* ZYX */ { 2, 1, 0, 0 }
	};
	if ((uint32_t)ord >= sizeof(tbl) / sizeof(tbl[0])) {
		ord = exRotOrd::XYZ;
	}
	int i0 = tbl[(int)ord].i0;
	int i1 = tbl[(int)ord].i1;
	int i2 = tbl[(int)ord].i2;
	float sgn = tbl[(int)ord].s ? 1.0f : -1.0f;
	cxVec rm[3];
	rm[0].set(get_at(i0, i0), get_at(i0, i1), get_at(i0, i2));
	rm[1].set(get_at(i1, i0), get_at(i1, i1), get_at(i1, i2));
	rm[2].set(get_at(i2, i0), get_at(i2, i1), get_at(i2, i2));
	float r[3] = { 0, 0, 0 };
	r[i0] = ::atan2f(rm[1][2], rm[2][2]);
	r[i1] = ::atan2f(-rm[0][2], ::sqrtf(nxCalc::sq(rm[0][0]) + nxCalc::sq(rm[0][1])));
#if 0
	r[i2] = ::atan2f(rm[0][1], rm[0][0]);
#else
	// http://www.insomniacgames.com/mike-day-extracting-euler-angles-from-a-rotation-matrix/
	float s = ::sinf(r[i0]);
	float c = ::cosf(r[i0]);
	r[i2] = ::atan2f(s*rm[2][0] - c*rm[1][0], c*rm[1][1] - s*rm[2][1]);
#endif
	for (int i = 0; i < 3; ++i) {
		r[i] *= sgn;
	}
	for (int i = 0; i < 3; ++i) {
		r[i] = limit_pi(r[i]);
	}
	rv.from_mem(r);
	return rv;
}

void cxMtx::calc_xform(const cxMtx& mtxT, const cxMtx& mtxR, const cxMtx& mtxS, exTransformOrd ord) {
	const uint8_t S = 0;
	const uint8_t R = 1;
	const uint8_t T = 2;
	static struct { uint8_t m0, m1, m2; } tbl[] = {
		/* SRT */ { S, R, T },
		/* STR */ { S, T, R },
		/* RST */ { R, S, T },
		/* RTS */ { R, T, S },
		/* TSR */ { T, S, R },
		/* TRS */ { T, R, S }
	};
	const cxMtx* lst[3];
	lst[S] = &mtxS;
	lst[R] = &mtxR;
	lst[T] = &mtxT;

	if ((uint32_t)ord >= sizeof(tbl) / sizeof(tbl[0])) {
		ord = exTransformOrd::SRT;
	}
	int m0 = tbl[(int)ord].m0;
	int m1 = tbl[(int)ord].m1;
	int m2 = tbl[(int)ord].m2;
	xset(lst[m0]->xget());
	mul(*lst[m1]);
	mul(*lst[m2]);
}

void cxMtx::mk_view(const cxVec& pos, const cxVec& tgt, const cxVec& upvec) {
	xset(XMMatrixLookAtRH(pos.get_xv(), tgt.get_xv(), upvec.get_xv()));
}

void cxMtx::mk_proj(float fovy, float aspect, float znear, float zfar) {
	xset(XMMatrixPerspectiveFovRH(fovy, aspect, znear, zfar));
}

void cxMtx::sprintf_py(char* pBuf, size_t bufSize) const {
	if (!pBuf) return;
	::sprintf_s(pBuf, bufSize, "[[%.8f, %.8f, %.8f, %.8f], [%.8f, %.8f, %.8f, %.8f], [%.8f, %.8f, %.8f, %.8f], [%.8f, %.8f, %.8f, %.8f]]",
		get_at(0, 0), get_at(0, 1), get_at(0, 2), get_at(0, 3),
		get_at(1, 0), get_at(1, 1), get_at(1, 2), get_at(1, 3),
		get_at(2, 0), get_at(2, 1), get_at(2, 2), get_at(2, 3),
		get_at(3, 0), get_at(3, 1), get_at(3, 2), get_at(3, 3)
	);
}


void cxQuat::from_mtx(const cxMtx& m) {
	xstore(XMQuaternionRotationMatrix(m));
}

cxMtx cxQuat::to_mtx() const {
	return XMMatrixRotationQuaternion(xload());
}

float cxQuat::get_axis_ang(cxVec* pAxis) const {
	XMVECTOR axis;
	float ang;
	XMQuaternionToAxisAngle(&axis, &ang, xload());
	if (pAxis) {
		pAxis->set_xv(axis);
	}
	return ang;
}

#if 1
cxVec cxQuat::get_log_vec() const {
	float cosh = w;
	float hang = ::acosf(cosh);
	cxVec axis(x, y, z);
	axis.normalize();
	axis.scl(hang);
	return axis;
}

void cxQuat::from_log_vec(const cxVec& lvec, bool nrmFlg) {
	float hang = lvec.mag();
	float cosh = ::cosf(hang);
	cxVec axis = lvec * nxCalc::sinc(hang);
	x = axis.x;
	y = axis.y;
	z = axis.z;
	w = cosh;
	if (nrmFlg) {
		normalize();
	}
}
#else
cxVec cxQuat::get_log_vec() const {
	return XMQuaternionLn(xload());
}

void cxQuat::from_log_vec(const cxVec& lvec, bool nrmFlg) {
	xstore(XMQuaternionExp(lvec.get_xv()));
	if (nrmFlg) {
		normalize();
	}
}
#endif

void cxQuat::from_vecs(const cxVec& vfrom, const cxVec& vto) {
	cxVec axis = nxVec::cross(vfrom, vto);
	float sqm = axis.mag2();
	if (sqm == 0.0f) {
		identity();
	} else {
		float d = vfrom.dot(vto);
		if (d == 1.0f) {
			identity();
		} else {
			float c = ::sqrtf((1.0f + d) * 0.5f);
			float s = ::sqrtf((1.0f - d) * 0.5f);
			axis.scl(nxCalc::rcp0(::sqrtf(sqm)) * s);
			xstore(XMVectorSetW(axis.get_xv(), c));
			normalize();
		}
	}
}

cxMtx cxQuat::get_mul_mtx_r() const {
	cxMtx m;
	float tm[] = {
		 w, -z,  y, -x,
		 z,  w, -x, -y,
		-y,  x,  w, -z,
		 x,  y,  z,  w
	};
	m.from_mem(tm);
	return m;
}

cxMtx cxQuat::get_mul_mtx_l() const {
	cxMtx m;
	float tm[] = {
		 w,  z, -y, -x,
		-z,  w,  x, -y,
		 y, -x,  w, -z,
		 x,  y,  z,  w
	};
	m.from_mem(tm);
	return m;
}

void cxQuat::from_mul_mtx(const cxMtx& mtx) {
	xstore(mtx.get_row(3));
}

void cxQuat::set_rot(const cxVec& axis, float ang) {
	xstore(XMQuaternionRotationNormal(axis.get_normalized().get_xv(), ang));
}

void cxQuat::set_rot_x(float rx) {
	xstore(XMQuaternionRotationNormal(g_XMIdentityR0, rx));
}

void cxQuat::set_rot_y(float ry) {
	xstore(XMQuaternionRotationNormal(g_XMIdentityR1, ry));
}

void cxQuat::set_rot_z(float rz) {
	xstore(XMQuaternionRotationNormal(g_XMIdentityR2, rz));
}

void cxQuat::set_rot_xyz(float rx, float ry, float rz) {
	cxQuat qx, qy, qz;
	qx.set_rot_x(rx);
	qy.set_rot_y(ry);
	qz.set_rot_z(rz);
	mul(qz, qy);
	mul(qx);
}

void cxQuat::set_rot_xzy(float rx, float ry, float rz) {
	cxQuat qx, qy, qz;
	qx.set_rot_x(rx);
	qy.set_rot_y(ry);
	qz.set_rot_z(rz);
	mul(qy, qz);
	mul(qx);
}

void cxQuat::set_rot_yxz(float rx, float ry, float rz) {
	cxQuat qx, qy, qz;
	qx.set_rot_x(rx);
	qy.set_rot_y(ry);
	qz.set_rot_z(rz);
	mul(qz, qx);
	mul(qy);
}

void cxQuat::set_rot_yzx(float rx, float ry, float rz) {
	cxQuat qx, qy, qz;
	qx.set_rot_x(rx);
	qy.set_rot_y(ry);
	qz.set_rot_z(rz);
	mul(qx, qz);
	mul(qy);
}

void cxQuat::set_rot_zxy(float rx, float ry, float rz) {
	cxQuat qx, qy, qz;
	qx.set_rot_x(rx);
	qy.set_rot_y(ry);
	qz.set_rot_z(rz);
	mul(qy, qx);
	mul(qz);
}

void cxQuat::set_rot_zyx(float rx, float ry, float rz) {
	cxQuat qx, qy, qz;
	qx.set_rot_x(rx);
	qy.set_rot_y(ry);
	qz.set_rot_z(rz);
	mul(qz, qy);
	mul(qx);
}

void cxQuat::set_rot(float rx, float ry, float rz, exRotOrd ord) {
	switch (ord) {
		case exRotOrd::XYZ:
			set_rot_xyz(rx, ry, rz);
			break;
		case exRotOrd::XZY:
			set_rot_xzy(rx, ry, rz);
			break;
		case exRotOrd::YXZ:
			set_rot_yxz(rx, ry, rz);
			break;
		case exRotOrd::YZX:
			set_rot_yzx(rx, ry, rz);
			break;
		case exRotOrd::ZXY:
			set_rot_zxy(rx, ry, rz);
			break;
		case exRotOrd::ZYX:
			set_rot_zyx(rx, ry, rz);
			break;
		default:
			identity();
			break;
	}
}

void cxQuat::set_rot_degrees(const cxVec& r, exRotOrd ord) {
	cxVec rr = r * XD_DEG2RAD(1.0f);
	set_rot(rr.x, rr.y, rr.z, ord);
}

// http://www.geometrictools.com/Documentation/ConstrainedQuaternions.pdf

static inline cxQuat closest_quat_by_axis(const cxQuat& qsrc, int axis) {
	cxQuat qres;
	qres.identity();
	float e = qsrc.get_at(axis);
	float w = qsrc.get_w();
	float sqmag = nxCalc::sq(e) + nxCalc::sq(w);
	if (sqmag > 0.0f) {
		float imag = nxCalc::rcp0(::sqrtf(sqmag));
		qres.set_at(axis, e*imag);
		qres.set_w(w*imag);
	}
	return qres;
}

cxQuat cxQuat::get_closest_x() const { return closest_quat_by_axis(*this, 0); }

cxQuat cxQuat::get_closest_y() const { return closest_quat_by_axis(*this, 1); }

cxQuat cxQuat::get_closest_z() const { return closest_quat_by_axis(*this, 2); }

cxQuat cxQuat::get_closest_xy() const {
	cxQuat q;
	float x = get_x();
	float y = get_y();
	float z = get_z();
	float w = get_w();
	float det = ::fabsf(-x*y - z*w);
	float imag;
	if (det < 0.5f) {
		float d = ::sqrtf(::fabsf(1.0f - 4.0f*nxCalc::sq(det)));
		float a = x*w - y*z;
		float b = nxCalc::sq(w) - nxCalc::sq(x) + nxCalc::sq(y) - nxCalc::sq(z);
		float s0, c0;
		if (b >= 0.0f) {
			s0 = a;
			c0 = (d + b)*0.5f;
		} else {
			s0 = (d - b)*0.5f;
			c0 = a;
		}
		imag = nxCalc::rcp0(nxCalc::hypot(s0, c0));
		s0 *= imag;
		c0 *= imag;

		float s1 = y*c0 - z*s0;
		float c1 = w*c0 + x*s0;
		imag = nxCalc::rcp0(nxCalc::hypot(s1, c1));
		s1 *= imag;
		c1 *= imag;

		q.set(s0*c1, c0*s1, -s0*s1, c0*c1);
	} else {
		imag = nxCalc::rcp0(::sqrtf(det));
		q.set(x*imag, 0.0f, 0.0f, w*imag);
	}
	return q;
}

cxQuat cxQuat::get_closest_yx() const {
	cxQuat q = cxQuat(get_x(), get_y(), -get_z(), get_w()).get_closest_xy();
	q.set_z(-q.get_z());
	return q;
}

cxQuat cxQuat::get_closest_xz() const {
	cxQuat q = cxQuat(get_x(), get_z(), get_y(), get_w()).get_closest_yx();
	q.xstore(XD_SHUF(q.xload(), 0, 2, 1, 3));
	return q;
}

cxQuat cxQuat::get_closest_zx() const {
	cxQuat q = cxQuat(get_x(), get_z(), get_y(), get_w()).get_closest_xy();
	q.xstore(XD_SHUF(q.xload(), 0, 2, 1, 3));
	return q;
}

cxQuat cxQuat::get_closest_yz() const {
	cxQuat q = cxQuat(get_y(), get_z(), get_x(), get_w()).get_closest_xy();
	q.xstore(XD_SHUF(q.xload(), 2, 0, 1, 3));
	return q;
}

cxQuat cxQuat::get_closest_zy() const {
	cxQuat q = cxQuat(get_y(), get_z(), get_x(), get_w()).get_closest_yx();
	q.xstore(XD_SHUF(q.xload(), 2, 0, 1, 3));
	return q;
}

void cxQuat::slerp(const cxQuat& q1, const cxQuat& q2, float t) {
	xstore(XMQuaternionSlerp(q1.xload(), q2.xload(), t));
}


namespace nxColor {

XMGLOBALCONST XMVECTORF32 g_luma601 = { 0.299f, 0.587f, 0.114f, 0.0f };
XMGLOBALCONST XMVECTORF32 g_luma709 = { 0.2126f, 0.7152f, 0.0722f, 0.0f };
XMGLOBALCONST XMVECTORF32 g_Y709 = { 0.212671f, 0.71516f, 0.072169f, 0.0f };

float luma(XMVECTOR vrgb) { return nxVec::dot4(vrgb, nxColor::g_luma601); }
float luma_hd(XMVECTOR vrgb) { return nxVec::dot4(vrgb, nxColor::g_luma709); }
float luminance(XMVECTOR vrgb) { return nxVec::dot4(vrgb, nxColor::g_Y709); }

XMVECTOR RGB_to_YCgCo(XMVECTOR vrgb) {
#if 0
	cxVec rgb(vrgb);
	return XMVectorSet(
		rgb.dot(cxVec(0.25f, 0.5f, 0.25f)),
		rgb.dot(cxVec(-0.25f, 0.5f, -0.25f)),
		rgb.dot(cxVec(0.5f, 0.0f, -0.5f)),
		0.0f
	);
#else
	XMVECTOR hc = vrgb * 0.5f;
	float hr = XMVectorGetX(hc);
	float hg = XMVectorGetY(hc);
	float hb = XMVectorGetZ(hc);
	float qr = hr * 0.5f;
	float qb = hb * 0.5f;
	return XMVectorSet(qr + hg + qb, -qr + hg - qb, hr - hb, 0.0f);
#endif
}

XMVECTOR YCgCo_to_RGB(XMVECTOR vygo) {
#if 0
	cxVec ygo(vygo);
	return XMVectorSet(
		ygo.dot(cxVec(1.0f, -1.0f, 1.0f)),
		ygo.dot(cxVec(1.0f, 1.0f, 0.0f)),
		ygo.dot(cxVec(1.0f, -1.0f, -1.0f)),
		1.0f
	);
#else
	float y = XMVectorGetX(vygo);
	float g = XMVectorGetY(vygo);
	float o = XMVectorGetZ(vygo);
	float t = y - g;
	return XMVectorSet(t + o, y + g, t - o, 1.0f);
#endif
}

XMVECTOR RGB_to_TMI(XMVECTOR vrgb) {
	float r = XMVectorGetX(vrgb);
	float g = XMVectorGetY(vrgb);
	float b = XMVectorGetZ(vrgb);
	float t = b - r;
	float m = (r - g*2.0f + b) * 0.5f;
	float i = (r + g + b) / 3.0f;
	return XMVectorSet(t, m, i, 0.0f);
}

XMVECTOR TMI_to_RGB(XMVECTOR vtmi) {
	float t = XMVectorGetX(vtmi);
	float m = XMVectorGetY(vtmi);
	float i = XMVectorGetZ(vtmi);
	float r = i - t*0.5f + m/3.0f;
	float g = i - m*2.0f/3.0f;
	float b = i + t*0.5f + m/3.0f;
	return XMVectorSet(r, g, b, 1.0f);
}

static XMMATRIX s_tm709 = {
	{0.412453f, 0.212671f, 0.019334f, 0.0f},
	{0.357580f, 0.715160f, 0.119193f, 0.0f},
	{0.180423f, 0.072169f, 0.950227f, 0.0f},
	{0.0f, 0.0f, 0.0f, 1.0f}
};

static XMMATRIX s_im709 = {
	{ 3.240479f, -0.969256f,  0.055648f, 0.0f},
	{-1.537150f,  1.875992f, -0.204043f, 0.0f},
	{-0.498535f,  0.041556f,  1.057311f, 0.0f},
	{0.0f, 0.0f, 0.0f, 1.0f}
};

static inline cxMtx* mtx_RGB2XYZ(cxMtx* pRGB2XYZ) {
	if (!pRGB2XYZ) {
		pRGB2XYZ = (cxMtx*)&s_tm709;
	}
	return pRGB2XYZ;
}

static inline cxMtx* mtx_XYZ2RGB(cxMtx* pXYZ2RGB) {
	if (!pXYZ2RGB) {
		pXYZ2RGB = (cxMtx*)&s_im709;
	}
	return pXYZ2RGB;
}

XMVECTOR RGB_to_XYZ(XMVECTOR vrgb, cxMtx* pRGB2XYZ) {
	pRGB2XYZ = mtx_RGB2XYZ(pRGB2XYZ);
	return pRGB2XYZ->calc_vec(vrgb).get_xv();
}

XMVECTOR XYZ_to_RGB(XMVECTOR vxyz, cxMtx* pXYZ2RGB) {
	pXYZ2RGB = mtx_XYZ2RGB(pXYZ2RGB);
	return pXYZ2RGB->calc_vec(vxyz).get_xv_pnt(); /* A = 1 */
}

XMVECTOR XYZ_to_Lab(XMVECTOR vxyz, cxMtx* pRGB2XYZ) {
	pRGB2XYZ = mtx_RGB2XYZ(pRGB2XYZ);
	XMVECTOR white = RGB_to_XYZ(XMVectorReplicate(1.0f), pRGB2XYZ);
	XMVECTOR lv = vxyz * nxVec::rcp0(white);
	XMVECTOR t0 = lv*XMVectorReplicate(7.787f) + XMVectorReplicate(16.0f / 116);
	XMVECTOR t1 = nxVec::cb_root(lv);
	XMVECTOR m = _mm_cmple_ps(lv, XMVectorReplicate(0.008856f));
	lv = _mm_or_ps(_mm_and_ps(m, t0), _mm_andnot_ps(m, t1));
	float lx = XMVectorGetX(lv);
	float ly = XMVectorGetY(lv);
	float lz = XMVectorGetZ(lv);
	return XMVectorSet(116.0f*ly - 16.0f, 500.0f*(lx-ly), 200.0f*(ly-lz), 0.0f);
}

XMVECTOR Lab_to_XYZ(XMVECTOR vlab, cxMtx* pRGB2XYZ) {
	pRGB2XYZ = mtx_RGB2XYZ(pRGB2XYZ);
	XMVECTOR white = RGB_to_XYZ(XMVectorReplicate(1.0f), pRGB2XYZ);
	float L = XMVectorGetX(vlab);
	float a = XMVectorGetY(vlab);
	float b = XMVectorGetZ(vlab);
	float ly = (L + 16.0f) / 116.0f;
	XMVECTOR t = XMVectorSet(a/500.0f + ly, ly, -b/200.0f + ly, 0.0f);
	XMVECTOR t0 = (t - XMVectorReplicate(16.0f/116)) / XMVectorReplicate(7.787f);
	XMVECTOR t1 = t*t*t;
	XMVECTOR m = _mm_cmple_ps(t, XMVectorReplicate(0.206893f));
	t = _mm_or_ps(_mm_and_ps(m, t0), _mm_andnot_ps(m, t1));
	return t*white;
}

XMVECTOR RGB_to_Lab(XMVECTOR vrgb, cxMtx* pRGB2XYZ) {
	return XYZ_to_Lab(RGB_to_XYZ(vrgb, pRGB2XYZ), pRGB2XYZ);
}

XMVECTOR Lab_to_RGB(XMVECTOR vlab, cxMtx* pRGB2XYZ, cxMtx* pXYZ2RGB) {
	return XYZ_to_RGB(Lab_to_XYZ(vlab, pRGB2XYZ), pXYZ2RGB);
}

XMVECTOR Lab_to_Lch(XMVECTOR vlab) {
	float L = XMVectorGetX(vlab);
	float a = XMVectorGetY(vlab);
	float b = XMVectorGetZ(vlab);
	float c = nxCalc::hypot(a, b);
	float h = ::atan2f(b, a);
	return XMVectorSet(L, c, h, 0.0f);
}

XMVECTOR Lch_to_Lab(XMVECTOR vlch) {
	float L = XMVectorGetX(vlch);
	float c = XMVectorGetY(vlch);
	float h = XMVectorGetZ(vlch);
	float hs = ::sinf(h);
	float hc = ::cosf(h);
	return XMVectorSet(L, c*hc, c*hs, 0.0f);
}

void init_XYZ_transform(cxMtx* pRGB2XYZ, cxMtx* pXYZ2RGB, cxVec* pPrims, cxVec* pWhite) {
	cxVec rx, ry, rz;
	if (pPrims) {
		rx = pPrims[0];
		ry = pPrims[1];
		rz = pPrims[2];
	} else {
		/* 709 primaries */
		rx.set(0.640f, 0.330f, 0.030f);
		ry.set(0.300f, 0.600f, 0.100f);
		rz.set(0.150f, 0.060f, 0.790f);
	}

	cxVec w;
	if (pWhite) {
		w = *pWhite;
	} else {
		/* D65 */
		w.set(0.3127f, 0.3290f, 0.3582f);
	}
	w.scl(nxCalc::rcp0(w.y));

	cxMtx cm;
	cm.set_rot_frame(rx, ry, rz);

	cxMtx im = cm.get_inverted();
	cxVec j = im.calc_vec(w);
	cxMtx tm;
	tm.mk_scl(j);
	tm.mul(cm);
	if (pRGB2XYZ) {
		*pRGB2XYZ = tm;
	}
	if (pXYZ2RGB) {
		*pXYZ2RGB = tm.get_inverted();
	}
}

} // nxColor


float cxColor::luma() const {
	return nxColor::luma(mRGBA);
}

float cxColor::luminance() const {
	return nxColor::luminance(mRGBA);
}

XD_NOINLINE void cxColor::make_linear() {
	mRGBA = XMColorSRGBToRGB(mRGBA);
}

XD_NOINLINE void cxColor::make_sRGB() {
	mRGBA = XMColorRGBToSRGB(mRGBA);
}

XMVECTOR cxColor::to_HSV() const {
	return XMColorRGBToHSV(mRGBA);
}

void cxColor::from_HSV(XMVECTOR hsv) {
	mRGBA = XMColorHSVToRGB(hsv);
}

XMVECTOR cxColor::to_YCgCo() const {
	return nxColor::RGB_to_YCgCo(mRGBA);
}

void cxColor::from_YCgCo(XMVECTOR ygo) {
	mRGBA = nxColor::YCgCo_to_RGB(ygo);
}

XMVECTOR cxColor::to_TMI() const {
	return nxColor::RGB_to_TMI(mRGBA);
}

void cxColor::from_TMI(XMVECTOR tmi) {
	mRGBA = nxColor::TMI_to_RGB(tmi);
}


static cxVec tball_proj(float x, float y, float r) {
	float d = ::hypotf(x, y);
	float t = r / ::sqrtf(2.0f);
	float z;
	if (d < t) {
		z = ::sqrtf(r*r - d*d);
	} else {
		z = (t*t) / d;
	}
	return cxVec(x, y, z);
}

void cxTrackball::update(float x0, float y0, float x1, float y1) {
	cxVec tp0 = tball_proj(x0, y0, mRadius);
	cxVec tp1 = tball_proj(x1, y1, mRadius);
	cxVec dir = tp0 - tp1;
	cxVec axis = nxVec::cross(tp1, tp0);
	float t = nxCalc::clamp(dir.mag() / (2.0f*mRadius), -1.0f, 1.0f);
	float ang = 2.0f * ::asinf(t);
	mSpin.set_rot(axis, ang);
	mQuat.mul(mSpin);
	mQuat.normalize();
}


namespace nxGeom {

bool seg_seg_overlap_2d(float s0x0, float s0y0, float s0x1, float s0y1, float s1x0, float s1y0, float s1x1, float s1y1) {
	bool res = false;
	float a1 = signed_tri_area_2d(s0x0, s0y0, s0x1, s0y1, s1x1, s1y1);
	float a2 = signed_tri_area_2d(s0x0, s0y0, s0x1, s0y1, s1x0, s1y0);
	if (a1*a2 < 0.0f) {
		float a3 = signed_tri_area_2d(s1x0, s1y0, s1x1, s1y1, s0x0, s0y0);
		float a4 = a3 + a2 - a1;
		if (a3*a4 < 0.0f) {
			res = true;
		}
	}
	return res;
}

bool seg_plane_intersect(const cxVec& p0, const cxVec& p1, const cxPlane& pln, float* pT) {
	cxVec d = p1 - p0;
	cxVec n = pln.get_normal();
	float dn = d.dot(n);
	float t = (pln.get_D() - n.dot(p0)) / dn;
	bool res = t >= 0.0f && t <= 1.0f;
	if (res && pT) {
		*pT = t;
	}
	return res;
}

bool seg_quad_intersect_cw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, cxVec* pHitPos, cxVec* pHitNrm) {
	return seg_quad_intersect_ccw(p0, p1, v3, v2, v1, v0, pHitPos, pHitNrm);
}

bool seg_quad_intersect_cw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, const cxVec& nrm, cxVec* pHitPos) {
	return seg_quad_intersect_ccw_n(p0, p1, v3, v2, v1, v0, nrm, pHitPos);
}

bool seg_quad_intersect_ccw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, cxVec* pHitPos, cxVec* pHitNrm) {
	cxVec vec[4];
	cxVec edge[4];
	edge[0] = v1 - v0;
	cxVec n = nxVec::cross(edge[0], v2 - v0).get_normalized();
	vec[0] = p0 - v0;
	float d0 = vec[0].dot(n);
	float d1 = (p1 - v0).dot(n);
	if (d0*d1 > 0.0f || (d0 == 0.0f && d1 == 0.0f)) return false;
	edge[1] = v2 - v1;
	edge[2] = v3 - v2;
	edge[3] = v0 - v3;
	vec[1] = p0 - v1;
	vec[2] = p0 - v2;
	vec[3] = p0 - v3;
	cxVec dir = p1 - p0;
	if (nxVec::scalar_triple(edge[0], dir, vec[0]) < 0.0f) return false;
	if (nxVec::scalar_triple(edge[1], dir, vec[1]) < 0.0f) return false;
	if (nxVec::scalar_triple(edge[2], dir, vec[2]) < 0.0f) return false;
	if (nxVec::scalar_triple(edge[3], dir, vec[3]) < 0.0f) return false;
	float d = dir.dot(n);
	float t;
	if (d == 0.0f || d0 == 0.0f) {
		t = 0.0f;
	} else {
		t = -d0 / d;
	}
	if (t > 1.0f || t < 0.0f) return false;
	if (pHitPos) {
		*pHitPos = p0 + dir*t;
	}
	if (pHitNrm) {
		*pHitNrm = n;
	}
	return true;
}

bool seg_quad_intersect_ccw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3, const cxVec& nrm, cxVec* pHitPos) {
	cxVec vec[4];
	cxVec edge[4];
	vec[0] = p0 - v0;
	float d0 = vec[0].dot(nrm);
	float d1 = (p1 - v0).dot(nrm);
	if (d0*d1 > 0.0f || (d0 == 0.0f && d1 == 0.0f)) return false;
	edge[0] = v1 - v0;
	edge[1] = v2 - v1;
	edge[2] = v3 - v2;
	edge[3] = v0 - v3;
	vec[1] = p0 - v1;
	vec[2] = p0 - v2;
	vec[3] = p0 - v3;
	cxVec dir = p1 - p0;
	if (nxVec::scalar_triple(edge[0], dir, vec[0]) < 0.0f) return false;
	if (nxVec::scalar_triple(edge[1], dir, vec[1]) < 0.0f) return false;
	if (nxVec::scalar_triple(edge[2], dir, vec[2]) < 0.0f) return false;
	if (nxVec::scalar_triple(edge[3], dir, vec[3]) < 0.0f) return false;
	float d = dir.dot(nrm);
	float t;
	if (d == 0.0f || d0 == 0.0f) {
		t = 0.0f;
	} else {
		t = -d0 / d;
	}
	if (t > 1.0f || t < 0.0f) return false;
	if (pHitPos) {
		*pHitPos = p0 + dir*t;
	}
	return true;
}

bool seg_polyhedron_intersect(const cxVec& p0, const cxVec& p1, const cxPlane* pPln, int plnNum, float* pFirst, float* pLast) {
	float first = 0.0f;
	float last = 1.0f;
	bool res = false;
	cxVec d = p1 - p0;
	for (int i = 0; i < plnNum; ++i) {
		cxVec nrm = pPln[i].get_normal();
		float dnm = d.dot(nrm);
		float dist = pPln[i].get_D() - nrm.dot(p0);
		if (dnm == 0.0f) {
			if (dist < 0.0f) goto _exit;
		} else {
			float t = dist / dnm;
			if (dnm < 0.0f) {
				if (t > first) first = t;
			} else {
				if (t < last) last = t;
			}
			if (first > last) goto _exit;
		}
	}
	res = true;
_exit:
	if (pFirst) {
		*pFirst = first;
	}
	if (pLast) {
		*pLast = last;
	}
	return res;
}

bool seg_cylinder_intersect(const cxVec& p0, const cxVec& p1, const cxVec& cp0, const cxVec& cp1, float cr, cxVec* pHitPos, float* pHitDist) {
	bool res = false;
	float t = 0.0f;
	cxVec d = cp1 - cp0;
	cxVec m = p0 - cp0;
	cxVec n = p1 - p0;
	float md = m.dot(d);
	float nd = n.dot(d);
	float dd = d.dot(d);
	if (md < 0.0f && md + nd < 0.0f) return false;
	if (md > dd && md + nd > dd) return false;
	float nn = n.dot(n);
	float mn = m.dot(n);
	float mm = m.dot(m);
	float a = dd*nn - nxCalc::sq(nd);
	float k = mm - nxCalc::sq(cr);
	float c = dd*k - nxCalc::sq(md);
	if (::fabsf(a) < 1e-5f) {
		if (c <= 0.0f) {
			if (md < 0.0f) {
				t = -mn / nn;
			} else if (md > dd) {
				t = (nd - mn) / nn;
			}
			res = true;
		}
	} else {
		float b = dd*mn - nd*md;
		float dsc = nxCalc::sq(b) - a*c;
		if (dsc >= 0.0f) {
			t = (-b - ::sqrtf(dsc)) / a;
			if (t >= 0.0f && t <= 1.0f) {
				float q = md + t*nd;
				if (q < 0.0f) {
					if (nd > 0.0f) {
						t = -md / nd;
						if (k + 2.0f*t*(mn + t*nn) <= 0.0f) {
							res = true;
						}
					}
				} else if (q > dd) {
					if (nd < 0.0f) {
						t = (dd - md) / nd;
						if (k + dd - 2 * md + t*(2.0f*(mn - nd) + t*nn) <= 0.0f) {
							res = true;
						}
					}
				} else {
					res = true;
				}
			}
		}
	}
	if (res) {
		if (pHitPos) {
			*pHitPos = nxVec::lerp(p0, p1, t);
		}
		if (pHitDist) {
			*pHitDist = t;
		}
	}
	return res;
}

bool seg_tri_intersect_cw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& nrm, cxVec* pHitPos) {
	return seg_quad_intersect_cw_n(p0, p1, v0, v1, v2, v0, nrm, pHitPos);
}

bool seg_tri_intersect_cw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, cxVec* pHitPos, cxVec* pHitNrm) {
	return seg_quad_intersect_cw(p0, p1, v0, v1, v2, v0, pHitPos, pHitNrm);
}


bool seg_tri_intersect_ccw_n(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& nrm, cxVec* pHitPos) {
	return seg_quad_intersect_ccw_n(p0, p1, v0, v1, v2, v0, nrm, pHitPos);
}

bool seg_tri_intersect_ccw(const cxVec& p0, const cxVec& p1, const cxVec& v0, const cxVec& v1, const cxVec& v2, cxVec* pHitPos, cxVec* pHitNrm) {
	return seg_quad_intersect_ccw(p0, p1, v0, v1, v2, v0, pHitPos, pHitNrm);
}

cxVec barycentric(const cxVec& pos, const cxVec& v0, const cxVec& v1, const cxVec& v2) {
	cxVec dv0 = pos - v0;
	cxVec dv1 = v1 - v0;
	cxVec dv2 = v2 - v0;
	float d01 = dv0.dot(dv1);
	float d02 = dv0.dot(dv2);
	float d12 = dv1.dot(dv2);
	float d11 = dv1.dot(dv1);
	float d22 = dv2.dot(dv2);
	float d = d11*d22 - nxCalc::sq(d12);
	float ood = 1.0f / d;
	float u = (d01*d22 - d02*d12) * ood;
	float v = (d02*d11 - d01*d12) * ood;
	float w = (1.0f - u - v);
	return cxVec(u, v, w);
}

int quad_convex_ck(const cxVec& v0, const cxVec& v1, const cxVec& v2, const cxVec& v3) {
	int res = 0;
	cxVec e0 = v1 - v0;
	cxVec e1 = v2 - v1;
	cxVec e2 = v3 - v2;
	cxVec e3 = v0 - v3;
	cxVec n01 = nxVec::cross(e0, e1);
	cxVec n23 = nxVec::cross(e2, e3);
	if (n01.dot(n23) > 0.0f) res |= 1;
	cxVec n12 = nxVec::cross(e1, e2);
	cxVec n30 = nxVec::cross(e3, e0);
	if (n12.dot(n30) > 0.0f) res |= 2;
	return res;
}

float quad_dist2(const cxVec& pos, const cxVec vtx[4]) {
	int i;
	float t;
	cxVec v[4];
	cxVec e[4];
	for (i = 0; i < 4; ++i) {
		v[i] = pos - vtx[i];
		e[i] = vtx[i<3 ? i + 1 : 0] - vtx[i];
	}
	cxVec n = nxVec::cross(e[0], vtx[2] - vtx[0]).get_normalized();
	XMVECTOR dv = XMVectorZero();
	for (i = 0; i < 4; ++i) {
		t = nxVec::scalar_triple(e[i], v[i], n);
		dv = XMVectorSetByIndex(dv, t, i);
	}
	uint32_t cr;
	XMVectorGreaterOrEqualR(&cr, dv, XMVectorZero());
	if (XMComparisonAllTrue(cr)) {
		return nxCalc::sq(v[0].dot(n));
	}
	if (::fabsf(XMVectorGetW(dv)) < 1e-6f) {
		dv = XMVectorSetW(dv, XMVectorGetZ(dv)); /* dv[3] = dv[2] */
		e[3] = e[2];
	}
	for (i = 0; i < 4; ++i) {
		t = e[i].mag2();
		dv = XMVectorSetByIndex(dv, t, i);
	}
	XMVECTOR vlen = XMVectorSqrt(dv);
	XMVECTOR ilen = nxVec::rcp0(vlen);
	for (i = 0; i < 4; ++i) {
		e[i].scl(XMVectorGetByIndex(ilen, i));
		t = v[i].dot(e[i]);
		dv = XMVectorSetByIndex(dv, t, i);
	}
	dv = XMVectorClamp(dv, XMVectorZero(), vlen);
	for (i = 0; i < 4; ++i) {
		e[i].scl(XMVectorGetByIndex(dv, i));
		v[i] = vtx[i] + e[i];
	}

	return nxCalc::min(nxVec::dist2(pos, v[0]), nxVec::dist2(pos, v[1]), nxVec::dist2(pos, v[2]), nxVec::dist2(pos, v[3]));
}

void update_nrm_newell(cxVec* pNrm, cxVec* pVtxI, cxVec* pVtxJ) {
	cxVec dv = *pVtxI - *pVtxJ;
	cxVec sv = *pVtxI + *pVtxJ;
	*pNrm += cxVec(dv[1], dv[2], dv[0]) * cxVec(sv[2], sv[0], sv[1]);
}

cxVec poly_normal_cw(cxVec* pVtx, int vtxNum) {
	cxVec nrm;
	nrm.zero();
	for (int i = 0; i < vtxNum; ++i) {
		int j = i - 1;
		if (j < 0) j = vtxNum - 1;
		update_nrm_newell(&nrm, &pVtx[i], &pVtx[j]);
	}
	nrm.normalize();
	return nrm;
}

cxVec poly_normal_ccw(cxVec* pVtx, int vtxNum) {
	cxVec nrm;
	nrm.zero();
	for (int i = 0; i < vtxNum; ++i) {
		int j = i + 1;
		if (j >= vtxNum) j = 0;
		update_nrm_newell(&nrm, &pVtx[i], &pVtx[j]);
	}
	nrm.normalize();
	return nrm;
}

float seg_seg_dist2(const cxVec& s0p0, const cxVec& s0p1, const cxVec& s1p0, const cxVec& s1p1, cxVec* pBridgeP0, cxVec* pBridgeP1) {
	static float eps = 1e-6f;
	float t0 = 0.0f;
	float t1 = 0.0f;
	cxVec dir0 = s0p1 - s0p0;
	cxVec dir1 = s1p1 - s1p0;
	cxVec vec = s0p0 - s1p0;
	float len0 = dir0.mag2();
	float len1 = dir1.mag2();
	float vd1 = vec.dot(dir1);
	if (len0 <= eps) {
		if (len1 > eps) {
			t1 = nxCalc::saturate(vd1 / len1);
		}
	} else {
		float vd0 = vec.dot(dir0);
		if (len1 <= eps) {
			t0 = nxCalc::saturate(-vd0 / len0);
		} else {
			float dd = dir0.dot(dir1);
			float dn = len0*len1 - nxCalc::sq(dd);
			if (dn != 0.0f) {
				t0 = nxCalc::saturate((dd*vd1 - vd0*len1) / dn);
			}
			t1 = (dd*t0 + vd1) / len1;
			if (t1 < 0.0f) {
				t0 = nxCalc::saturate(-vd0 / len0);
				t1 = 0.0f;
			} else if (t1 > 1.0f) {
				t0 = nxCalc::saturate((dd - vd0) / len0);
				t1 = 1.0f;
			}
		}
	}
	cxVec bp0 = s0p0 + dir0*t0;
	cxVec bp1 = s1p0 + dir1*t1;
	if (pBridgeP0) {
		*pBridgeP0 = bp0;
	}
	if (pBridgeP1) {
		*pBridgeP1 = bp1;
	}
	return (bp1 - bp0).mag2();
}

bool cap_aabb_overlap(const cxVec& cp0, const cxVec& cp1, float cr, const cxVec& bmin, const cxVec& bmax) {
	cxVec brad = (bmax - bmin) * 0.5f;
	cxVec bcen = (bmin + bmax) * 0.5f;
	cxVec ccen = (cp0 + cp1) * 0.5f;
	cxVec h = (cp1 - cp0) * 0.5f;
	cxVec v = ccen - bcen;
	cxVec d0 = v.abs_val();
	cxVec d1 = h.abs_val() + brad + cxVec(cr);
	if (d0.gt(d1)) return false;
	cxVec tpos;
	line_pnt_closest(cp0, cp1, bcen, &tpos);
	cxVec a = (tpos - bcen).get_normalized();
	if (::fabsf(v.dot(a)) > ::fabsf(brad.dot(a)) + cr) return false;
	return true;
}

} // nxGeom


namespace nxColl {

bool separate_sph_sph(const cxSphere& mvSph, cxVec& vel, cxSphere& stSph, cxVec* pSepVec, float margin) {
	cxVec sepVec;
	sepVec.zero();
	bool flg = mvSph.overlaps(stSph);
	if (flg) {
		float sepDist = mvSph.get_radius() + stSph.get_radius() + margin;
		cxVec sepDir = vel.neg_val();
		cxVec dv = mvSph.get_center() - stSph.get_center();
		cxVec vec = dv + sepDir*dv.mag();
		float len = vec.mag();
		if (len < 1e-5f) {
			vec = sepDir.get_normalized();
		} else {
			sepDist /= len;
		}
		sepVec = vec*sepDist - dv;
	}
	if (pSepVec) {
		*pSepVec = sepVec;
	}
	return flg;
}

bool adjust_sph_sph(const cxSphere& mvSph, cxVec& vel, cxSphere& stSph, cxVec* pAdjPos, float margin) {
	cxVec sepVec;
	cxVec tstPos = mvSph.get_center();
	cxVec adjPos = tstPos;
	bool flg = separate_sph_sph(mvSph, vel, stSph, &sepVec, margin);
	if (flg) {
		cxVec nv = (tstPos + sepVec - stSph.get_center()).get_normalized();
		cxVec rv = nxVec::reflect(vel, nv);
		adjPos += rv;
		cxSphere tstSph(adjPos, mvSph.get_radius());
		if (tstSph.overlaps(stSph)) {
			adjPos = tstPos + sepVec;
		}
	}
	if (pAdjPos) {
		*pAdjPos = adjPos;
	}
	return flg;
}

bool separate_sph_cap(const cxSphere& mvSph, cxVec& vel, cxCapsule& stCap, cxVec* pSepVec, float margin) {
	cxVec sepVec;
	sepVec.zero();
	cxVec axisPnt;
	bool flg = mvSph.overlaps(stCap, &axisPnt);
	if (flg) {
		float sepDist = mvSph.get_radius() + stCap.get_radius() + margin;
		cxVec sepDir = vel.neg_val();
		cxVec dv = mvSph.get_center() - axisPnt;
		cxVec vec = dv + sepDir*dv.mag();
		float len = vec.mag();
		if (len < 1e-5f) {
			vec = sepDir.get_normalized();
		} else {
			sepDist /= len;
		}
		sepVec = vec*sepDist - dv;
	}
	return flg;
}

} // nxColl


bool cxSphere::overlaps(const cxSphere& sph) const {
	return nxGeom::sph_sph_overlap(get_center(), get_radius(), sph.get_center(), sph.get_radius());
}

bool cxSphere::overlaps(const cxAABB& box) const {
	return nxGeom::sph_aabb_overlap(get_center(), get_radius(), box.get_min_pos(), box.get_max_pos());
}

bool cxSphere::overlaps(const cxCapsule& cap, cxVec* pCapAxisPos) const {
	return nxGeom::sph_cap_overlap(get_center(), get_radius(), cap.get_pos0(), cap.get_pos1(), cap.get_radius(), pCapAxisPos);
}


void cxAABB::transform(const cxAABB& box, const cxMtx& mtx) {
	XMVECTOR omin = box.mMin.get_xv();
	XMVECTOR omax = box.mMax.get_xv();
	XMVECTOR nmin = mtx.get_row(3);
	XMVECTOR nmax = nmin;
	XMVECTOR va, ve, vf;

	va = mtx.get_row(0);
	ve = va * XMVectorSplatX(omin);
	vf = va * XMVectorSplatX(omax);
	nmin += XMVectorMin(ve, vf);
	nmax += XMVectorMax(ve, vf);

	va = mtx.get_row(1);
	ve = va * XMVectorSplatY(omin);
	vf = va * XMVectorSplatY(omax);
	nmin += XMVectorMin(ve, vf);
	nmax += XMVectorMax(ve, vf);

	va = mtx.get_row(2);
	ve = va * XMVectorSplatZ(omin);
	vf = va * XMVectorSplatZ(omax);
	nmin += XMVectorMin(ve, vf);
	nmax += XMVectorMax(ve, vf);

	mMin.set_xv(nmin);
	mMax.set_xv(nmax);
}

bool cxAABB::seg_ck(const cxVec& p0, const cxVec& p1) const {
	cxVec dir = p1 - p0;
	float len = dir.mag();
	if (len < FLT_EPSILON) {
		return contains(p0);
	}
	XMVECTOR dv = nxVec::div0(XMVectorReplicate(len), dir.get_xv());
	XMVECTOR t1 = (mMin.get_xv() - p0.get_xv()) * dv;
	XMVECTOR t2 = (mMax.get_xv() - p0.get_xv()) * dv;
	XMVECTOR vmin = XMVectorMin(t1, t2);
	XMVECTOR vmax = XMVectorMax(t1, t2);
	float tmin = 0.0f;
	float tmax = len;

	if (dir.x != 0) {
		tmin = nxCalc::max(tmin, XMVectorGetX(vmin));
		tmax = nxCalc::min(tmax, XMVectorGetX(vmax));
		if (tmin > tmax) {
			return false;
		}
	} else {
		if (p0.x < mMin.x || p0.x > mMax.x) return false;
	}

	if (dir.y != 0) {
		tmin = nxCalc::max(tmin, XMVectorGetY(vmin));
		tmax = nxCalc::min(tmax, XMVectorGetY(vmax));
		if (tmin > tmax) {
			return false;
		}
	} else {
		if (p0.y < mMin.y || p0.y > mMax.y) return false;
	}

	if (dir.z != 0) {
		tmin = nxCalc::max(tmin, XMVectorGetZ(vmin));
		tmax = nxCalc::min(tmax, XMVectorGetZ(vmax));
		if (tmin > tmax) {
			return false;
		}
	} else {
		if (p0.z < mMin.z || p0.z > mMax.z) return false;
	}

	if (tmax > len) return false;
	return true;
}

bool cxAABB::overlaps(const cxCapsule& cap) const {
	return nxGeom::cap_aabb_overlap(cap.get_pos0(), cap.get_pos1(), cap.get_radius(), mMin, mMax);
}


bool cxCapsule::overlaps(const cxAABB& box) const {
	return nxGeom::cap_aabb_overlap(mPos0, mPos1, mRadius, box.get_min_pos(), box.get_max_pos());
}

bool cxCapsule::overlaps(const cxCapsule& cap) const {
	return nxGeom::cap_cap_overlap(mPos0, mPos1, mRadius, cap.mPos0, cap.mPos1, cap.mRadius);
}

void cxAABB::get_polyhedron(cxPlane* pPln) const {
	if (!pPln) return;
	cxVec vmin = get_min_pos();
	cxVec vmax = get_max_pos();
	pPln[0].calc(vmin, nxVec::get_axis(exAxis::MINUS_Y)); /* bottom */
	pPln[1].calc(vmax, nxVec::get_axis(exAxis::PLUS_Y)); /* top */
	pPln[2].calc(vmin, nxVec::get_axis(exAxis::MINUS_X)); /* left */
	pPln[3].calc(vmax, nxVec::get_axis(exAxis::PLUS_X)); /* right */
	pPln[4].calc(vmin, nxVec::get_axis(exAxis::MINUS_Z)); /* far */
	pPln[5].calc(vmax, nxVec::get_axis(exAxis::PLUS_Z)); /* near */
}


void cxFrustum::calc_normals() {
	mNrm[0] = nxGeom::tri_normal_cw(mPnt[0], mPnt[1], mPnt[3]); /* near */
	mNrm[1] = nxGeom::tri_normal_cw(mPnt[0], mPnt[3], mPnt[4]); /* left */
	mNrm[2] = nxGeom::tri_normal_cw(mPnt[0], mPnt[4], mPnt[5]); /* top */
	mNrm[3] = nxGeom::tri_normal_cw(mPnt[6], mPnt[2], mPnt[1]); /* right */
	mNrm[4] = nxGeom::tri_normal_cw(mPnt[6], mPnt[7], mPnt[3]); /* bottom */
	mNrm[5] = nxGeom::tri_normal_cw(mPnt[6], mPnt[5], mPnt[4]); /* far */
}

void cxFrustum::calc_planes() {
	for (int i = 0; i < 6; ++i) {
		static int vtxNo[] = { 0, 0, 0, 6, 6, 6 };
		mPlane[i].calc(mPnt[vtxNo[i]], mNrm[i]);
	}
}

void cxFrustum::init(const cxMtx& mtx, float fovy, float aspect, float znear, float zfar) {
	int i;
	float x, y, z;
	float t = ::tanf(fovy * 0.5f);
	z = znear;
	y = t * z;
	x = y * aspect;
	mPnt[0].set(-x,  y, -z);
	mPnt[1].set( x,  y, -z);
	mPnt[2].set( x, -y, -z);
	mPnt[3].set(-x, -y, -z);
	z = zfar;
	y = t * z;
	x = y * aspect;
	mPnt[4].set(-x,  y, -z);
	mPnt[5].set( x,  y, -z);
	mPnt[6].set( x, -y, -z);
	mPnt[7].set(-x, -y, -z);
	calc_normals();
	for (i = 0; i < 8; ++i) {
		mPnt[i] = mtx.calc_pnt(mPnt[i]);
	}
	for (i = 0; i < 6; ++i) {
		mNrm[i] = mtx.calc_vec(mNrm[i]);
	}
	calc_planes();
}

cxVec cxFrustum::get_center() const {
	cxVec c = mPnt[0];
	for (int i = 1; i < 8; ++i) {
		c += mPnt[i];
	}
	c /= 8.0f;
	return c;
}

bool cxFrustum::cull(const cxSphere& sph) const {
	cxVec c = sph.get_center();
	float r = sph.get_radius();
	cxVec v = c - mPnt[0];
	for (int i = 0; i < 6; ++i) {
		if (i == 3) v = c - mPnt[6];
		float d = v.dot(mNrm[i]);
		if (r < d) return true;
	}
	return false;
}

bool cxFrustum::overlaps(const cxSphere& sph) const {
	float sd = FLT_MAX;
	bool flg = false;
	cxVec c = sph.get_center();
	for (int i = 0; i < 6; ++i) {
		if (mPlane[i].signed_dist(c) > 0.0f) {
			static uint16_t vtxTbl[6] = {
				0x0123,
				0x0374,
				0x1045,
				0x5621,
				0x6732,
				0x4765
			};
			uint32_t vidx = vtxTbl[i];
			cxVec vtx[4];
			for (int j = 0; j < 4; ++j) {
				vtx[j] = mPnt[vidx & 0xF];
				vidx >>= 4;
			}
			float dist = nxGeom::quad_dist2(c, vtx);
			sd = nxCalc::min(sd, dist);
			flg = true;
		}
	}
	if (!flg) return true;
	if (sd <= nxCalc::sq(sph.get_radius())) return true;
	return false;
}

bool cxFrustum::cull(const cxAABB& box) const {
	cxVec c = box.get_center();
	cxVec r = box.get_max_pos() - c;
	cxVec v = c - mPnt[0];
	for (int i = 0; i < 6; ++i) {
		cxVec n = mNrm[i];
		if (i == 3) v = c - mPnt[6];
		if (r.dot(n.abs_val()) < v.dot(n)) return true;
	}
	return false;
}

bool cxFrustum::overlaps(const cxAABB& box) const {
	static struct {
		uint8_t i0, i1;
	} edgeTbl[] = {
		{ 0, 1 }, { 1, 2 }, { 3, 4 }, { 3, 0 },
		{ 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
		{ 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
	};
	for (int i = 0; i < 12; ++i) {
		cxVec p0 = mPnt[edgeTbl[i].i0];
		cxVec p1 = mPnt[edgeTbl[i].i1];
		if (box.seg_ck(p0, p1)) return true;
	}
	const int imin = 0;
	const int imax = 1;
	cxVec bb[2];
	bb[imin] = box.get_min_pos();
	bb[imax] = box.get_max_pos();
	static struct {
		uint8_t i0x, i0y, i0z, i1x, i1y, i1z;
	} boxEdgeTbl[] = {
		{ imin, imin, imin, imax, imin, imin },
		{ imax, imin, imin, imax, imin, imax },
		{ imax, imin, imax, imin, imin, imax },
		{ imin, imin, imax, imin, imin, imin },

		{ imin, imax, imin, imax, imax, imin },
		{ imax, imax, imin, imax, imax, imax },
		{ imax, imax, imax, imin, imax, imax },
		{ imin, imax, imax, imin, imax, imin },

		{ imin, imin, imin, imin, imax, imin },
		{ imax, imin, imin, imax, imax, imin },
		{ imax, imin, imax, imax, imax, imax },
		{ imin, imin, imax, imin, imax, imax }
	};
	for (int i = 0; i < 12; ++i) {
		float p0x = bb[boxEdgeTbl[i].i0x].x;
		float p0y = bb[boxEdgeTbl[i].i0y].y;
		float p0z = bb[boxEdgeTbl[i].i0z].z;
		float p1x = bb[boxEdgeTbl[i].i1x].x;
		float p1y = bb[boxEdgeTbl[i].i1y].y;
		float p1z = bb[boxEdgeTbl[i].i1z].z;
		cxVec p0(p0x, p0y, p0z);
		cxVec p1(p1x, p1y, p1z);
		if (nxGeom::seg_polyhedron_intersect(p0, p1, mPlane, 6)) return true;
	}
	return false;
}


XD_NOINLINE void cxBasicIK::ChainInfo::reset() {
	set_axis_leg();
	mTopOffs.zero();
	mRotOffs.set(0.0f, -1.0f, 0.0f);
	mEndOffs.set(0.0f, -1.0f, 0.0f);
	mExtOffs.zero();
	mFlags.mAll = 0;
	mFlags.mCalcEnd = true;
}


void cxBasicIK::init(const ChainInfo& info) {
	mInfo = info;
	mLen0 = mInfo.mRotOffs.mag();
	mLen1 = mInfo.mEndOffs.mag();
}

void cxBasicIK::solve(ChainResult* pRes, const ChainPose& pose) {
	if (!pRes) return;

	cxMtx topMtx;
	topMtx.from_quat_and_pos(pose.mTopRot, mInfo.mTopOffs);
	topMtx.mul(pose.mParentW);

	cxVec effPos = pose.get_eff_pos_w();
	cxVec endPos;
	cxMtx extMtx;
	if (mInfo.is_rev_foot()) {
		extMtx.from_quat(pose.mEndRot);
		extMtx.mul(pose.mRootW);
		extMtx.set_translation(effPos);
		cxVec extOffs = extMtx.calc_vec(mInfo.mExtOffs);
		endPos = effPos - extOffs;
	} else {
		endPos = effPos;
	}

	cxVec topPos = topMtx.get_translation();
	float len0 = mLen0;
	float len1 = mLen1;
	cxVec az = endPos - topPos;
	float dist = az.mag();
	float rot0 = 0.0f;
	float rot1 = 0.0f;
	if (dist < len0 + len1) {
		float c0 = ((len0*len0) - (len1*len1) + (dist*dist)) / (2.0f*len0*dist);
		float c1 = ((len0*len0) + (len1*len1) - (dist*dist)) / (2.0f*len0*len1);
		c0 = nxCalc::clamp(c0, -1.0f, 1.0f);
		c1 = nxCalc::clamp(c1, -1.0f, 1.0f);
		rot0 = -::acosf(c0);
		rot1 = XD_PI - ::acosf(c1);
	}

	cxVec ikAxis = nxVec::get_axis(mInfo.mDirAxis);
	cxVec ikUp = nxVec::get_axis(mInfo.mUpAxis);
	cxVec ikSide = nxVec::cross(ikUp, ikAxis);
	cxVec ax = topMtx.calc_vec(ikSide);
	ax.normalize();
	az.normalize();
	cxVec ay = nxVec::cross(az, ax).get_normalized();
	ax = nxVec::cross(ay, az).get_normalized();
	cxMtx ikMtx = nxMtx::mk_rot_frame(ikSide, ikUp, ikAxis).get_transposed_sr() * nxMtx::mk_rot_frame(ax, ay, az);

	cxMtx rotMtx;
	rotMtx.set_rot(ikSide, rot0);
	topMtx = rotMtx * ikMtx;
	topMtx.set_translation(topPos);
	cxVec rotPos = topMtx.calc_pnt(mInfo.mRotOffs);
	rotMtx.set_rot(ikSide, rot1);
	rotMtx.mul(topMtx);
	rotMtx.set_translation(rotPos);
	pRes->mTop = topMtx * pose.mParentW.get_inverted();
	pRes->mRot = rotMtx * topMtx.get_inverted();

	cxMtx endMtx;
	if (mInfo.is_calc_end() || mInfo.is_rev_foot()) {
		cxQuat q = pose.mEndRot;
		if (mInfo.is_rev_foot()) {
			q.mul(pose.mExtRot);
		}
		endMtx.from_quat(q);
		endMtx.mul(pose.mRootW);
		endMtx.set_translation(endPos);
	}
	if (mInfo.is_calc_end()) {
		pRes->mEnd = endMtx * rotMtx.get_inverted();
		if (mInfo.is_rev_foot()) {
			pRes->mExt = extMtx * endMtx.get_inverted();
		}
	}
}


// http://www.ppsloan.org/publications/SHJCGT.pdf
// http://jcgt.org/published/0002/02/06/
namespace nxSH {

double calc_K(int l, int m) {
	int am = ::abs(m);
	double v = 1.0;
	for (int k = l + am; k > l - am; --k) {
		v *= k;
	}
	return ::sqrt((2.0*l + 1.0) / (4.0*M_PI*v));
}

double calc_Pmm(int m) {
	double v = 1.0;
	for (int k = 0; k <= m; ++k) {
		v *= 1.0 - 2.0*k;
	}
	return v;
}

double calc_constA(int m) {
	return calc_Pmm(m) * calc_K(m, m) * ::sqrt(2.0);
}

double calc_constB(int m) {
	return double(2*m + 1) * calc_Pmm(m) * calc_K(m+1, m);
}

double calc_constC1(int l, int m) {
	double c = calc_K(l, m) / calc_K(l-1, m) * double(2*l-1) / double(l-m);
	if (l > 80) {
		if (::isnan(c)) c = 0.0;
	}
	return c;
}

double calc_constC2(int l, int m) {
	double c = -calc_K(l, m) / calc_K(l-2, m) * double(l+m-1) / double(l-m);
	if (l > 80) {
		if (::isnan(c)) c = 0.0;
	}
	return c;
}

double calc_constD1(int m) {
	return double((2*m+3)*(2*m+1)) * calc_Pmm(m) / 2.0 * calc_K(m+2, m);
}

double calc_constD2(int m) {
	return double(-(2*m + 1)) * calc_Pmm(m) / 2.0 * calc_K(m+2, m);
}

double calc_constE1(int m) {
	return double((2*m+5)*(2*m+3)*(2*m+1)) * calc_Pmm(m) / 6.0 * calc_K(m+3,m);
}

double calc_constE2(int m) {
	double pmm = calc_Pmm(m);
	return (double((2*m+5)*(2*m+1))*pmm/6.0 + double((2*m+2)*(2*m+1))*pmm/3.0) * -calc_K(m+3, m);
}

template<int ORDER, typename CONST_T = float> struct tsxEvalSH {
	CONST_T mConsts[ORDER < 1 ? 0 : ORDER*ORDER - (ORDER-1)];

	tsxEvalSH() {
		int l, m;
		int idx = 0;
		mConsts[idx++] = (CONST_T)calc_K(0, 0);
		if (ORDER > 1) {
			// 1, 0
			mConsts[idx++] = (CONST_T)(calc_Pmm(0) * calc_K(1, 0));
		}
		if (ORDER > 2) {
			// 2, 0
			mConsts[idx++] = (CONST_T)calc_constD1(0);
			mConsts[idx++] = (CONST_T)calc_constD2(0);
		}
		if (ORDER > 3) {
			// 2, 0
			mConsts[idx++] = (CONST_T)calc_constE1(0);
			mConsts[idx++] = (CONST_T)calc_constE2(0);
		}
		for (l = 4; l < ORDER; ++l) {
			mConsts[idx++] = (CONST_T)calc_constC1(l, 0);
			mConsts[idx++] = (CONST_T)calc_constC2(l, 0);
		}
		const double scl = ::sqrt(2.0);
		for (m = 1; m < ORDER - 1; ++m) {
			mConsts[idx++] = (CONST_T)calc_constA(m);
			if (m+1 < ORDER) {
				++l;
				mConsts[idx++] = (CONST_T)(calc_constB(m) * scl);
			}
			if (m+2 < ORDER) {
				++l;
				mConsts[idx++] = (CONST_T)(calc_constD1(m) * scl);
				mConsts[idx++] = (CONST_T)(calc_constD2(m) * scl);
			}
			if (m+3 < ORDER) {
				++l;
				mConsts[idx++] = (CONST_T)(calc_constE1(m) * scl);
				mConsts[idx++] = (CONST_T)(calc_constE2(m) * scl);
			}
			for (l = m+4; l < ORDER; ++l) {
				mConsts[idx++] = (CONST_T)calc_constC1(l, m);
				mConsts[idx++] = (CONST_T)calc_constC2(l, m);
			}
		}
		if (ORDER > 1) {
			mConsts[idx++] = (CONST_T)calc_constA(ORDER-1);
		}
	}

	template<typename EVAL_T> void eval(EVAL_T* pCoef, EVAL_T x, EVAL_T y, EVAL_T z) {
		int l = 0;
		int m = 0;
		int idx = 0;
		EVAL_T tmp = (EVAL_T)0;
		EVAL_T zz = z*z;

		pCoef[0] = (EVAL_T)mConsts[idx++];
		if (ORDER > 1) {
			pCoef[calc_ary_idx(1, 0)] = (EVAL_T)mConsts[idx++] * z;
		}
		if (ORDER > 2) {
			tmp = (EVAL_T)mConsts[idx++] * zz;
			tmp += (EVAL_T)mConsts[idx++];
			pCoef[calc_ary_idx(2, 0)] = tmp;
		}
		if (ORDER > 3) {
			tmp = (EVAL_T)mConsts[idx++] * zz;
			tmp += (EVAL_T)mConsts[idx++];
			pCoef[calc_ary_idx(3, 0)] = tmp * z;
		}
		for (l = 4; l < ORDER; ++l) {
			tmp = (EVAL_T)mConsts[idx++] * z * pCoef[calc_ary_idx(l-1, 0)];
			tmp += (EVAL_T)mConsts[idx++] * pCoef[calc_ary_idx(l-2, 0)];
			pCoef[calc_ary_idx(l, 0)] = tmp;
		}
		EVAL_T prev[3] = { (EVAL_T)0 };
		EVAL_T s[2];
		s[0] = y;
		EVAL_T c[2];
		c[0] = x;
		int scIdx = 0;
		for (m = 1; m < ORDER-1; ++m) {
			l = m;
			tmp = (EVAL_T)mConsts[idx++];
			pCoef[l*l + l - m] = tmp * s[scIdx];
			pCoef[l*l + l + m] = tmp * c[scIdx];

			if (m+1 < ORDER) {
				// (m+1, -m), (m+1, m)
				l = m+1;
				prev[1] = (EVAL_T)mConsts[idx++] * z;
				pCoef[l*l + l - m] = prev[1] * s[scIdx];
				pCoef[l*l + l + m] = prev[1] * c[scIdx];
			}
			if (m+2 < ORDER) {
				// (m+2, -m), (m+2, m)
				l = m+2;
				tmp = (EVAL_T)mConsts[idx++] * zz;
				tmp += (EVAL_T)mConsts[idx++];
				prev[2] = tmp;
				pCoef[l*l + l - m] = prev[2] * s[scIdx];
				pCoef[l*l + l + m] = prev[2] * c[scIdx];
			}
			if (m+3 < ORDER) {
				// (m+3, -m), (m+3, m)
				l = m+3;
				tmp = (EVAL_T)mConsts[idx++] * zz;
				tmp += (EVAL_T)mConsts[idx++];
				prev[0] = tmp * z;
				pCoef[l*l + l - m] = prev[0] * s[scIdx];
				pCoef[l*l + l + m] = prev[0] * c[scIdx];
			}
			const unsigned prevMask = 1 | (0 << 2) | (2 << 4) | (1 << 6) | (0 << 8) | (2 << 10) | (1 << 12);
			unsigned mask = prevMask | ((prevMask >> 2) << 14);
			unsigned maskCnt = 0;
			for (l = m+4; l < ORDER; ++l) {
				unsigned prevIdx = mask & 3;
				tmp = (EVAL_T)mConsts[idx++] * z;
				tmp *= prev[(mask >> 2) & 3];
				prev[prevIdx] = tmp;
				tmp = (EVAL_T)mConsts[idx++];
				tmp *= prev[(mask >> 4) & 3];
				prev[prevIdx] += tmp;
				pCoef[l*l + l - m] = prev[prevIdx] * s[scIdx];
				pCoef[l*l + l + m] = prev[prevIdx] * c[scIdx];
				if (ORDER < 11) {
					mask >>= 4;
				} else {
					++maskCnt;
					if (maskCnt < 3) {
						mask >>= 4;
					} else {
						mask = prevMask;
						maskCnt = 0;
					}
				}
			}

			s[scIdx^1] = x*s[scIdx] + y*c[scIdx];
			c[scIdx^1] = x*c[scIdx] - y*s[scIdx];
			scIdx ^= 1;
		}

		if (ORDER > 1) {
			tmp = (EVAL_T)mConsts[idx++];
			pCoef[calc_ary_idx(ORDER-1, -(ORDER-1))] = tmp * s[scIdx];
			pCoef[calc_ary_idx(ORDER-1, (ORDER-1))] = tmp * c[scIdx];
		}
	}
};

static tsxEvalSH<2> s_SH2f;
static tsxEvalSH<3> s_SH3f;
static tsxEvalSH<4> s_SH4f;
static tsxEvalSH<5> s_SH5f;
static tsxEvalSH<6> s_SH6f;
static tsxEvalSH<7> s_SH7f;
static tsxEvalSH<8> s_SH8f;
static tsxEvalSH<9> s_SH9f;
static tsxEvalSH<10> s_SH10f;
static tsxEvalSH<11> s_SH11f;
static tsxEvalSH<12> s_SH12f;
static tsxEvalSH<13> s_SH13f;
static tsxEvalSH<14> s_SH14f;
static tsxEvalSH<15> s_SH15f;
static tsxEvalSH<16> s_SH16f;
static tsxEvalSH<17> s_SH17f;
static tsxEvalSH<18> s_SH18f;
static tsxEvalSH<19> s_SH19f;
static tsxEvalSH<20> s_SH20f;
static tsxEvalSH<30> s_SH30f;
static tsxEvalSH<40> s_SH40f;
static tsxEvalSH<50> s_SH50f;
static tsxEvalSH<60> s_SH60f;
static tsxEvalSH<70> s_SH70f;
static tsxEvalSH<80> s_SH80f;


#if XD_SHEVAL_F64
static tsxEvalSH<2, double> s_SH2d;
static tsxEvalSH<3, double> s_SH3d;
static tsxEvalSH<4, double> s_SH4d;
static tsxEvalSH<5, double> s_SH5d;
static tsxEvalSH<6, double> s_SH6d;
static tsxEvalSH<7, double> s_SH7d;
static tsxEvalSH<8, double> s_SH8d;
static tsxEvalSH<9, double> s_SH9d;
static tsxEvalSH<10, double> s_SH10d;
#endif

#undef _XD_SHEVAL_FUNC
#define _XD_SHEVAL_FUNC(_ord_) void eval##_ord_(float* pCoef, float x, float y, float z) { s_SH##_ord_##f.eval(pCoef, x, y, z); }
_XD_SHEVAL_FUNC(2)
_XD_SHEVAL_FUNC(3)
_XD_SHEVAL_FUNC(4)
_XD_SHEVAL_FUNC(5)
_XD_SHEVAL_FUNC(6)
_XD_SHEVAL_FUNC(7)
_XD_SHEVAL_FUNC(8)
_XD_SHEVAL_FUNC(9)
_XD_SHEVAL_FUNC(10)
_XD_SHEVAL_FUNC(11)
_XD_SHEVAL_FUNC(12)
_XD_SHEVAL_FUNC(13)
_XD_SHEVAL_FUNC(14)
_XD_SHEVAL_FUNC(15)
_XD_SHEVAL_FUNC(16)
_XD_SHEVAL_FUNC(17)
_XD_SHEVAL_FUNC(18)
_XD_SHEVAL_FUNC(19)
_XD_SHEVAL_FUNC(20)
_XD_SHEVAL_FUNC(30)
_XD_SHEVAL_FUNC(40)
_XD_SHEVAL_FUNC(50)
_XD_SHEVAL_FUNC(60)
_XD_SHEVAL_FUNC(70)
_XD_SHEVAL_FUNC(80)


#define _XD_SHEVAL_CASE(_ord_) case _ord_: eval##_ord_(pCoef, x, y, z); break;

void eval(int order, float* pCoef, float x, float y, float z) {
	switch (order) {
		_XD_SHEVAL_CASE(2)
		_XD_SHEVAL_CASE(3)
		_XD_SHEVAL_CASE(4)
		_XD_SHEVAL_CASE(5)
		_XD_SHEVAL_CASE(6)
		_XD_SHEVAL_CASE(7)
		_XD_SHEVAL_CASE(8)
		_XD_SHEVAL_CASE(9)
		_XD_SHEVAL_CASE(10)
		_XD_SHEVAL_CASE(11)
		_XD_SHEVAL_CASE(12)
		_XD_SHEVAL_CASE(13)
		_XD_SHEVAL_CASE(14)
		_XD_SHEVAL_CASE(15)
		_XD_SHEVAL_CASE(16)
		_XD_SHEVAL_CASE(17)
		_XD_SHEVAL_CASE(18)
		_XD_SHEVAL_CASE(19)
		_XD_SHEVAL_CASE(20)
		_XD_SHEVAL_CASE(30)
		_XD_SHEVAL_CASE(40)
		_XD_SHEVAL_CASE(50)
		_XD_SHEVAL_CASE(60)
		_XD_SHEVAL_CASE(70)
		_XD_SHEVAL_CASE(80)
	}
}

#undef _XD_SHEVAL_CASE
#define _XD_SHEVAL_CASE(_ord_) case _ord_: pConsts = s_SH##_ord_##f.mConsts; break;

float* get_consts_f32(int order) {
	float* pConsts = nullptr;
	switch (order) {
		_XD_SHEVAL_CASE(2)
		_XD_SHEVAL_CASE(3)
		_XD_SHEVAL_CASE(4)
		_XD_SHEVAL_CASE(5)
		_XD_SHEVAL_CASE(6)
		_XD_SHEVAL_CASE(7)
		_XD_SHEVAL_CASE(8)
		_XD_SHEVAL_CASE(9)
		_XD_SHEVAL_CASE(10)
		_XD_SHEVAL_CASE(11)
		_XD_SHEVAL_CASE(12)
		_XD_SHEVAL_CASE(13)
		_XD_SHEVAL_CASE(14)
		_XD_SHEVAL_CASE(15)
		_XD_SHEVAL_CASE(16)
		_XD_SHEVAL_CASE(17)
		_XD_SHEVAL_CASE(18)
		_XD_SHEVAL_CASE(19)
		_XD_SHEVAL_CASE(20)
		_XD_SHEVAL_CASE(30)
		_XD_SHEVAL_CASE(40)
		_XD_SHEVAL_CASE(50)
		_XD_SHEVAL_CASE(60)
		_XD_SHEVAL_CASE(70)
		_XD_SHEVAL_CASE(80)
	}
	return pConsts;
}

#if XD_SHEVAL_F64
#undef _XD_SHEVAL_FUNC
#define _XD_SHEVAL_FUNC(_ord_) void eval##_ord_(double* pCoef, double x, double y, double z) { s_SH##_ord_##d.eval(pCoef, x, y, z); }
_XD_SHEVAL_FUNC(2)
_XD_SHEVAL_FUNC(3)
_XD_SHEVAL_FUNC(4)
_XD_SHEVAL_FUNC(5)
_XD_SHEVAL_FUNC(6)
_XD_SHEVAL_FUNC(7)
_XD_SHEVAL_FUNC(8)
_XD_SHEVAL_FUNC(9)
_XD_SHEVAL_FUNC(10)

void eval(int order, double* pCoef, double x, double y, double z) {
	switch (order) {
		_XD_SHEVAL_CASE(2)
		_XD_SHEVAL_CASE(3)
		_XD_SHEVAL_CASE(4)
		_XD_SHEVAL_CASE(5)
		_XD_SHEVAL_CASE(6)
		_XD_SHEVAL_CASE(7)
		_XD_SHEVAL_CASE(8)
		_XD_SHEVAL_CASE(9)
		_XD_SHEVAL_CASE(10)
	}
}
#endif

void project_polar_map(int order, float* pCoefR, float* pCoefG, float* pCoefB, cxColor* pMap, int w, int h, float* pTmp) {
	if (order < 2) return;
	const int maxOrder = 10;
	tsxSHCoefs<maxOrder> tmpSH;
	if (!pTmp) {
		if (order > maxOrder) return;
		pTmp = tmpSH.mData;
	}
	int ncoef = calc_coefs_num(order);
	size_t dataSize = ncoef * sizeof(float);
	::memset(pCoefR, 0, dataSize);
	::memset(pCoefG, 0, dataSize);
	::memset(pCoefB, 0, dataSize);
	float da = (2.0f*XD_PI / (float)w) * (XD_PI / (float)h);
	float sum = 0.0f;
	for (int y = 0; y < h; ++y) {
		float v = 1.0f - (y + 0.5f) / h;
		float dw = da * ::sinf(XD_PI*v);
		for (int x = 0; x < w; ++x) {
			int idx = y*w + x;
			cxColor clr = pMap[idx];
			float u = (x + 0.5f) / w;
			cxVec dir = nxVec::from_polar_uv(u, v);
			eval(order, pTmp, dir.x, dir.y, dir.z);
			for (int i = 0; i < ncoef; ++i) {
				float sw = pTmp[i] * dw;
				pCoefR[i] += clr.r * sw;
				pCoefG[i] += clr.g * sw;
				pCoefB[i] += clr.b * sw;
			}
			sum += dw;
		}
	}
	float s = XD_PI*4.0f / sum;
	for (int i = 0; i < ncoef; ++i) {
		pCoefR[i] *= s;
		pCoefG[i] *= s;
		pCoefB[i] *= s;
	}
}

void get_irrad_weights(float* pWgt, int order, float scl) {
	static float w[] = { 1.0f, 2.094395f / XD_PI, 0.785398f / XD_PI };
	if (!pWgt) return;
	for (int i = 0; i < order; ++i) {
		if (i < 3) {
			pWgt[i] = w[i] * scl;
		} else {
			pWgt[i] = 0.0f;
		}
	}
}

void calc_weights(float* pWgt, int order, float s, float scl) {
	if (!pWgt) return;
	for (int i = 0; i < order; ++i) {
		pWgt[i] = ::expf(float(-i*i) / (2.0f*s)) * scl;
	}
}

void apply_weights(float* pDst, int order, const float* pSrc, const float* pWgt) {
	for (int l = 0; l < order; ++l) {
		int i0 = calc_coefs_num(l);
		int i1 = calc_coefs_num(l+1);
		float w = pWgt[l];
		for (int i = i0; i < i1; ++i) {
			pDst[i] = pSrc[i] * w;
		}
	}
}

float dot(int order, const float* pA, const float* pB) {
	float d = 0.0f;
	int n = calc_coefs_num(order);
	for (int i = 0; i < n; ++i) {
		d += pA[i] * pB[i];
	}
	return d;
}

cxVec extract_dominant_dir(const float* pCoefR, const float* pCoefG, const float* pCoefB) {
	int idx = calc_ary_idx(1, 1);
	float lx = cxColor(pCoefR[idx], pCoefG[idx], pCoefB[idx]).luminance();
	idx = calc_ary_idx(1, -1);
	float ly = cxColor(pCoefR[idx], pCoefG[idx], pCoefB[idx]).luminance();
	idx = calc_ary_idx(1, 0);
	float lz = cxColor(pCoefR[idx], pCoefG[idx], pCoefB[idx]).luminance();
	cxVec dir(-lx, -ly, lz);
	dir.normalize();
	return dir;
}

} // nxSH


