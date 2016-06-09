#include "xcore.hpp"
#include "xcalc.hpp"
#include "xdata.hpp"

static cxVec eval_pol_color(const sxGeometryData::Polygon& pol, const cxVec& pos, const sxGeometryData::QuadInfo& quadInfo) {
	int i;
	cxVec vtx[3];
	cxVec clr[3];
	int pntIdx[3];
	if (pol.is_tri()) {
		for (i = 0; i < 3; ++i) {
			pntIdx[i] = pol.get_vtx_pnt_id(i);
		}
	} else if (pol.is_quad()) {
		for (i = 0; i < 3; ++i) {
			pntIdx[i] = pol.get_vtx_pnt_id(quadInfo.get_hit_tri_vtx(i));
		}
	} else {
		return false;
	}
	for (i = 0; i < 3; ++i) {
		vtx[i] = pol.get_geo()->get_pnt(pntIdx[i]);
		clr[i] = pol.get_geo()->get_pnt_color(pntIdx[i]).mRGBA;
	}
	cxVec uvw = nxGeom::barycentric(pos, vtx[0], vtx[1], vtx[2]);
	float u = uvw.x;
	float v = uvw.y;
	u = nxCalc::saturate(u);
	v = nxCalc::saturate(v);
	cxVec cd1 = clr[1] - clr[0];
	cxVec cd2 = clr[2] - clr[0];
	cd1.scl(u);
	cd2.scl(v);
	cxVec cres = cd1 + cd2 + clr[0];
	return cres;
}

class cEnvHitFn : public sxGeometryData::HitFunc {
protected:
	bool mHitFlg;
	cxLineSeg mRay;
	float mNearest;
	const sxGeometryData* mpGeo;
	int mPolId;
	cxVec mHitPos;
	cxVec mHitNrm;
	sxGeometryData::QuadInfo mQuadInfo;

public:
	cEnvHitFn() : mHitFlg(false), mNearest(FLT_MAX), mpGeo(nullptr) {}

	void set_ray(const cxLineSeg& ray) { mRay = ray; }

	bool get_hit_flg() const { return mHitFlg; }

	cxColor get_color() const {
		cxColor clr(0.0f);
		if (mHitFlg && mpGeo && mpGeo->ck_pol_idx(mPolId)) {
			sxGeometryData::Polygon pol = mpGeo->get_pol(mPolId);
			cxVec c = eval_pol_color(pol, mHitPos, mQuadInfo);
			clr.set(c.x, c.y, c.z);
		}
		return clr;
	}

	virtual bool operator()(const sxGeometryData::Polygon& pol, const cxVec& hitPos, const cxVec& hitNrm, const sxGeometryData::QuadInfo& quadInfo) {
		float dist = nxVec::dist(mRay.get_pos0(), hitPos);
		if (dist < mNearest) {
			mpGeo = pol.get_geo();
			mPolId = pol.get_id();
			mHitPos = hitPos;
			mHitNrm = hitNrm;
			mQuadInfo = quadInfo;
			mNearest = dist;
			mHitFlg = true;
		}
		return true;
	}
};

cxColor sample_env_color(sxGeometryData* pEnvGeo, float u, float v, float dist) {
	cxColor clr(0.0f);
	cxVec dir = nxVec::from_polar_uv(u, v);
	dir.scl(dist);
	cEnvHitFn fn;
	cxLineSeg ray;
	ray.set(cxVec(0.0f), dir);
	fn.set_ray(ray);
	pEnvGeo->hit_query(ray, fn);
	if (fn.get_hit_flg()) {
		clr = fn.get_color();
	} else {
		clr.a = 0.0f;
	}
	return clr;
}

struct sEnvCoefs {
	int mOrder;
	float* mpR;
	float* mpG;
	float* mpB;
	double mDtMs;

	sEnvCoefs(int order = 10) : mOrder(order), mDtMs(0.0), mpR(nullptr), mpG(nullptr), mpB(nullptr) {
		alloc();
	}

	~sEnvCoefs() {
		free();
	}

	void alloc() {
		free();
		int order = mOrder;
		mpR = nxDataUtil::alloc_sh_coefs_f32(order);
		mpG = nxDataUtil::alloc_sh_coefs_f32(order);
		mpB = nxDataUtil::alloc_sh_coefs_f32(order);
	}

	void free() {
		if (mpR) {
			nxCore::mem_free(mpR);
			mpR = nullptr;
		}
		if (mpG) {
			nxCore::mem_free(mpG);
			mpG = nullptr;
		}
		if (mpB) {
			nxCore::mem_free(mpB);
			mpB = nullptr;
		}
	}

	void clear() {
		int ncoef = nxSH::calc_coefs_num(mOrder);
		size_t dataSize = ncoef * sizeof(float);
		::memset(mpR, 0, dataSize);
		::memset(mpG, 0, dataSize);
		::memset(mpB, 0, dataSize);
	}

	void proj_env_polar(int w, int h, cxColor* pMap);
};

XD_NOINLINE void sEnvCoefs::proj_env_polar(int w, int h, cxColor* pMap) {
	float* pTmp = nxDataUtil::alloc_sh_coefs_f32(mOrder);
	int64_t ft0 = nxCore::get_timestamp();
	nxSH::project_polar_map(mOrder, mpR, mpG, mpB, pMap, w, h, pTmp);
	int64_t ft1 = nxCore::get_timestamp();
	double freq = nxCore::get_perf_freq();
	double dt = (ft1 - ft0) / freq;
	mDtMs = dt * 1000.0f;
	nxCore::mem_free(pTmp);
}

static cxColor* sh_env_apply(const sxGeometryData& tgtGeo, const sEnvCoefs& envSH) {
	int npnt = tgtGeo.get_pnt_num();
	cxColor* pPntClr = nxCore::obj_alloc<cxColor>(npnt);
	int order = envSH.mOrder;
	float* pWgt = nxCore::obj_alloc<float>(order);
	float* pN = nxDataUtil::alloc_sh_coefs_f32(order);
	float* pR = nxDataUtil::alloc_sh_coefs_f32(order);
	float* pG = nxDataUtil::alloc_sh_coefs_f32(order);
	float* pB = nxDataUtil::alloc_sh_coefs_f32(order);
	if (pPntClr && pWgt && pN && pR && pG && pB) {
		::printf("Applying SH to target obj (%d vertices).\n", npnt);
		float s = 6.0f;
		nxSH::calc_weights(pWgt, order, s);
		nxSH::apply_weights(pR, order, envSH.mpR, pWgt);
		nxSH::apply_weights(pG, order, envSH.mpG, pWgt);
		nxSH::apply_weights(pB, order, envSH.mpB, pWgt);
		int64_t ft0 = nxCore::get_timestamp();
		for (int i = 0; i < npnt; ++i) {
			cxVec nrm = tgtGeo.get_pnt_normal(i);
			nxSH::eval(order, pN, nrm.x, nrm.y, nrm.z);
			float r = nxSH::dot(order, pN, pR);
			float g = nxSH::dot(order, pN, pG);
			float b = nxSH::dot(order, pN, pB);
			pPntClr[i].set(r, g, b);
		}
		int64_t ft1 = nxCore::get_timestamp();
		double freq = nxCore::get_perf_freq();
		double dt = (ft1 - ft0) / freq;
		double dtms = dt * 1000.0f;
		::printf("Elapsed %f ms.\n", dtms);

		nxCore::obj_free<float>(pWgt, order);
		nxCore::mem_free(pN);
		nxCore::mem_free(pR);
		nxCore::mem_free(pG);
		nxCore::mem_free(pB);
	}
	return pPntClr;
}

void sh_perf_test() {
	const char* envPath = "../data/env.xgeo";
	sxData* pEnvData = nxData::load(envPath);
	if (!pEnvData) return;
	bool isGeo = pEnvData->is<sxGeometryData>();
	if (!isGeo) {
		nxData::unload(pEnvData);
		return;
	}
	sxGeometryData* pEnvGeo = pEnvData->as<sxGeometryData>();

	int mapW = 256;
	int mapH = 128;
	uint32_t ddsSize = 0;
	sxDDSHead* pPolarDDS = nxTexture::alloc_dds128(mapW, mapH, &ddsSize);
	cxColor* pMap = reinterpret_cast<cxColor*>(pPolarDDS + 1);
	cxAABB envBB = pEnvGeo->mBBox;
	float envRad = envBB.get_bounding_radius();
	int64_t ft0 = nxCore::get_timestamp();
	for (int y = 0; y < mapH; ++y) {
		float v = 1.0f - (y + 0.5f) / mapH;
		for (int x = 0; x < mapW; ++x) {
			float u = (x + 0.5f) / mapW;
			int idx = y*mapW + x;
			cxColor clr = sample_env_color(pEnvGeo, u, v, envRad);
			pMap[idx] = clr;
		}
	}
	int64_t ft1 = nxCore::get_timestamp();
	double freq = nxCore::get_perf_freq();
	double dt = (ft1 - ft0) / freq;
	double dtms = dt * 1000.0f;
	::printf("Envmap creation took %f sec. (%f ms)\n", dt, dtms);

	const char* pEnvMapOutPath = "../data/env_map.dds";
	::printf("Saving envmap to \"%s\"\n", pEnvMapOutPath);
	nxCore::bin_save(pEnvMapOutPath, pPolarDDS, ddsSize);

	nxData::unload(pEnvData);

	int order = ::atoi("10");
	sEnvCoefs envSH(order);
	::printf("Testing order %d projection @ %dx%d.\n", order, mapW, mapH);
	envSH.proj_env_polar(mapW, mapH, pMap);
	::printf("Elapsed %f ms.\n", envSH.mDtMs);

	const char* tgtPath = "../data/tgt.xgeo";
	sxData* pTgtData = nxData::load(tgtPath);
	if (!pTgtData) return;
	isGeo = pTgtData->is<sxGeometryData>();
	if (!isGeo) {
		nxData::unload(pTgtData);
		nxCore::mem_free(pPolarDDS);
		return;
	}
	sxGeometryData* pTgtGeo = pTgtData->as<sxGeometryData>();
	cxColor* pTgtClr = sh_env_apply(*pTgtGeo, envSH);
	if (pTgtClr) {
		int npnt = pTgtGeo->get_pnt_num();
		FILE* pOut = nullptr;
		const char* pOutGeoPath = "../data/_out.geo";
		::printf("Saving geometry to \"%s\"\n", pOutGeoPath);
		::fopen_s(&pOut, pOutGeoPath, "w");
		if (pOut) {
			int npol = pTgtGeo->get_pol_num();
			::fprintf(pOut, "PGEOMETRY V5\nNPoints %d NPrims %d\nNPointGroups 0 NPrimGroups 0\n", npnt, npol);
			::fprintf(pOut, "NPointAttrib 1 NVertexAttrib 0 NPrimAttrib 0 NAttrib 0\n");
			::fprintf(pOut, "PointAttrib\n");
			::fprintf(pOut, "Cd 3 float 1 1 1\n");
			for (int i = 0; i < npnt; ++i) {
				cxVec pos = pTgtGeo->get_pnt(i);
				cxColor clr = pTgtClr[i];
				clr.scl_rgb(0.15f);
				::fprintf(pOut, "%.10f %.10f %.10f 1 (%.10f %.10f %.10f)\n", pos.x, pos.y, pos.z, clr.r, clr.g, clr.b);
			}
			::fprintf(pOut, "Run %d Poly\n", npol);
			for (int i = 0; i < npol; ++i) {
				sxGeometryData::Polygon pol = pTgtGeo->get_pol(i);
				int nvtx = pol.get_vtx_num();
				::fprintf(pOut, " %d <", nvtx);
				for (int j = 0; j < nvtx; ++j) {
					::fprintf(pOut, " %d", pol.get_vtx_pnt_id(j));
				}
				::fprintf(pOut, "\n");
			}
			::fprintf(pOut, "beginExtra\nendExtra\n");
			::fclose(pOut);
		}
		nxCore::obj_free<cxColor>(pTgtClr, npnt);
	}

	nxData::unload(pTgtData);
	nxCore::mem_free(pPolarDDS);
}

int main() {
	nxCore::set_exe_cwd();

	DWORD_PTR mask;
	HANDLE hThr = ::GetCurrentThread();
	mask = ::SetThreadAffinityMask(hThr, 1);

	sh_perf_test();

	::SetThreadAffinityMask(hThr, mask);

	return 0;
}
