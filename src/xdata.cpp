/*
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

#include "xcore.hpp"
#include "xcalc.hpp"
#include "xdata.hpp"

namespace nxData {

XD_NOINLINE sxData* load(const char* pPath) {
	size_t size = 0;
	sxData* pData = reinterpret_cast<sxData*>(nxCore::bin_load(pPath, &size, true));
	if (pData) {
		if (pData->mFileSize == size) {
			pData->mFilePathLen = (uint32_t)::strlen(pPath);
		} else {
			unload(pData);
			pData = nullptr;
		}
	}
	return pData;
}

XD_NOINLINE void unload(sxData* pData) {
	nxCore::bin_unload(pData);
}

} // nxData

namespace nxDataUtil {

exAnimChan anim_chan_from_str(const char* pStr) {
	exAnimChan res = exAnimChan::UNKNOWN;
	static struct {
		const char* pName;
		exAnimChan id;
	} tbl[] = {
		{ "tx", exAnimChan::TX },
		{ "ty", exAnimChan::TY },
		{ "tz", exAnimChan::TZ },
		{ "rx", exAnimChan::RX },
		{ "ry", exAnimChan::RY },
		{ "rz", exAnimChan::RZ },
		{ "sx", exAnimChan::SX },
		{ "sy", exAnimChan::SY },
		{ "sz", exAnimChan::SZ },
	};
	if (pStr) {
		for (int i = 0; i < XD_ARY_LEN(tbl); ++i) {
			if (nxCore::str_eq(pStr, tbl[i].pName)) {
				res = tbl[i].id;
				break;
			}
		}
	}
	return res;

}
const char* anim_chan_to_str(exAnimChan chan) {
	const char* pName = "<unknown>";
	switch (chan) {
		case exAnimChan::TX: pName = "tx"; break;
		case exAnimChan::TY: pName = "ty"; break;
		case exAnimChan::TZ: pName = "tz"; break;
		case exAnimChan::RX: pName = "rx"; break;
		case exAnimChan::RY: pName = "ry"; break;
		case exAnimChan::RZ: pName = "rz"; break;
		case exAnimChan::SX: pName = "sx"; break;
		case exAnimChan::SY: pName = "sy"; break;
		case exAnimChan::SZ: pName = "sz"; break;
		default: break;
	}
	return pName;
}

exRotOrd rot_ord_from_str(const char* pStr) {
	exRotOrd rord = exRotOrd::XYZ;
	static struct {
		const char* pName;
		exRotOrd ord;
	} tbl[] = {
		{ "xyz", exRotOrd::XYZ },
		{ "xzy", exRotOrd::XZY },
		{ "yxz", exRotOrd::YXZ },
		{ "yzx", exRotOrd::YZX },
		{ "zxy", exRotOrd::ZXY },
		{ "zyx", exRotOrd::ZYX }
	};
	if (pStr) {
		for (int i = 0; i < XD_ARY_LEN(tbl); ++i) {
			if (nxCore::str_eq(pStr, tbl[i].pName)) {
				rord = tbl[i].ord;
				break;
			}
		}
	}
	return rord;
}

const char* rot_ord_to_str(exRotOrd rord) {
	const char* pStr = "xyz";
	switch (rord) {
		case exRotOrd::XYZ: pStr = "xyz"; break;
		case exRotOrd::XZY: pStr = "xzy"; break;
		case exRotOrd::YXZ: pStr = "yxz"; break;
		case exRotOrd::YZX: pStr = "yzx"; break;
		case exRotOrd::ZXY: pStr = "zxy"; break;
		case exRotOrd::ZYX: pStr = "zyx"; break;
		default: break;
	}
	return pStr;
}

exTransformOrd xform_ord_from_str(const char* pStr) {
	exTransformOrd xord = exTransformOrd::SRT;
	static struct {
		const char* pName;
		exTransformOrd ord;
	} tbl[] = {
		{ "srt", exTransformOrd::SRT },
		{ "str", exTransformOrd::STR },
		{ "rst", exTransformOrd::RST },
		{ "rts", exTransformOrd::RTS },
		{ "tsr", exTransformOrd::TSR },
		{ "trs", exTransformOrd::TRS }
	};
	if (pStr) {
		for (int i = 0; i < XD_ARY_LEN(tbl); ++i) {
			if (nxCore::str_eq(pStr, tbl[i].pName)) {
				xord = tbl[i].ord;
				break;
			}
		}
	}
	return xord;
}

const char* xform_ord_to_str(exTransformOrd xord) {
	const char* pStr = "srt";
	switch (xord) {
		case exTransformOrd::SRT: pStr = "srt"; break;
		case exTransformOrd::STR: pStr = "str"; break;
		case exTransformOrd::RST: pStr = "rst"; break;
		case exTransformOrd::RTS: pStr = "rts"; break;
		case exTransformOrd::TSR: pStr = "tsr"; break;
		case exTransformOrd::TRS: pStr = "trs"; break;
		default: break;
	}
	return pStr;
}

float* alloc_sh_coefs_f32(int order) {
	float* pSH = nullptr;
	int n = nxSH::calc_coefs_num(order);
	if (n) {
		pSH = reinterpret_cast<float*>(nxCore::mem_alloc(n * sizeof(float)));
	}
	return pSH;
}

double* alloc_sh_coefs_f64(int order) {
	double* pSH = nullptr;
	int n = nxSH::calc_coefs_num(order);
	if (n) {
		pSH = reinterpret_cast<double*>(nxCore::mem_alloc(n * sizeof(double)));
	}
	return pSH;
}

} // nxDataUtil

XD_NOINLINE int sxStrList::find_str(const char* pStr) const {
	if (!pStr) return -1;
	uint32_t i;
	uint32_t n = mNum;
	uint32_t nmain = n >> 3;
	uint16_t h = nxCore::str_hash16(pStr);
	__m128i htst = _mm_set1_epi16(h);
	__m128i* pHash = (__m128i*)get_hash_top();
	for (i = 0; i < nmain; ++i) {
		__m128i href = _mm_loadu_si128(pHash);
		__m128i c = _mm_cmpeq_epi16(href, htst);
		c = _mm_packs_epi16(c, c);
		uint32_t msk = _mm_movemask_epi8(c) & 0xFF;
		while (msk) {
			uint32_t bit = nxCore::bit_scan_fwd(msk);
			int idx = bit + (i << 3);
			if (0 == ::strcmp(get_str(idx), pStr)) return idx;
			msk &= ~(1 << bit);
		}
		++pHash;
	}
	uint32_t ntail = n & 7;
	if (ntail) {
		__m128i href = _mm_loadu_si128(pHash);
		__m128i c = _mm_cmpeq_epi16(href, htst);
		c = _mm_packs_epi16(c, c);
		uint32_t msk = _mm_movemask_epi8(c);
		msk &= 0xFFU >> (8 - ntail);
		while (msk) {
			uint32_t bit = nxCore::bit_scan_fwd(msk);
			int idx = bit + (i << 3);
			if (0 == ::strcmp(get_str(idx), pStr)) return idx;
			msk &= ~(1 << bit);
		}
	}
	return -1;
}


int sxVecList::get_elems(float* pDst, int idx, int num) const {
	int n = 0;
	if (ck_idx(idx)) {
		const float* pVal = get_ptr(idx);
		const float* pLim = reinterpret_cast<const float*>(XD_INCR_PTR(this, mSize));
		for (int i = 0; i < num; ++i) {
			if (pVal >= pLim) break;
			if (pDst) {
				pDst[i] = *pVal;
			}
			++pVal;
			++n;
		}
	}
	return n;
}

xt_float2 sxVecList::get_f2(int idx) {
	xt_float2 v;
	v.fill(0.0f);
	get_elems(v, idx, 2);
	return v;
}

xt_float3 sxVecList::get_f3(int idx) {
	xt_float3 v;
	v.fill(0.0f);
	get_elems(v, idx, 3);
	return v;
}

xt_float4 sxVecList::get_f4(int idx) {
	xt_float4 v;
	v.fill(0.0f);
	get_elems(v, idx, 4);
	return v;
}


int sxValuesData::Group::find_val_idx(const char* pName) const {
	int idx = -1;
	if (pName && is_valid()) {
		sxStrList* pStrLst = mpVals->get_str_list();
		if (pStrLst) {
			int nameId = pStrLst->find_str(pName);
			if (nameId >= 0) {
				const GrpInfo* pInfo = get_info();
				const ValInfo* pVal = pInfo->mVals;
				int nval = pInfo->mValNum;
				for (int i = 0; i < nval; ++i) {
					if (pVal[i].mNameId == nameId) {
						idx = i;
						break;
					}
				}
			}
		}
	}
	return idx;
}

int sxValuesData::Group::get_val_i(int idx) const {
	int res = 0;
	const ValInfo* pVal = get_val_info(idx);
	if (pVal) {
		float ftmp[1];
		eValType typ = pVal->get_type();
		switch (typ) {
			case eValType::INT:
				res = pVal->mValId.i;
				break;
			case eValType::FLOAT:
				res = (int)pVal->mValId.f;
				break;
			case eValType::VEC2:
			case eValType::VEC3:
			case eValType::VEC4:
				mpVals->get_vec_list()->get_elems(ftmp, pVal->mValId.i, 1);
				res = (int)ftmp[0];
				break;
		}
	}
	return res;
}

float sxValuesData::Group::get_val_f(int idx) const {
	float res = 0.0f;
	const ValInfo* pVal = get_val_info(idx);
	if (pVal) {
		eValType typ = pVal->get_type();
		switch (typ) {
			case eValType::INT:
				res = (float)pVal->mValId.i;
				break;
			case eValType::FLOAT:
				res = pVal->mValId.f;
				break;
			case eValType::VEC2:
			case eValType::VEC3:
			case eValType::VEC4:
				mpVals->get_vec_list()->get_elems(&res, pVal->mValId.i, 1);
				break;
		}
	}
	return res;
}

xt_float2 sxValuesData::Group::get_val_f2(int idx) const {
	xt_float2 res;
	res.fill(0.0f);
	const ValInfo* pVal = get_val_info(idx);
	if (pVal) {
		eValType typ = pVal->get_type();
		switch (typ) {
		case eValType::INT:
			res.fill((float)pVal->mValId.i);
			break;
		case eValType::FLOAT:
			res.fill(pVal->mValId.f);
			break;
		case eValType::VEC2:
		case eValType::VEC3:
		case eValType::VEC4:
			res = mpVals->get_vec_list()->get_f2(pVal->mValId.i);
			break;
		}
	}
	return res;
}

xt_float3 sxValuesData::Group::get_val_f3(int idx) const {
	xt_float3 res;
	res.fill(0.0f);
	const ValInfo* pVal = get_val_info(idx);
	if (pVal) {
		eValType typ = pVal->get_type();
		switch (typ) {
		case eValType::INT:
			res.fill((float)pVal->mValId.i);
			break;
		case eValType::FLOAT:
			res.fill(pVal->mValId.f);
			break;
		case eValType::VEC2:
		case eValType::VEC3:
		case eValType::VEC4:
			res = mpVals->get_vec_list()->get_f3(pVal->mValId.i);
			break;
		}
	}
	return res;
}

xt_float4 sxValuesData::Group::get_val_f4(int idx) const {
	xt_float4 res;
	res.fill(0.0f);
	const ValInfo* pVal = get_val_info(idx);
	if (pVal) {
		eValType typ = pVal->get_type();
		switch (typ) {
		case eValType::INT:
			res.fill((float)pVal->mValId.i);
			break;
		case eValType::FLOAT:
			res.fill(pVal->mValId.f);
			break;
		case eValType::VEC2:
		case eValType::VEC3:
		case eValType::VEC4:
			res = mpVals->get_vec_list()->get_f4(pVal->mValId.i);
			break;
		}
	}
	return res;
}

const char* sxValuesData::Group::get_val_s(int idx) const {
	const char* pStr = nullptr;
	const ValInfo* pVal = get_val_info(idx);
	if (pVal) {
		sxStrList* pStrLst = mpVals->get_str_list();
		if (pStrLst) {
			if (pVal->get_type() == eValType::STRING) {
				pStr = pStrLst->get_str(pVal->mValId.i);
			}
		}
	}
	return pStr;
}

float sxValuesData::Group::get_float(const char* pName, const float defVal) const {
	float f = defVal;
	int idx = find_val_idx(pName);
	if (idx >= 0) {
		f = get_val_f(idx);
	}
	return f;
}

int sxValuesData::Group::get_int(const char* pName, const int defVal) const {
	int i = defVal;
	int idx = find_val_idx(pName);
	if (idx >= 0) {
		i = get_val_i(idx);
	}
	return i;
}

cxVec sxValuesData::Group::get_vec(const char* pName, const cxVec& defVal) const {
	cxVec v = defVal;
	int idx = find_val_idx(pName);
	if (idx >= 0) {
		xt_float3 f3 = get_val_f3(idx);
		v.set(f3.x, f3.y, f3.z);
	}
	return v;
}

cxColor sxValuesData::Group::get_rgb(const char* pName, const cxColor& defVal) const {
	cxColor c = defVal;
	int idx = find_val_idx(pName);
	if (idx >= 0) {
		xt_float3 f3 = get_val_f3(idx);
		c.set(f3.x, f3.y, f3.z);
	}
	return c;
}

const char* sxValuesData::Group::get_str(const char* pName, const char* pDefVal) const {
	const char* pStr = pDefVal;
	int idx = find_val_idx(pName);
	if (idx >= 0) {
		pStr = get_val_s(idx);
	}
	return pStr;
}


int sxValuesData::find_grp_idx(const char* pName, const char* pPath, int startIdx) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	if (ck_grp_idx(startIdx) && pStrLst) {
		int ngrp = get_grp_num();
		int nameId = pName ? pStrLst->find_str(pName) : -1;
		if (nameId >= 0) {
			if (pPath) {
				int pathId = pStrLst->find_str(pPath);
				if (pathId >= 0) {
					for (int i = startIdx; i < ngrp; ++i) {
						GrpInfo* pGrp = get_grp_info(i);
						if (nameId == pGrp->mNameId && pathId == pGrp->mPathId) {
							idx = i;
							break;
						}
					}
				}
			} else {
				for (int i = startIdx; i < ngrp; ++i) {
					GrpInfo* pGrp = get_grp_info(i);
					if (nameId == pGrp->mNameId) {
						idx = i;
						break;
					}
				}
			}
		} else if (pPath) {
			int pathId = pStrLst->find_str(pPath);
			if (pathId >= 0) {
				int ngrp = get_grp_num();
				for (int i = startIdx; i < ngrp; ++i) {
					GrpInfo* pGrp = get_grp_info(i);
					if (pathId == pGrp->mPathId) {
						idx = i;
						break;
					}
				}
			}
		}
	}
	return idx;
}

sxValuesData::Group sxValuesData::get_grp(int idx) const {
	Group grp;
	if (ck_grp_idx(idx)) {
		grp.mpVals = this;
		grp.mGrpId = idx;
	} else {
		grp.mpVals = nullptr;
		grp.mGrpId = -1;
	}
	return grp;
}

/*static*/ const char* sxValuesData::get_val_type_str(eValType typ) {
	const char* pStr = "UNKNOWN";
	switch (typ) {
		case eValType::FLOAT:
			pStr = "FLOAT";
			break;
		case eValType::VEC2:
			pStr = "VEC2";
			break;
		case eValType::VEC3:
			pStr = "VEC3";
			break;
		case eValType::VEC4:
			pStr = "VEC4";
			break;
		case eValType::INT:
			pStr = "INT";
			break;
		case eValType::STRING:
			pStr = "STRING";
			break;
		default:
			break;
	}
	return pStr;
}



sxRigData::Node* sxRigData::get_node_ptr(int idx) const {
	Node* pNode = nullptr;
	Node* pTop = get_node_top();
	if (pTop) {
		if (ck_node_idx(idx)) {
			pNode = pTop + idx;
		}
	}
	return pNode;
}

const char* sxRigData::get_node_name(int idx) const {
	const char* pName = nullptr;
	if (ck_node_idx(idx)) {
		sxStrList* pStrLst = get_str_list();
		if (pStrLst) {
			pName = pStrLst->get_str(get_node_ptr(idx)->mNameId);
		}
	}
	return pName;
}

const char* sxRigData::get_node_path(int idx) const {
	const char* pPath = nullptr;
	if (ck_node_idx(idx)) {
		sxStrList* pStrLst = get_str_list();
		if (pStrLst) {
			pPath = pStrLst->get_str(get_node_ptr(idx)->mPathId);
		}
	}
	return pPath;
}

const char* sxRigData::get_node_type(int idx) const {
	const char* pType = nullptr;
	if (ck_node_idx(idx)) {
		sxStrList* pStrLst = get_str_list();
		if (pStrLst) {
			pType = pStrLst->get_str(get_node_ptr(idx)->mTypeId);
		}
	}
	return pType;
}

int sxRigData::find_node(const char* pName, const char* pPath) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	if (pStrLst) {
		int n = mNodeNum;
		int nameId = pStrLst->find_str(pName);
		if (nameId >= 0) {
			if (pPath) {
				int pathId = pStrLst->find_str(pPath);
				if (pathId >= 0) {
					for (int i = 0; i < n; ++i) {
						Node* pNode = get_node_ptr(i);
						if (pNode->mNameId == nameId && pNode->mPathId == pathId) {
							idx = i;
							break;
						}
					}
				}
			} else {
				for (int i = 0; i < n; ++i) {
					Node* pNode = get_node_ptr(i);
					if (pNode->mNameId == nameId) {
						idx = i;
						break;
					}
				}
			}
		}
	}
	return idx;
}

cxMtx sxRigData::get_wmtx(int idx) const {
	cxMtx mtx;
	cxMtx* pMtx = get_wmtx_ptr(idx);
	if (pMtx) {
		mtx = *pMtx;
	} else {
		mtx.identity();
	}
	return mtx;
}

cxMtx* sxRigData::get_wmtx_ptr(int idx) const {
	cxMtx* pMtx = nullptr;
	if (ck_node_idx(idx) && mOffsWMtx) {
		cxMtx* pTop = reinterpret_cast<cxMtx*>(XD_INCR_PTR(this, mOffsWMtx));
		pMtx = pTop + idx;
	}
	return pMtx;
}

cxMtx sxRigData::get_imtx(int idx) const {
	cxMtx mtx;
	cxMtx* pMtx = get_imtx_ptr(idx);
	if (pMtx) {
		mtx = *pMtx;
	} else {
		pMtx = get_wmtx_ptr(idx);
		if (pMtx) {
			mtx = pMtx->get_inverted();
		} else {
			mtx.identity();
		}
	}
	return mtx;
}

cxMtx* sxRigData::get_imtx_ptr(int idx) const {
	cxMtx* pMtx = nullptr;
	if (ck_node_idx(idx) && mOffsIMtx) {
		cxMtx* pTop = reinterpret_cast<cxMtx*>(XD_INCR_PTR(this, mOffsIMtx));
		pMtx = pTop + idx;
	}
	return pMtx;
}

cxMtx sxRigData::get_lmtx(int idx) const {
	cxMtx mtx;
	cxMtx* pMtx = get_lmtx_ptr(idx);
	if (pMtx) {
		mtx = *pMtx;
	} else {
		mtx = calc_lmtx(idx);
	}
	return mtx;
}

cxMtx* sxRigData::get_lmtx_ptr(int idx) const {
	cxMtx* pMtx = nullptr;
	if (ck_node_idx(idx) && mOffsLMtx) {
		cxMtx* pTop = reinterpret_cast<cxMtx*>(XD_INCR_PTR(this, mOffsLMtx));
		pMtx = pTop + idx;
	}
	return pMtx;
}

cxMtx sxRigData::calc_lmtx(int idx) const {
	Node* pNode = get_node_ptr(idx);
	if (pNode->mParentIdx < 0) {
		return get_wmtx(idx);
	}
	cxMtx m = get_wmtx(idx);
	m.mul(get_imtx(pNode->mParentIdx));
	return m;
}

cxVec sxRigData::calc_parent_offs(int idx) const {
	Node* pNode = get_node_ptr(idx);
	if (pNode->mParentIdx < 0) {
		return get_wmtx_ptr(idx)->get_translation();
	}
	return get_wmtx_ptr(idx)->get_translation() - get_wmtx_ptr(pNode->mParentIdx)->get_translation();
}

cxVec sxRigData::get_lpos(int idx) const {
	cxVec pos(0.0f);
	if (ck_node_idx(idx) && mOffsLPos) {
		cxVec* pTop = reinterpret_cast<cxVec*>(XD_INCR_PTR(this, mOffsLPos));
		pos = pTop[idx];
	}
	return pos;
}

cxVec sxRigData::get_lscl(int idx) const {
	cxVec scl(1.0f);
	if (ck_node_idx(idx) && mOffsLScl) {
		cxVec* pTop = reinterpret_cast<cxVec*>(XD_INCR_PTR(this, mOffsLScl));
		scl = pTop[idx];
	}
	return scl;
}

cxVec sxRigData::get_lrot(int idx, bool inRadians) const {
	cxVec rot(0.0f);
	if (ck_node_idx(idx) && mOffsLRot) {
		cxVec* pTop = reinterpret_cast<cxVec*>(XD_INCR_PTR(this, mOffsLRot));
		rot = pTop[idx];
		if (inRadians) {
			rot.scl(XD_DEG2RAD(1.0f));
		}
	}
	return rot;
}

cxQuat sxRigData::calc_lquat(int idx) const {
	cxQuat q;
	cxVec r = get_lrot(idx, true);
	q.set_rot(r.x, r.y, r.z, get_rot_order(idx));
	return q;
}

cxMtx sxRigData::calc_wmtx(int idx, const cxMtx* pMtxLocal, cxMtx* pParentWMtx) const {
	cxMtx mtx;
	mtx.identity();
	cxMtx parentMtx;
	parentMtx.identity();
	if (pMtxLocal && ck_node_idx(idx)) {
		Node* pNode = get_node_ptr(idx);
		mtx = pMtxLocal[pNode->mSelfIdx];
		pNode = get_node_ptr(pNode->mParentIdx);
		while (pNode && !pNode->is_hrc_top()) {
			parentMtx.mul(pMtxLocal[pNode->mSelfIdx]);
			pNode = get_node_ptr(pNode->mParentIdx);
		}
		parentMtx.mul(pMtxLocal[pNode->mSelfIdx]);
		mtx.mul(parentMtx);
	}
	if (pParentWMtx) {
		*pParentWMtx = parentMtx;
	}
	return mtx;
}

void sxRigData::dump_node_names_f(FILE* pFile) const {
	if (!pFile) return;
	int n = get_nodes_num();
	for (int i = 0; i < n; ++i) {
		const char* pName = get_node_name(i);
		::fprintf(pFile, "%s\n", pName);
	}
}

void sxRigData::dump_node_names(const char* pOutPath) const {
	char outPath[MAX_PATH];
	if (!pOutPath) {
		const char* pPath = get_file_path();
		if (!pPath) return;
		::sprintf_s(outPath, "%s.names.txt", pPath);
		pOutPath = outPath;
	}
	FILE* pOut = nullptr;
	if (::fopen_s(&pOut, pOutPath, "w") != 0) {
		return;
	}
	dump_node_names_f(pOut);
	::fclose(pOut);
}


int sxGeometryData::get_vtx_idx_size() const {
	int size = 0;
	if (mPntNum <= (1 << 8)) {
		size = 1;
	} else if (mPntNum <= (1 << 16)) {
		size = 2;
	} else {
		size = 3;
	}
	return size;
}

sxGeometryData::Polygon sxGeometryData::get_pol(int idx) const {
	Polygon pol;
	if (ck_pol_idx(idx)) {
		pol.mPolId = idx;
		pol.mpGeom = this;
	} else {
		pol.mPolId = -1;
		pol.mpGeom = nullptr;
	}
	return pol;
}

int32_t* sxGeometryData::get_skin_node_name_ids() const {
	int32_t* pIds = nullptr;
	cxSphere* pSphTop = get_skin_sph_top();
	if (pSphTop) {
		pIds = reinterpret_cast<int32_t*>(&pSphTop[mSkinNodeNum]);
	}
	return pIds;
}

const char* sxGeometryData::get_skin_node_name(int idx) const {
	const char* pName = nullptr;
	if (ck_skin_idx(idx)) {
		sxStrList* pStrLst = get_str_list();
		cxSphere* pSphTop = get_skin_sph_top();
		if (pStrLst && pSphTop) {
			int32_t* pNameIds = reinterpret_cast<int32_t*>(&pSphTop[mSkinNodeNum]);
			pName = pStrLst->get_str(pNameIds[idx]);
		}
	}
	return pName;
}

int sxGeometryData::find_skin_node(const char* pName) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	if (has_skin_nodes() && pStrLst) {
		int nameId = pStrLst->find_str(pName);
		if (nameId >= 0) {
			cxSphere* pSphTop = get_skin_sph_top();
			int32_t* pNameIds = reinterpret_cast<int32_t*>(&pSphTop[mSkinNodeNum]);
			int n = get_skin_nodes_num();
			for (int i = 0; i < n; ++i) {
				if (pNameIds[i] == nameId) {
					idx = i;
					break;
				}
			}
		}
	}
	return idx;
}

uint32_t sxGeometryData::get_attr_info_offs(eAttrClass cls) const {
	uint32_t offs = 0;
	switch (cls) {
		case eAttrClass::GLOBAL:
			offs = mGlbAttrOffs;
			break;
		case eAttrClass::POINT:
			offs = mPntAttrOffs;
			break;
		case eAttrClass::POLYGON:
			offs = mPolAttrOffs;
			break;
	}
	return offs;
}

uint32_t sxGeometryData::get_attr_info_num(eAttrClass cls) const {
	uint32_t num = 0;
	switch (cls) {
	case eAttrClass::GLOBAL:
		num = mGlbAttrNum;
		break;
	case eAttrClass::POINT:
		num = mPntAttrNum;
		break;
	case eAttrClass::POLYGON:
		num = mPolAttrNum;
		break;
	}
	return num;
}

uint32_t sxGeometryData::get_attr_item_num(eAttrClass cls) const {
	uint32_t num = 0;
	switch (cls) {
	case eAttrClass::GLOBAL:
		num = 1;
		break;
	case eAttrClass::POINT:
		num = mPntNum;
		break;
	case eAttrClass::POLYGON:
		num = mPolNum;
		break;
	}
	return num;
}

int sxGeometryData::find_attr(const char* pName, eAttrClass cls) const {
	int attrId = -1;
	uint32_t offs = get_attr_info_offs(cls);
	uint32_t num = get_attr_info_num(cls);
	if (offs && num) {
		sxStrList* pStrLst = get_str_list();
		if (pStrLst) {
			int nameId = pStrLst->find_str(pName);
			if (nameId >= 0) {
				AttrInfo* pAttr = reinterpret_cast<AttrInfo*>(XD_INCR_PTR(this, offs));
				for (int i = 0; i < (int)num; ++i) {
					if (pAttr[i].mNameId == nameId) {
						attrId = i;
						break;
					}
				}
			}
		}
	}
	return attrId;
}

sxGeometryData::AttrInfo* sxGeometryData::get_attr_info(int attrIdx, eAttrClass cls) const {
	AttrInfo* pAttr = nullptr;
	uint32_t offs = get_attr_info_offs(cls);
	uint32_t num = get_attr_info_num(cls);
	if (offs && num && (uint32_t)attrIdx < num) {
		pAttr = &reinterpret_cast<AttrInfo*>(XD_INCR_PTR(this, offs))[attrIdx];
	}
	return pAttr;
}

float* sxGeometryData::get_attr_data_f(int attrIdx, eAttrClass cls, int itemIdx, int minElem) const {
	float* pData = nullptr;
	uint32_t itemNum = get_attr_item_num(cls);
	if ((uint32_t)itemIdx < itemNum) {
		AttrInfo* pInfo = get_attr_info(attrIdx, cls);
		if (pInfo && pInfo->is_float() && pInfo->mElemNum >= minElem) {
			float* pTop = reinterpret_cast<float*>(XD_INCR_PTR(this, pInfo->mDataOffs));
			pData = &pTop[itemIdx*pInfo->mElemNum];
		}
	}
	return pData;
}

float sxGeometryData::get_pnt_attr_val_f(int attrIdx, int pntIdx) const {
	float res = 0.0f;
	float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx);
	if (pData) {
		res = *pData;
	}
	return res;
}

xt_float3 sxGeometryData::get_pnt_attr_val_f3(int attrIdx, int pntIdx) const {
	xt_float3 res;
	float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 3);
	if (pData) {
		res.set(pData[0], pData[1], pData[2]);
	} else {
		res.fill(0.0f);
	}
	return res;
}

cxVec sxGeometryData::get_pnt_normal(int pntIdx) const {
	cxVec nrm(0.0f, 1.0f, 0.0f);
	int attrIdx = find_pnt_attr("N");
	if (attrIdx >= 0) {
		float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 3);
		if (pData) {
			nrm.from_mem(pData);
		}
	}
	return nrm;
}

cxVec sxGeometryData::get_pnt_tangent(int pntIdx) const {
	cxVec tng(1.0f, 0.0f, 0.0f);
	int attrIdx = find_pnt_attr("tangentu");
	if (attrIdx >= 0) {
		float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 3);
		if (pData) {
			tng.from_mem(pData);
		}
	}
	return tng;
}

cxVec sxGeometryData::get_pnt_bitangent(int pntIdx) const {
	cxVec btg(0.0f, 0.0f, 1.0f);
	int attrIdx = find_pnt_attr("tangentv");
	if (attrIdx >= 0) {
		float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 3);
		if (pData) {
			btg.from_mem(pData);
		} else {
			btg = calc_pnt_bitangent(pntIdx);
		}
	} else {
		btg = calc_pnt_bitangent(pntIdx);
	}
	return btg;
}

cxVec sxGeometryData::calc_pnt_bitangent(int pntIdx) const {
	cxVec nrm = get_pnt_normal(pntIdx);
	cxVec tng = get_pnt_tangent(pntIdx);
	cxVec btg = nxVec::cross(tng, nrm);
	btg.normalize();
	return btg;
}

cxColor sxGeometryData::get_pnt_color(int pntIdx, bool useAlpha) const {
	cxColor clr(1.0f, 1.0f, 1.0f);
	int attrIdx = find_pnt_attr("Cd");
	if (attrIdx >= 0) {
		float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 3);
		if (pData) {
			clr.set(pData[0], pData[1], pData[2]);
		}
	}
	if (useAlpha) {
		attrIdx = find_pnt_attr("Alpha");
		if (attrIdx >= 0) {
			float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 1);
			if (pData) {
				clr.set_a(*pData);
			}
		}
	}
	return clr;
}

xt_texcoord sxGeometryData::get_pnt_texcoord(int pntIdx) const {
	xt_texcoord tex;
	tex.set(0.0f, 0.0f);
	int attrIdx = find_pnt_attr("uv");
	if (attrIdx >= 0) {
		float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 2);
		if (pData) {
			tex.set(pData[0], pData[1]);
		}
	}
	return tex;
}

xt_texcoord sxGeometryData::get_pnt_texcoord2(int pntIdx) const {
	xt_texcoord tex;
	tex.set(0.0f, 0.0f);
	int attrIdx = find_pnt_attr("uv2");
	if (attrIdx < 0) {
		attrIdx = find_pnt_attr("uv");
	}
	if (attrIdx >= 0) {
		float* pData = get_attr_data_f(attrIdx, eAttrClass::POINT, pntIdx, 2);
		if (pData) {
			tex.set(pData[0], pData[1]);
		}
	}
	return tex;
}

int sxGeometryData::get_pnt_wgt_num(int pntIdx) const {
	int n = 0;
	if (has_skin() && ck_pnt_idx(pntIdx)) {
		uint32_t* pSkinTbl = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mSkinOffs));
		uint8_t* pWgtNum = reinterpret_cast<uint8_t*>(&pSkinTbl[mPntNum]);
		n = pWgtNum[pntIdx];
	}
	return n;
}

int sxGeometryData::get_pnt_skin_jnt(int pntIdx, int wgtIdx) const {
	int idx = -1;
	if (has_skin() && ck_pnt_idx(pntIdx)) {
		uint32_t* pSkinTbl = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mSkinOffs));
		uint8_t* pWgtNum = reinterpret_cast<uint8_t*>(&pSkinTbl[mPntNum]);
		int nwgt = pWgtNum[pntIdx];
		if ((uint32_t)wgtIdx < (uint32_t)nwgt) {
			float* pWgt = reinterpret_cast<float*>(XD_INCR_PTR(this, pSkinTbl[pntIdx]));
			uint8_t* pIdx = reinterpret_cast<uint8_t*>(&pWgt[nwgt]);
			int nnodes = get_skin_nodes_num();
			if (nnodes <= (1 << 8)) {
				idx = pIdx[wgtIdx];
			} else {
				idx = pIdx[wgtIdx*2];
				idx |= pIdx[wgtIdx*2 + 1] << 8;
			}
		}
	}
	return idx;
}

float sxGeometryData::get_pnt_skin_wgt(int pntIdx, int wgtIdx) const {
	float w = 0.0f;
	if (has_skin() && ck_pnt_idx(pntIdx)) {
		uint32_t* pSkinTbl = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mSkinOffs));
		uint8_t* pWgtNum = reinterpret_cast<uint8_t*>(&pSkinTbl[mPntNum]);
		int nwgt = pWgtNum[pntIdx];
		if ((uint32_t)wgtIdx < (uint32_t)nwgt) {
			float* pWgt = reinterpret_cast<float*>(XD_INCR_PTR(this, pSkinTbl[pntIdx]));
			w = pWgt[wgtIdx];
		}
	}
	return w;
}

int sxGeometryData::find_mtl_grp_idx(const char* pName, const char* pPath) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	int nmtl = (int)mMtlNum;
	if (nmtl && mMtlOffs && pStrLst) {
		int nameId = pStrLst->find_str(pName);
		if (nameId >= 0) {
			uint32_t* pOffs = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mMtlOffs));
			if (pPath) {
				int pathId = pStrLst->find_str(pPath);
				if (pathId >= 0) {
					for (int i = 0; i < nmtl; ++i) {
						if (pOffs[i]) {
							GrpInfo* pInfo = reinterpret_cast<GrpInfo*>(XD_INCR_PTR(this, pOffs[i]));
							if (pInfo->mNameId == nameId && pInfo->mPathId == pathId) {
								idx = i;
								break;
							}
						}
					}
				}
			} else {
				for (int i = 0; i < nmtl; ++i) {
					if (pOffs[i]) {
						GrpInfo* pInfo = reinterpret_cast<GrpInfo*>(XD_INCR_PTR(this, pOffs[i]));
						if (pInfo->mNameId == nameId) {
							idx = i;
							break;
						}
					}
				}
			}
		}
	}
	return idx;
}

sxGeometryData::GrpInfo* sxGeometryData::get_mtl_info(int idx) const {
	GrpInfo* pInfo = nullptr;
	if (ck_mtl_idx(idx) && mMtlOffs) {
		uint32_t offs = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mMtlOffs))[idx];
		if (offs) {
			pInfo = reinterpret_cast<GrpInfo*>(XD_INCR_PTR(this, offs));
		}
	}
	return pInfo;
}

sxGeometryData::Group sxGeometryData::get_mtl_grp(int idx) const {
	Group grp;
	GrpInfo* pInfo = get_mtl_info(idx);
	if (pInfo) {
		grp.mpGeom = this;
		grp.mpInfo = pInfo;
	} else {
		grp.mpGeom = nullptr;
		grp.mpInfo = nullptr;
	}
	return grp;
}

sxGeometryData::GrpInfo* sxGeometryData::get_pnt_grp_info(int idx) const {
	GrpInfo* pInfo = nullptr;
	if (ck_pnt_grp_idx(idx) && mPntGrpOffs) {
		uint32_t offs = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mPntGrpOffs))[idx];
		if (offs) {
			pInfo = reinterpret_cast<GrpInfo*>(XD_INCR_PTR(this, offs));
		}
	}
	return pInfo;
}

sxGeometryData::Group sxGeometryData::get_pnt_grp(int idx) const {
	Group grp;
	GrpInfo* pInfo = get_pnt_grp_info(idx);
	if (pInfo) {
		grp.mpGeom = this;
		grp.mpInfo = pInfo;
	} else {
		grp.mpGeom = nullptr;
		grp.mpInfo = nullptr;
	}
	return grp;
}

sxGeometryData::GrpInfo* sxGeometryData::get_pol_grp_info(int idx) const {
	GrpInfo* pInfo = nullptr;
	if (ck_pol_grp_idx(idx) && mPolGrpOffs) {
		uint32_t offs = reinterpret_cast<uint32_t*>(XD_INCR_PTR(this, mPolGrpOffs))[idx];
		if (offs) {
			pInfo = reinterpret_cast<GrpInfo*>(XD_INCR_PTR(this, offs));
		}
	}
	return pInfo;
}

sxGeometryData::Group sxGeometryData::get_pol_grp(int idx) const {
	Group grp;
	GrpInfo* pInfo = get_pol_grp_info(idx);
	if (pInfo) {
		grp.mpGeom = this;
		grp.mpInfo = pInfo;
	} else {
		grp.mpGeom = nullptr;
		grp.mpInfo = nullptr;
	}
	return grp;
}

void sxGeometryData::hit_query_nobvh(const cxLineSeg& seg, HitFunc& fun) const {
	cxVec hitPos;
	cxVec hitNrm;
	QuadInfo quadInfo;
	int npol = get_pol_num();
	for (int i = 0; i < npol; ++i) {
		Polygon pol = get_pol(i);
		bool hitFlg = pol.intersect(seg, &hitPos, &hitNrm, &quadInfo);
		if (hitFlg) {
			float hitDist = nxVec::dist(seg.get_pos0(), hitPos);
			bool contFlg = fun(pol, hitPos, hitNrm, hitDist, quadInfo);
			if (!contFlg) {
				break;
			}
		}
	}
}

struct sxBVHWork {
	const sxGeometryData* mpGeo;
	cxAABB mQryBBox;
	cxLineSeg mQrySeg;
	sxGeometryData::HitFunc* mpHitFunc;
	sxGeometryData::RangeFunc* mpRangeFunc;
	bool mStopFlg;

	sxBVHWork() : mpGeo(nullptr), mpHitFunc(nullptr), mpRangeFunc(nullptr) {}
};

static void BVH_hit_sub(sxBVHWork& wk, int nodeId) {
	if (wk.mStopFlg) return;
	sxGeometryData::BVH::Node* pNode = wk.mpGeo->get_BVH_node(nodeId);
	if (pNode->mBBox.overlaps(wk.mQryBBox) && pNode->mBBox.seg_ck(wk.mQrySeg)) {
		if (pNode->is_leaf()) {
			cxVec hitPos;
			cxVec hitNrm;
			sxGeometryData::QuadInfo quadInfo;
			sxGeometryData::Polygon pol = wk.mpGeo->get_pol(pNode->get_pol_id());
			bool hitFlg = pol.intersect(wk.mQrySeg, &hitPos, &hitNrm, &quadInfo);
			if (hitFlg) {
				float hitDist = nxVec::dist(wk.mQrySeg.get_pos0(), hitPos);
				bool contFlg = (*wk.mpHitFunc)(pol, hitPos, hitNrm, hitDist, quadInfo);
				if (!contFlg) {
					wk.mStopFlg = true;
				}
			}
		} else {
			BVH_hit_sub(wk, pNode->mLeft);
			BVH_hit_sub(wk, pNode->mRight);
		}
	}
}

void sxGeometryData::hit_query(const cxLineSeg& seg, HitFunc& fun) const {
	BVH* pBVH = get_BVH();
	if (pBVH) {
		sxBVHWork wk;
		wk.mpGeo = this;
		wk.mQrySeg = seg;
		wk.mQryBBox.from_seg(seg);
		wk.mpHitFunc = &fun;
		wk.mStopFlg = false;
		BVH_hit_sub(wk, 0);
	} else {
		hit_query_nobvh(seg, fun);
	}
}

void sxGeometryData::range_query_nobvh(const cxAABB& box, RangeFunc& fun) const {
	int npol = get_pol_num();
	for (int i = 0; i < npol; ++i) {
		Polygon pol = get_pol(i);
		bool rangeFlg = pol.calc_bbox().overlaps(box);
		if (rangeFlg) {
			bool contFlg = fun(pol);
			if (!contFlg) {
				break;
			}
		}
	}
}

static void BVH_range_sub(sxBVHWork& wk, int nodeId) {
	if (wk.mStopFlg) return;
	sxGeometryData::BVH::Node* pNode = wk.mpGeo->get_BVH_node(nodeId);
	if (wk.mQryBBox.overlaps(pNode->mBBox)) {
		if (pNode->is_leaf()) {
			sxGeometryData::Polygon pol = wk.mpGeo->get_pol(pNode->get_pol_id());
			bool contFlg = (*wk.mpRangeFunc)(pol);
			if (!contFlg) {
				wk.mStopFlg = true;
			}
		} else {
			BVH_range_sub(wk, pNode->mLeft);
			BVH_range_sub(wk, pNode->mRight);
		}
	}
}

void sxGeometryData::range_query(const cxAABB& box, RangeFunc& fun) const {
	BVH* pBVH = get_BVH();
	if (pBVH) {
		sxBVHWork wk;
		wk.mpGeo = this;
		wk.mQryBBox = box;
		wk.mpRangeFunc = &fun;
		wk.mStopFlg = false;
		BVH_range_sub(wk, 0);
	} else {
		range_query_nobvh(box, fun);
	}
}

cxAABB sxGeometryData::calc_world_bbox(cxMtx* pMtxW, int* pIdxMap) const {
	cxAABB bbox = mBBox;
	if (pMtxW) {
		if (has_skin()) {
			cxSphere* pSph = get_skin_sph_top();
			if (pSph) {
				int n = mSkinNodeNum;
				for (int i = 0; i < n; ++i) {
					int idx = i;
					if (pIdxMap) {
						idx = pIdxMap[i];
					}
					cxSphere sph = pSph[i];
					cxVec spos = pMtxW[idx].calc_pnt(sph.get_center());
					cxVec rvec(sph.get_radius());
					cxAABB sbb(spos - rvec, spos + rvec);
					if (i == 0) {
						bbox = sbb;
					} else {
						bbox.merge(sbb);
					}
				}
			}
		} else {
			bbox.transform(*pMtxW);
		}
	}
	return bbox;
}

uint8_t* sxGeometryData::Polygon::get_vtx_lst() const {
	uint8_t* pLst = nullptr;
	if (is_valid()) {
		if (mpGeom->is_same_pol_size()) {
			uint32_t offs = mpGeom->mPolOffs;
			if (!mpGeom->is_same_pol_mtl()) {
				offs += mpGeom->get_pol_num() * get_mtl_id_size();
			}
			int vlstSize = mpGeom->get_vtx_idx_size() * mpGeom->mMaxVtxPerPol;
			offs += vlstSize*mPolId;
			pLst = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpGeom, offs));
		} else {
			uint32_t offs = reinterpret_cast<uint32_t*>(XD_INCR_PTR(mpGeom, mpGeom->mPolOffs))[mPolId];
			pLst = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpGeom, offs));
		}
	}
	return pLst;
}

int sxGeometryData::Polygon::get_vtx_pnt_id(int vtxIdx) const {
	int pntIdx = -1;
	if (ck_vtx_idx(vtxIdx)) {
		uint8_t* pIdx = get_vtx_lst();
		int idxSize = mpGeom->get_vtx_idx_size();
		pIdx += idxSize * vtxIdx;
		switch (idxSize) {
			case 1:
				pntIdx = *pIdx;
				break;
			case 2:
				pntIdx = pIdx[0] | (pIdx[1] << 8);
				break;
			case 3:
				pntIdx = pIdx[0] | (pIdx[1] << 8) | (pIdx[2] << 16);
				break;
		}
	}
	return pntIdx;
}

int sxGeometryData::Polygon::get_vtx_num() const {
	int nvtx = 0;
	if (is_valid()) {
		if (mpGeom->is_same_pol_size()) {
			nvtx = mpGeom->mMaxVtxPerPol;
		} else {
			int npol = mpGeom->get_pol_num();
			uint8_t* pNum = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpGeom, mpGeom->mPolOffs));
			pNum += npol * 4;
			if (!mpGeom->is_same_pol_mtl()) {
				pNum += npol * get_mtl_id_size();
			}
			if (mpGeom->mMaxVtxPerPol < (1 << 8)) {
				nvtx = pNum[mPolId];
			} else {
				pNum += mPolId * 2;
				nvtx = pNum[0] | (pNum[1] << 8);
			}
		}
	}
	return nvtx;
}

int sxGeometryData::Polygon::get_mtl_id() const {
	int mtlId = -1;
	if (mpGeom->get_mtl_num() > 0 && is_valid()) {
		if (mpGeom->is_same_pol_mtl()) {
			mtlId = 0;
		} else {
			uint8_t* pId = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpGeom, mpGeom->mPolOffs));
			if (!mpGeom->is_same_pol_size()) {
				pId += mpGeom->get_pol_num() * 4;
			}
			switch (get_mtl_id_size()) {
				case 1:
					mtlId = reinterpret_cast<int8_t*>(pId)[mPolId];
					break;
				case 2:
				default:
					mtlId = reinterpret_cast<int16_t*>(pId)[mPolId];
					break;
			}
		}
	}
	return mtlId;
}

cxVec sxGeometryData::Polygon::calc_centroid() const {
	int nvtx = get_vtx_num();
	cxVec c(0.0f);
	for (int i = 0; i < nvtx; ++i) {
		c += get_vtx_pos(i);
	}
	c.scl(nxCalc::rcp0((float)nvtx));
	return c;
}

cxAABB sxGeometryData::Polygon::calc_bbox() const {
	cxAABB bbox;
	bbox.init();
	int nvtx = get_vtx_num();
	for (int i = 0; i < nvtx; ++i) {
		bbox.add_pnt(get_vtx_pos(i));
	}
	return bbox;
}

cxVec sxGeometryData::Polygon::calc_normal_cw() const {
	cxVec nrm;
	nrm.zero();
	cxVec* pPnt = mpGeom->get_pnt_top();
	int nvtx = get_vtx_num();
	for (int i = 0; i < nvtx; ++i) {
		int j = i - 1;
		if (j < 0) j = nvtx - 1;
		int pntI = get_vtx_pnt_id(i);
		int pntJ = get_vtx_pnt_id(j);
		nxGeom::update_nrm_newell(&nrm, &pPnt[pntI], &pPnt[pntJ]);
	}
	nrm.normalize();
	return nrm;
}

cxVec sxGeometryData::Polygon::calc_normal_ccw() const {
	cxVec nrm;
	nrm.zero();
	cxVec* pPnt = mpGeom->get_pnt_top();
	int nvtx = get_vtx_num();
	for (int i = 0; i < nvtx; ++i) {
		int j = i + 1;
		if (j >= nvtx) j = 0;
		int pntI = get_vtx_pnt_id(i);
		int pntJ = get_vtx_pnt_id(j);
		nxGeom::update_nrm_newell(&nrm, &pPnt[pntI], &pPnt[pntJ]);
	}
	nrm.normalize();
	return nrm;
}

bool sxGeometryData::Polygon::is_planar(float eps) {
	int nvtx = get_vtx_num();
	if (nvtx < 4) return true;
	cxVec c = calc_centroid();
	cxVec n = calc_normal_cw();
	cxPlane pln;
	pln.calc(c, n);
	for (int i = 0; i < nvtx; ++i) {
		if (pln.dist(get_vtx_pos(i)) > eps) return false;
	}
	return true;
}

bool sxGeometryData::Polygon::intersect(const cxLineSeg& seg, cxVec* pHitPos, cxVec* pHitNrm, QuadInfo* pQuadInfo) {
	int i;
	bool res = false;
	int nvtx = get_vtx_num();
	bool planarFlg = false;
	int cvxMsk = 0;
	int hitTriIdx = 0;
	static uint8_t tri0[] = { 0, 1, 2,  0, 2, 3 };
	static uint8_t tri1[] = { 0, 1, 3,  1, 2, 3 };
	uint8_t* pTri;
	cxVec qvtx[4];
	cxVec sp0 = seg.get_pos0();
	cxVec sp1 = seg.get_pos1();
	switch (nvtx) {
		case 3:
			res = nxGeom::seg_tri_intersect_cw(sp0, sp1, get_vtx_pos(0), get_vtx_pos(1), get_vtx_pos(2), pHitPos, pHitNrm);
			break;
		case 4:
			for (i = 0; i < 4; ++i) {
				qvtx[i] = get_vtx_pos(i);
			}
			pTri = tri0;
			if (mpGeom->all_quads_planar_convex()) {
				planarFlg = true;
				cvxMsk = 3;
			} else {
				planarFlg = is_planar();
				cvxMsk = nxGeom::quad_convex_ck(qvtx[0], qvtx[1], qvtx[2], qvtx[3]);
			}
			if (planarFlg && cvxMsk == 3) {
				res = nxGeom::seg_quad_intersect_cw(sp0, sp1, qvtx[0], qvtx[1], qvtx[2], qvtx[3], pHitPos, pHitNrm);
			} else {
				if (cvxMsk == 1) {
					pTri = tri0;
				} else {
					pTri = tri1;
				}
				hitTriIdx = 0;
				res = nxGeom::seg_tri_intersect_cw(sp0, sp1, qvtx[pTri[0]], qvtx[pTri[1]], qvtx[pTri[2]], pHitPos, pHitNrm);
				if (!res) {
					res = nxGeom::seg_tri_intersect_cw(sp0, sp1, qvtx[pTri[3]], qvtx[pTri[4]], qvtx[pTri[5]], pHitPos, pHitNrm);
					hitTriIdx = 1;
				}
			}
			if (res) {
				if (pQuadInfo) {
					pQuadInfo->mIsPlanar = planarFlg;
					pQuadInfo->mConvexMask = cvxMsk;
					pQuadInfo->mHitIdx = hitTriIdx;
					pQuadInfo->mTri0V0 = pTri[0];
					pQuadInfo->mTri0V1 = pTri[1];
					pQuadInfo->mTri0V2 = pTri[2];
					pQuadInfo->mTri1V0 = pTri[3];
					pQuadInfo->mTri1V1 = pTri[4];
					pQuadInfo->mTri1V2 = pTri[5];
				}
			}
			break;
		default:
			break;
	}
	return res;
}


void* sxGeometryData::Group::get_idx_top() const {
	void* pIdxTop = is_valid() ? mpInfo + 1 : nullptr;
	if (pIdxTop) {
		int nskn = mpInfo->mSkinNodeNum;
		if (nskn) {
			if (mpGeom->has_skin_spheres()) {
				pIdxTop = XD_INCR_PTR(pIdxTop, sizeof(cxSphere)*nskn); /* skip spheres */
			}
			pIdxTop = XD_INCR_PTR(pIdxTop, sizeof(uint16_t)*nskn); /* skip node list */
		}
	}
	return pIdxTop;
}

int sxGeometryData::Group::get_idx_elem_size() const {
	int idxSpan = mpInfo->mMaxIdx - mpInfo->mMinIdx;
	int res = 0;
	if (idxSpan < (1 << 8)) {
		res = 1;
	} else if (idxSpan < (1 << 16)) {
		res = 2;
	} else {
		res = 3;
	}
	return res;
}

int sxGeometryData::Group::get_rel_idx(int at) const {
	int idx = -1;
	if (ck_idx(at)) {
		void* pIdxTop = get_idx_top();
		int elemSize = get_idx_elem_size();
		if (elemSize == 1) {
			idx = reinterpret_cast<uint8_t*>(pIdxTop)[at];
		} else if (elemSize == 2) {
			idx = reinterpret_cast<uint16_t*>(pIdxTop)[at];
		} else {
			uint8_t* pIdx = reinterpret_cast<uint8_t*>(XD_INCR_PTR(pIdxTop, at*3));
			idx = pIdx[0] | (pIdx[1] << 8) | (pIdx[2] << 16);
		}
	}
	return idx;
}

int sxGeometryData::Group::get_idx(int at) const {
	int idx = -1;
	if (ck_idx(at)) {
		idx = get_rel_idx(at) + mpInfo->mMinIdx;
	}
	return idx;
}

bool sxGeometryData::Group::contains(int idx) const {
	bool res = false;
	if (is_valid() && idx >= mpInfo->mMinIdx && idx <= mpInfo->mMaxIdx) {
		int bit = idx - mpInfo->mMinIdx;
		void* pIdxTop = get_idx_top();
		int idxElemSize = get_idx_elem_size();
		uint8_t* pBits = reinterpret_cast<uint8_t*>(pIdxTop) + mpInfo->mIdxNum*idxElemSize;
		res = !!(pBits[bit >> 3] & (1 << (bit & 7)));
	}
	return res;
}

cxSphere* sxGeometryData::Group::get_skin_spheres() const {
	cxSphere* pSph = nullptr;
	if (is_valid()) {
		int nskn = mpInfo->mSkinNodeNum;
		if (nskn) {
			if (mpGeom->has_skin_spheres()) {
				pSph = (cxSphere*)(mpInfo + 1);
			}
		}
	}
	return pSph;
}

uint16_t* sxGeometryData::Group::get_skin_ids() const {
	uint16_t* pLst = nullptr;
	if (is_valid()) {
		int nskn = mpInfo->mSkinNodeNum;
		if (nskn) {
			if (mpGeom->has_skin_spheres()) {
				pLst = (uint16_t*)XD_INCR_PTR(mpInfo + 1, sizeof(cxSphere)*nskn);
			} else {
				pLst = (uint16_t*)(mpInfo + 1);
			}
		}
	}
	return pLst;
}


void sxTextureData::Plane::expand(float* pDst, int pixelStride) const {
	PlaneInfo* pInfo = mpTex->get_plane_info(mPlaneId);
	uint8_t* pBits = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpTex, pInfo->mDataOffs));
	const int tblSize = 1 << 8;
	int32_t tbl[tblSize];
	::memset(tbl, 0, sizeof(tbl));
	int bitCnt = pInfo->mBitCount;
	uxVal32 uval;
	int bitPtr = 0;
	int pred = 0;
	int hash = 0;
	while (bitCnt) {
		int bitLen = nxCore::fetch_bits32_loop(pBits, bitPtr, 5);
		bitPtr += 5;
		int xor = 0;
		if (bitLen) {
			xor = nxCore::fetch_bits32_loop(pBits, bitPtr, bitLen);
			bitPtr += bitLen;
		}
		bitCnt -= 5 + bitLen;
		int ival = xor ^ pred;
		tbl[hash] = ival;
		hash = (ival >> 21) & (tblSize - 1);
		pred = tbl[hash];
		uval.i = ival << pInfo->mTrailingZeroes;
		float fval = uval.f;
		fval += pInfo->mValOffs;
		*pDst = fval;
		pDst += pixelStride;
	}
}

void sxTextureData::Plane::get_data(float* pDst, int pixelStride) const {
	int w = mpTex->get_width();
	int h = mpTex->get_height();
	int npix = w * h;
	PlaneInfo* pInfo = mpTex->get_plane_info(mPlaneId);
	if (pInfo) {
		void* pDataTop = XD_INCR_PTR(mpTex, pInfo->mDataOffs);
		if (pInfo->is_const()) {
			float cval = *reinterpret_cast<float*>(pDataTop);
			for (int i = 0; i < npix; ++i) {
				*pDst = cval;
				pDst += pixelStride;
			}
		} else if (pInfo->is_compressed()) {
			expand(pDst, pixelStride);
		} else {
			float* pSrc = reinterpret_cast<float*>(pDataTop);
			for (int i = 0; i < npix; ++i) {
				*pDst = *pSrc;
				pDst += pixelStride;
				++pSrc;
			}
		}
	}
}

float* sxTextureData::Plane::get_data() const {
	int w = mpTex->get_width();
	int h = mpTex->get_height();
	int npix = w * h;
	int memsize = npix * sizeof(float);
	float* pDst = reinterpret_cast<float*>(nxCore::mem_alloc(memsize));
	get_data(pDst);
	return pDst;
}

int sxTextureData::calc_mip_num() const {
	int n = 1;
	uint32_t w = get_width();
	uint32_t h = get_height();
	while (w > 1 && h > 1) {
		if (w > 1) w >>= 1;
		if (h > 1) h >>= 1;
		++n;
	}
	return n;
}

int sxTextureData::find_plane_idx(const char* pName) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	int npln = mPlaneNum;
	if (npln && pStrLst) {
		int nameId = pStrLst->find_str(pName);
		if (nameId >= 0) {
			for (int i = 0; i < npln; ++i) {
				PlaneInfo* pInfo = get_plane_info(i);
				if (pInfo && pInfo->mNameId == nameId) {
					idx = i;
					break;
				}
			}
		}
	}
	return idx;
}

sxTextureData::Plane sxTextureData::find_plane(const char* pName) const {
	Plane plane;
	int idx = find_plane_idx(pName);
	if (idx < 0) {
		plane.mpTex = nullptr;
		plane.mPlaneId = -1;
	} else {
		plane.mpTex = this;
		plane.mPlaneId = idx;
	}
	return plane;
}

sxTextureData::Plane sxTextureData::get_plane(int idx) const {
	Plane plane;
	if (ck_plane_idx(idx)) {
		plane.mpTex = this;
		plane.mPlaneId = idx;
	} else {
		plane.mpTex = nullptr;
		plane.mPlaneId = -1;
	}
	return plane;
}

void sxTextureData::get_rgba(float* pDst, bool cvtToSRGB) const {
	if (!pDst) return;
	int w = get_width();
	int h = get_height();
	int npix = w * h;
	int memsize = npix * sizeof(cxColor);
	::memset(pDst, 0, memsize);
	static const char* plnName[] = { "r", "g", "b", "a" };
	for (int i = 0; i < 4; ++i) {
		Plane pln = find_plane(plnName[i]);
		if (pln.is_valid()) {
			pln.get_data(pDst + i, 4);
		}
	}
	if (cvtToSRGB) {
		cxColor* pClr = reinterpret_cast<cxColor*>(pDst);
		for (int i = 0; i < npix; ++i) {
			pClr[i].make_sRGB();
		}
	}
}

cxColor* sxTextureData::get_rgba(bool cvtToSRGB) const {
	int w = get_width();
	int h = get_height();
	int npix = w * h;
	int memsize = npix * sizeof(cxColor);
	float* pDst = reinterpret_cast<float*>(nxCore::mem_alloc(memsize));
	get_rgba(pDst, cvtToSRGB);
	return reinterpret_cast<cxColor*>(pDst);
}

namespace nxTexture {

sxDDSHead* alloc_dds128(int w, int h, uint32_t* pSize) {
	uint32_t npix = w * h;
	uint32_t dataSize = npix * sizeof(cxColor);
	uint32_t size = sizeof(sxDDSHead) + dataSize;
	sxDDSHead* pDDS = reinterpret_cast<sxDDSHead*>(nxCore::mem_alloc(size));
	if (pDDS) {
		::memset(pDDS, 0, sizeof(sxDDSHead));
		pDDS->mMagic = XD_FOURCC('D', 'D', 'S', ' ');
		pDDS->mSize = sizeof(sxDDSHead) - 4;
		pDDS->mFlags = 0x081007;
		pDDS->mHeight = h;
		pDDS->mWidth = w;
		pDDS->mPitchLin = dataSize;
		pDDS->mFormat.mSize = 0x20;
		pDDS->mFormat.mFlags = 4;
		pDDS->mFormat.mFourCC = 0x74;
		pDDS->mCaps1 = 0x1000;
		if (pSize) {
			*pSize = size;
		}
	}
	return pDDS;
}

}// nxTexture

sxTextureData::DDS sxTextureData::get_dds(bool cvtToSRGB) const {
	DDS dds;
	int w = get_width();
	int h = get_height();
	dds.mpHead = nxTexture::alloc_dds128(w, h, &dds.mSize);
	if (dds.mpHead) {
		float* pDst = reinterpret_cast<float*>(dds.mpHead + 1);
		get_rgba(pDst, cvtToSRGB);
	}
	return dds;
}

void sxTextureData::DDS::release() {
	if (mpHead) {
		nxCore::mem_free(mpHead);
	}
	mpHead = nullptr;
	mSize = 0;
}

void sxTextureData::DDS::save(const char* pOutPath) const {
	if (is_valid()) {
		nxCore::bin_save(pOutPath, mpHead, mSize);
	}
}

/* see PBRT book for details */
static void calc_resample_wgts(int oldRes, int newRes, xt_float4* pWgt, int16_t* pOrg) {
	float rt = float(oldRes) / float(newRes);
	float fw = 2.0f;
	for (int i = 0; i < newRes; ++i) {
		float c = (float(i) + 0.5f) * rt;
		float org = nxCalc::floor((c - fw) + 0.5f);
		pOrg[i] = (int16_t)org;
		float* pW = pWgt[i];
		float s = 0.0f;
		for (int j = 0; j < 4; ++j) {
			float pos = org + float(j) + 0.5f;
			float x = ::fabsf((pos - c) / fw);
			float w;
			if (x < 1.0e-5f) {
				w = 1.0f;
			} else if (x > 1.0f) {
				w = 1.0f;
			} else {
				x *= XD_PI;
				w = nxCalc::sinc(x*2.0f) * nxCalc::sinc(x);
			}
			pW[j] = w;
			s += w;
		}
		s = nxCalc::rcp0(s);
		pWgt[i].scl(s);
	}
}

sxTextureData::Pyramid* sxTextureData::get_pyramid() const {
	sxTextureData::Pyramid* pPmd = nullptr;
	int w0 = get_width();
	int h0 = get_height();
	int w = w0;
	int h = h0;
	bool flgW = nxCore::is_pow2(w);
	bool flgH = nxCore::is_pow2(h);
	int baseW = flgW ? w : (1 << (1 + (int)::log2f(float(w))));
	int baseH = flgH ? h : (1 << (1 + (int)::log2f(float(h))));
	w = baseW;
	h = baseH;
	int nlvl = 1;
	int npix = 1;
	while (w > 1 || h > 1) {
		npix += w*h;
		if (w > 1) w >>= 1;
		if (h > 1) h >>= 1;
		++nlvl;
	}
	size_t headSize = sizeof(Pyramid) + (nlvl - 1) * sizeof(uint32_t);
	uint32_t topOffs = (uint32_t)XD_ALIGN(headSize, 0x10);
	size_t memSize = topOffs + npix * sizeof(cxColor);
	pPmd = reinterpret_cast<sxTextureData::Pyramid*>(nxCore::mem_alloc(memSize));
	pPmd->mBaseWidth = baseW;
	pPmd->mBaseHeight = baseH;
	pPmd->mLvlNum = nlvl;
	w = baseW;
	h = baseH;
	uint32_t offs = topOffs;
	for (int i = 0; i < nlvl; ++i) {
		pPmd->mLvlOffs[i] = offs;
		offs += w*h * sizeof(cxColor);
		if (w > 1) w >>= 1;
		if (h > 1) h >>= 1;
	}
	int wgtNum = nxCalc::max(baseW, baseH);
	cxColor* pBase = get_rgba();
	cxColor* pLvl = pPmd->get_lvl(0);
	if (flgW && flgH) {
		::memcpy(pLvl, pBase, baseW*baseH * sizeof(cxColor));
	} else {
		xt_float4* pWgt = reinterpret_cast<xt_float4*>(nxCore::mem_alloc(wgtNum * sizeof(xt_float4)));
		int16_t* pOrg = reinterpret_cast<int16_t*>(nxCore::mem_alloc(wgtNum * sizeof(int16_t)));
		cxColor* pTmp = reinterpret_cast<cxColor*>(nxCore::mem_alloc(wgtNum * sizeof(cxColor)));
		calc_resample_wgts(w0, baseW, pWgt, pOrg);
		for (int y = 0; y < h0; ++y) {
			for (int x = 0; x < baseW; ++x) {
				cxColor clr;
				clr.zero();
				xt_float4 wgt = pWgt[x];
				for (int i = 0; i < 4; ++i) {
					int x0 = nxCalc::clamp(pOrg[x] + i, 0, w0 - 1);
					cxColor csrc = pBase[y*w0 + x0];
					csrc.scl(wgt[i]);
					clr.add(csrc);
				}
				pLvl[y*baseW + x] = clr;
			}
		}
		calc_resample_wgts(h0, baseH, pWgt, pOrg);
		for (int x = 0; x < baseW; ++x) {
			for (int y = 0; y < baseH; ++y) {
				cxColor clr;
				clr.zero();
				xt_float4 wgt = pWgt[y];
				for (int i = 0; i < 4; ++i) {
					int y0 = nxCalc::clamp(pOrg[y] + i, 0, h0 - 1);
					cxColor csrc = pLvl[y0*baseW + x];
					csrc.scl(wgt[i]);
					clr.add(csrc);
				}
				pTmp[y] = clr;
			}
			for (int y = 0; y < baseH; ++y) {
				pLvl[y*baseW + x] = pTmp[y];
			}
		}
		nxCore::mem_free(pTmp);
		nxCore::mem_free(pOrg);
		nxCore::mem_free(pWgt);
	}
	nxCore::mem_free(pBase);
	w = baseW / 2;
	h = baseH / 2;
	for (int i = 1; i < nlvl; ++i) {
		cxColor* pLvlSrc = pPmd->get_lvl(i - 1);
		cxColor* pLvlDst = pPmd->get_lvl(i);
		int wsrc = w * 2;
		int hsrc = h * 2;
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				int sx0 = nxCalc::min(x * 2, wsrc - 1);
				int sy0 = nxCalc::min(y * 2, hsrc - 1);
				int sx1 = nxCalc::min(sx0 + 1, wsrc - 1);
				int sy1 = nxCalc::min(sy0 + 1, hsrc - 1);
				cxColor clr = pLvlSrc[sy0*wsrc + sx0];
				clr.add(pLvlSrc[sy0*wsrc + sx1]);
				clr.add(pLvlSrc[sy1*wsrc + sx0]);
				clr.add(pLvlSrc[sy1*wsrc + sx1]);
				clr.scl(0.25f);
				pLvlDst[y*w + x] = clr;
			}
		}
		if (w > 1) w >>= 1;
		if (h > 1) h >>= 1;
	}
	return pPmd;
}

void sxTextureData::Pyramid::get_lvl_dims(int idx, int* pWidth, int* pHeight) const {
	int w = 0;
	int h = 0;
	if (ck_lvl_idx(idx)) {
		w = mBaseWidth;
		h = mBaseHeight;
		for (int i = 0; i < idx; ++i) {
			if (w > 1) w >>= 1;
			if (h > 1) h >>= 1;
		}
	}
	if (pWidth) {
		*pWidth = w;
	}
	if (pHeight) {
		*pHeight = h;
	}
}

sxTextureData::DDS sxTextureData::Pyramid::get_lvl_dds(int idx) const {
	DDS dds;
	dds.mpHead = nullptr;
	dds.mSize = 0;
	if (ck_lvl_idx(idx)) {
		int w, h;
		get_lvl_dims(idx, &w, &h);
		dds.mpHead = nxTexture::alloc_dds128(w, h, &dds.mSize);
		cxColor* pLvl = get_lvl(idx);
		if (pLvl) {
			cxColor* pDst = reinterpret_cast<cxColor*>(dds.mpHead + 1);
			::memcpy(pDst, pLvl, w*h * sizeof(cxColor));
		}
	}
	return dds;
}


bool sxKeyframesData::has_node_info() const {
	bool res = false;
	if (mHeadSize > offsetof(sxKeyframesData, mNodeInfoNum)) {
		res = mNodeInfoNum != 0 && mNodeInfoOffs != 0;
	}
	return res;
}

int sxKeyframesData::find_node_info_idx(const char* pName, const char* pPath, int startIdx) const {
	int idx = -1;
	if (has_node_info()) {
		sxStrList* pStrLst = get_str_list();
		if (pStrLst && ck_node_info_idx(startIdx)) {
			NodeInfo* pInfo = reinterpret_cast<NodeInfo*>(XD_INCR_PTR(this, mNodeInfoOffs));
			int ninfo = get_node_info_num();
			int nameId = pName ? pStrLst->find_str(pName) : -1;
			if (nameId >= 0) {
				if (pPath) {
					int pathId = pStrLst->find_str(pPath);
					if (pathId >= 0) {
						for (int i = startIdx; i < ninfo; ++i) {
							if (pInfo[i].mNameId == nameId && pInfo[i].mPathId == pathId) {
								idx = i;
								break;
							}
						}
					}
				} else {
					for (int i = startIdx; i < ninfo; ++i) {
						if (pInfo[i].mNameId == nameId) {
							idx = i;
							break;
						}
					}
				}
			} else if (pPath) {
				int pathId = pStrLst->find_str(pPath);
				if (pathId >= 0) {
					for (int i = startIdx; i < ninfo; ++i) {
						if (pInfo[i].mPathId == pathId) {
							idx = i;
							break;
						}
					}
				}
			}
		}
	}
	return idx;
}

template<int IDX_SIZE> int fno_get(uint8_t* pTop, int at) {
	uint8_t* pFno = pTop + at*IDX_SIZE;
	int fno = -1;
	switch (IDX_SIZE) {
		case 1:
			fno = *pFno;
			break;
		case 2:
			fno = pFno[0] | (pFno[1] << 8);
			break;
		case 3:
			fno = pFno[0] | (pFno[1] << 8) | (pFno[2] << 16);
			break;
	}
	return fno;
}

template<int IDX_SIZE> int find_fno_idx_lin(uint8_t* pTop, int num, int fno) {
	for (int i = 1; i < num; ++i) {
		int ck = fno_get<IDX_SIZE>(pTop, i);
		if (fno < ck) {
			return i - 1;
		}
	}
	return num - 1;
}

template<int IDX_SIZE> int find_fno_idx_bin(uint8_t* pTop, int num, int fno) {
	uint32_t istart = 0;
	uint32_t iend = num;
	do {
		uint32_t imid = (istart + iend) / 2;
		int ck = fno_get<IDX_SIZE>(pTop, imid);
		if (fno < ck) {
			iend = imid;
		} else {
			istart = imid;
		}
	} while (iend - istart >= 2);
	return istart;
}

template<int IDX_SIZE> int find_fno_idx(uint8_t* pTop, int num, int fno) {
	if (num > 10) {
		return find_fno_idx_bin<IDX_SIZE>(pTop, num, fno);
	}
	return find_fno_idx_lin<IDX_SIZE>(pTop, num, fno);
}

int sxKeyframesData::FCurve::find_key_idx(int fno) const {
	int idx = -1;
	if (is_valid() && mpKfr->ck_fno(fno)) {
		FCurveInfo* pInfo = get_info();
		if (!pInfo->is_const()) {
			if (pInfo->mFnoOffs) {
				uint8_t* pFno = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpKfr, pInfo->mFnoOffs));
				int nfno = pInfo->mKeyNum;
				int maxFno = mpKfr->get_max_fno();
				if (maxFno < (1 << 8)) {
					idx = find_fno_idx<1>(pFno, nfno, fno);
				} else if (maxFno < (1 << 16)) {
					idx = find_fno_idx<2>(pFno, nfno, fno);
				} else {
					idx = find_fno_idx<3>(pFno, nfno, fno);
				}
			} else {
				idx = fno;
			}
		}
	}
	return idx;
}

int sxKeyframesData::FCurve::get_fno(int idx) const {
	int fno = 0;
	FCurveInfo* pInfo = get_info();
	if (pInfo) {
		if (!pInfo->is_const() && (uint32_t)idx < (uint32_t)get_key_num()) {
			if (pInfo->has_fno_lst()) {
				uint8_t* pFno = reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpKfr, pInfo->mFnoOffs));
				int maxFno = mpKfr->get_max_fno();
				if (maxFno < (1 << 8)) {
					fno = fno_get<1>(pFno, idx);
				} else if (maxFno < (1 << 16)) {
					fno = fno_get<2>(pFno, idx);
				} else {
					fno = fno_get<3>(pFno, idx);
				}
			} else {
				fno = idx;
			}
		}
	}
	return fno;
}

float sxKeyframesData::FCurve::eval(float frm, bool extrapolate) const {
	float val = 0.0f;
	int fno = (int)frm;
	if (is_valid()) {
		FCurveInfo* pInfo = get_info();
		if (pInfo->is_const()) {
			val = pInfo->mMinVal;
		} else {
			bool loopFlg = false;
			if (pInfo->has_fno_lst()) {
				float lastFno = (float)get_fno(pInfo->mKeyNum - 1);
				if (frm > lastFno && frm < lastFno + 1.0f) {
					loopFlg = true;
				}
			}
			if (loopFlg) {
				int lastIdx = pInfo->mKeyNum - 1;
				eFunc func = pInfo->get_common_func();
				if (pInfo->mFuncOffs) {
					func = (eFunc)reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpKfr, pInfo->mFuncOffs))[lastIdx];
				}
				float* pVals = reinterpret_cast<float*>(XD_INCR_PTR(mpKfr, pInfo->mValOffs));
				if (func == eFunc::CONSTANT) {
					val = pVals[lastIdx];
				} else {
					float t = frm - (float)fno;
					if (extrapolate) {
						val = nxCalc::lerp(pVals[lastIdx], pVals[lastIdx] + pVals[0], t);
					} else {
						val = nxCalc::lerp(pVals[lastIdx], pVals[0], t);
					}
				}
			} else if (mpKfr->ck_fno(fno)) {
				float* pVals = reinterpret_cast<float*>(XD_INCR_PTR(mpKfr, pInfo->mValOffs));
				int i0 = find_key_idx(fno);
				int i1 = i0 + 1;
				float f0 = (float)get_fno(i0);
				float f1 = (float)get_fno(i1);
				float ffrc = frm - (float)fno;
				if (frm != f0) {
					eFunc func = pInfo->get_common_func();
					if (pInfo->mFuncOffs) {
						func = (eFunc)reinterpret_cast<uint8_t*>(XD_INCR_PTR(mpKfr, pInfo->mFuncOffs))[i0];
					}
					if (func == eFunc::CONSTANT) {
						val = pVals[i0];
					} else {
						float t = (frm - f0) / (f1 - f0);
						float v0 = pVals[i0];
						float v1 = pVals[i1];
						if (func == eFunc::LINEAR) {
							val = nxCalc::lerp(v0, v1, t);
						} else if (func == eFunc::CUBIC) {
							float* pSlopeL = pInfo->mLSlopeOffs ? reinterpret_cast<float*>(XD_INCR_PTR(mpKfr, pInfo->mLSlopeOffs)) : nullptr;
							float* pSlopeR = pInfo->mRSlopeOffs ? reinterpret_cast<float*>(XD_INCR_PTR(mpKfr, pInfo->mRSlopeOffs)) : nullptr;
							float outgoing = 0.0f;
							float incoming = 0.0f;
							if (pSlopeR) {
								outgoing = pSlopeR[i0];
							}
							if (pSlopeL) {
								incoming = pSlopeL[i1];
							}
							float fscl = nxCalc::rcp0(mpKfr->mFPS);
							float seg = (f1 - f0) * fscl;
							outgoing *= seg;
							incoming *= seg;
							val = nxCalc::hermite(v0, outgoing, v1, incoming, t);
						}
					}
				} else {
					val = pVals[i0];
				}
			}
		}
	}
	return val;
}

int sxKeyframesData::find_fcv_idx(const char* pNodeName, const char* pChanName, const char* pNodePath) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	if (pStrLst) {
		int chanId = pStrLst->find_str(pChanName);
		int nameId = chanId > 0 ? pStrLst->find_str(pNodeName) : -1;
		if (chanId >= 0 && nameId >= 0) {
			int i;
			FCurveInfo* pInfo;
			int nfcv = get_fcv_num();
			if (pNodePath) {
				int pathId = pStrLst->find_str(pNodePath);
				if (pathId >= 0) {
					for (i = 0; i < nfcv; ++i) {
						pInfo = get_fcv_info(i);
						if (pInfo && pInfo->mChanNameId == chanId && pInfo->mNodeNameId == nameId && pInfo->mNodePathId == pathId) {
							idx = i;
							break;
						}
					}
				}
			} else {
				for (i = 0; i < nfcv; ++i) {
					pInfo = get_fcv_info(i);
					if (pInfo && pInfo->mChanNameId == chanId && pInfo->mNodeNameId == nameId) {
						idx = i;
						break;
					}
				}
			}
		}
	}
	return idx;
}

sxKeyframesData::FCurve sxKeyframesData::get_fcv(int idx) const {
	FCurve fcv;
	if (ck_fcv_idx(idx)) {
		fcv.mpKfr = this;
		fcv.mFcvId = idx;
	} else {
		fcv.mpKfr = nullptr;
		fcv.mFcvId = -1;
	}
	return fcv;
}

void sxKeyframesData::dump_clip(const char* pOutPath) const {
	char outPath[MAX_PATH];
	if (!pOutPath) {
		const char* pSrcPath = get_file_path();
		if (!pSrcPath) {
			return;
		}
		::sprintf_s(outPath, sizeof(outPath), "%s.clip", pSrcPath);
		pOutPath = outPath;
	}
	FILE* pOut = nullptr;
	if (::fopen_s(&pOut, pOutPath, "w") != 0) {
		return;
	}
	int nfcv = get_fcv_num();
	int nfrm = get_frame_count();
	::fprintf(pOut, "{\n");
	::fprintf(pOut, "	rate = %d\n", (int)get_frame_rate());
	::fprintf(pOut, "	start = -1\n");
	::fprintf(pOut, "	tracklength = %d\n", nfrm);
	::fprintf(pOut, "	tracks = %d\n", nfcv);
	for (int i = 0; i < nfcv; ++i) {
		::fprintf(pOut, "   {\n");
		FCurve fcv = get_fcv(i);
		const char* pNodePath = fcv.get_node_path();
		const char* pNodeName = fcv.get_node_name();
		const char* pChanName = fcv.get_chan_name();
		::fprintf(pOut, "      name = %s/%s:%s\n", pNodePath, pNodeName, pChanName);
		::fprintf(pOut, "      data =");
		for (int j = 0; j < nfrm; ++j) {
			float frm = (float)j;
			float val = fcv.eval(frm);
			::fprintf(pOut, " %f", val);
		}
		::fprintf(pOut, "\n");
		::fprintf(pOut, "   }\n");
	}
	::fprintf(pOut, "}\n");
	::fclose(pOut);
}


enum class exExprFunc {
	_abs, _acos, _asin, _atan, _atan2, _ceil,
	_ch, _clamp, _cos, _deg, _detail, _distance, _exp,
	_fit, _fit01, _fit10, _fit11,
	_floor, _frac, _if, _int, _length,
	_log, _log10, _max, _min, _pow, _rad,
	_rint, _round, _sign, _sin, _sqrt, _tan
};


void sxCompiledExpression::Stack::alloc(int n) {
	free();
	if (n < 1) return;
	size_t memsize = n * sizeof(Val) + Tags::calc_mem_size(n);
	mpMem = nxCore::mem_alloc(memsize);
	if (mpMem) {
		::ZeroMemory(mpMem, memsize);
		mpVals = reinterpret_cast<Val*>(mpMem);
		mTags.init(mpVals + n, n);
	}
}

void sxCompiledExpression::Stack::free() {
	if (mpMem) {
		nxCore::mem_free(mpMem);
	}
	mpMem = nullptr;
	mpVals = nullptr;
	mPtr = 0;
	mTags.reset();
}

XD_NOINLINE void sxCompiledExpression::Stack::push_num(float num) {
	if (is_full()) {
		return;
	}
	mpVals[mPtr].num = num;
	mTags.set_num(mPtr);
	++mPtr;
}

XD_NOINLINE float sxCompiledExpression::Stack::pop_num() {
	if (is_empty()) {
		return 0.0f;
	}
	--mPtr;
	if (mTags.is_num(mPtr)) {
		return mpVals[mPtr].num;
	}
	return 0.0f;
}

XD_NOINLINE void sxCompiledExpression::Stack::push_str(int ptr) {
	if (is_full()) {
		return;
	}
	mpVals[mPtr].sptr = ptr;
	mTags.set_str(mPtr);
	++mPtr;
}

XD_NOINLINE int sxCompiledExpression::Stack::pop_str() {
	if (is_empty()) {
		return -1;
	}
	--mPtr;
	if (mTags.is_str(mPtr)) {
		return mpVals[mPtr].sptr;
	}
	return -1;
}

void sxCompiledExpression::get_str(int idx, String* pStr) const {
	if (!pStr) return;
	const StrInfo* pInfo = get_str_info(idx);
	if (pInfo) {
		pStr->mpChars = reinterpret_cast<const char*>(XD_INCR_PTR(&get_str_info_top()[mStrsNum], pInfo->mOffs));
		pStr->mHash = pInfo->mHash;
		pStr->mLen = pInfo->mLen;
	} else {
		pStr->mpChars = nullptr;
		pStr->mHash = 0;
		pStr->mLen = 0;
	}
}

sxCompiledExpression::String sxCompiledExpression::get_str(int idx) const {
	String str;
	get_str(idx, &str);
	return str;
}

static inline float expr_acos(float val) {
	return XD_RAD2DEG(::acosf(nxCalc::clamp(val, -1.0f, 1.0f)));
}

static inline float expr_asin(float val) {
	return XD_RAD2DEG(::asinf(nxCalc::clamp(val, -1.0f, 1.0f)));
}

static inline float expr_atan(float val) {
	return XD_RAD2DEG(::atanf(val));
}

static inline float expr_atan2(float y, float x) {
	return XD_RAD2DEG(::atan2f(y, x));
}

static inline float expr_dist(sxCompiledExpression::Stack* pStk) {
	cxVec p1;
	p1.z = pStk->pop_num();
	p1.y = pStk->pop_num();
	p1.x = pStk->pop_num();
	cxVec p0;
	p0.z = pStk->pop_num();
	p0.y = pStk->pop_num();
	p0.x = pStk->pop_num();
	return nxVec::dist(p0, p1);
}

static inline float expr_fit(sxCompiledExpression::Stack* pStk) {
	float newmax = pStk->pop_num();
	float newmin = pStk->pop_num();
	float oldmax = pStk->pop_num();
	float oldmin = pStk->pop_num();
	float num = pStk->pop_num();
	return nxCalc::fit(num, oldmin, oldmax, newmin, newmax);
}

static inline float expr_fit01(sxCompiledExpression::Stack* pStk) {
	float newmax = pStk->pop_num();
	float newmin = pStk->pop_num();
	float num = nxCalc::saturate(pStk->pop_num());
	return nxCalc::fit(num, 0.0f, 1.0f, newmin, newmax);
}

static inline float expr_fit10(sxCompiledExpression::Stack* pStk) {
	float newmax = pStk->pop_num();
	float newmin = pStk->pop_num();
	float num = nxCalc::saturate(pStk->pop_num());
	return nxCalc::fit(num, 1.0f, 0.0f, newmin, newmax);
}

static inline float expr_fit11(sxCompiledExpression::Stack* pStk) {
	float newmax = pStk->pop_num();
	float newmin = pStk->pop_num();
	float num = nxCalc::clamp(pStk->pop_num(), -1.0f, 1.0f);
	return nxCalc::fit(num, -1.0f, 1.0f, newmin, newmax);
}

static inline float expr_len(sxCompiledExpression::Stack* pStk) {
	cxVec v;
	v.z = pStk->pop_num();
	v.y = pStk->pop_num();
	v.x = pStk->pop_num();
	return v.mag();
}

void sxCompiledExpression::exec(ExecIfc& ifc) const {
	Stack* pStk = ifc.get_stack();
	if (!pStk) {
		return;
	}

	float valA;
	float valB;
	float valC;
	float cmpRes;
	int funcId;
	String str1;
	String str2;
	int n = mCodeNum;
	const Code* pCode = get_code_top();
	for (int i = 0; i < n; ++i) {
		eOp op = pCode->get_op();
		if (op == eOp::END) break;
		switch (op) {
		case eOp::NUM:
			pStk->push_num(get_val(pCode->mInfo));
			break;
		case eOp::STR:
			pStk->push_str(pCode->mInfo);
			break;
		case eOp::VAR:
			get_str(pStk->pop_str(), &str1);
			pStk->push_num(ifc.var(str1));
			break;
		case eOp::CMP:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			cmpRes = 0.0f;
			switch ((eCmp)pCode->mInfo) {
				case eCmp::EQ:
					cmpRes = valA == valB ? 1.0f : 0.0f;
					break;
				case eCmp::NE:
					cmpRes = valA != valB ? 1.0f : 0.0f;
					break;
				case eCmp::LT:
					cmpRes = valA < valB ? 1.0f : 0.0f;
					break;
				case eCmp::LE:
					cmpRes = valA <= valB ? 1.0f : 0.0f;
					break;
				case eCmp::GT:
					cmpRes = valA > valB ? 1.0f : 0.0f;
					break;
				case eCmp::GE:
					cmpRes = valA >= valB ? 1.0f : 0.0f;
					break;
			}
			pStk->push_num(cmpRes);
			break;
		case eOp::ADD:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num(valA + valB);
			break;
		case eOp::SUB:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num(valA - valB);
			break;
		case eOp::MUL:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num(valA * valB);
			break;
		case eOp::DIV:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num(nxCalc::div0(valA, valB));
			break;
		case eOp::MOD:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num(valB != 0.0f ? ::fmodf(valA, valB) : 0.0f);
			break;
		case eOp::NEG:
			valA = pStk->pop_num();
			pStk->push_num(-valA);
			break;
		case eOp::FUN:
			funcId = pCode->mInfo;
			switch ((exExprFunc)funcId) {
			case exExprFunc::_abs:
				valA = pStk->pop_num();
				pStk->push_num(::fabsf(valA));
				break;
			case exExprFunc::_acos:
				valA = pStk->pop_num();
				pStk->push_num(expr_acos(valA));
				break;
			case exExprFunc::_asin:
				valA = pStk->pop_num();
				pStk->push_num(expr_asin(valA));
				break;
			case exExprFunc::_atan:
				valA = pStk->pop_num();
				pStk->push_num(expr_atan(valA));
				break;
			case exExprFunc::_atan2:
				valB = pStk->pop_num();
				valA = pStk->pop_num();
				pStk->push_num(expr_atan2(valA, valB));
				break;
			case exExprFunc::_ceil:
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::ceil(valA));
				break;
			case exExprFunc::_ch:
				get_str(pStk->pop_str(), &str1);
				pStk->push_num(ifc.ch(str1));
				break;
			case exExprFunc::_clamp:
				valC = pStk->pop_num();
				valB = pStk->pop_num();
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::clamp(valA, valB, valC));
				break;
			case exExprFunc::_cos:
				valA = pStk->pop_num();
				pStk->push_num(::cosf(XD_DEG2RAD(valA)));
				break;
			case exExprFunc::_deg:
				valA = pStk->pop_num();
				pStk->push_num(XD_RAD2DEG(valA));
				break;
			case exExprFunc::_detail:
				valA = pStk->pop_num();
				get_str(pStk->pop_str(), &str2);
				get_str(pStk->pop_str(), &str1);
				pStk->push_num(ifc.detail(str1, str2, (int)valA));
				break;
			case exExprFunc::_distance:
				pStk->push_num(expr_dist(pStk));
				break;
			case exExprFunc::_exp:
				valA = pStk->pop_num();
				pStk->push_num(::expf(valA));
				break;
			case exExprFunc::_fit:
				pStk->push_num(expr_fit(pStk));
				break;
			case exExprFunc::_fit01:
				pStk->push_num(expr_fit01(pStk));
				break;
			case exExprFunc::_fit10:
				pStk->push_num(expr_fit10(pStk));
				break;
			case exExprFunc::_fit11:
				pStk->push_num(expr_fit11(pStk));
				break;
			case exExprFunc::_floor:
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::floor(valA));
				break;
			case exExprFunc::_frac:
				valA = pStk->pop_num();
				pStk->push_num(valA - nxCalc::floor(valA)); // Houdini-compatible: frac(-2.7) = 0.3
				break;
			case exExprFunc::_if:
				valC = pStk->pop_num();
				valB = pStk->pop_num();
				valA = pStk->pop_num();
				pStk->push_num(valA != 0.0f ? valB : valC);
				break;
			case exExprFunc::_int:
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::trunc(valA));
				break;
			case exExprFunc::_length:
				pStk->push_num(expr_len(pStk));
				break;
			case exExprFunc::_log:
				valA = pStk->pop_num();
				pStk->push_num(::logf(valA));
				break;
			case exExprFunc::_log10:
				valA = pStk->pop_num();
				pStk->push_num(::log10f(valA));
				break;
			case exExprFunc::_max:
				valB = pStk->pop_num();
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::max(valA, valB));
				break;
			case exExprFunc::_min:
				valB = pStk->pop_num();
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::min(valA, valB));
				break;
			case exExprFunc::_pow:
				valB = pStk->pop_num();
				valA = pStk->pop_num();
				pStk->push_num(::powf(valA, valB));
				break;
			case exExprFunc::_rad:
				valA = pStk->pop_num();
				pStk->push_num(XD_DEG2RAD(valA));
				break;
			case exExprFunc::_rint:
			case exExprFunc::_round:
				valA = pStk->pop_num();
				pStk->push_num(nxCalc::round(valA));
				break;
			case exExprFunc::_sign:
				valA = pStk->pop_num();
				pStk->push_num(valA == 0.0f ? 0 : valA < 0.0f ? -1.0f : 1.0f);
				break;
			case exExprFunc::_sin:
				valA = pStk->pop_num();
				pStk->push_num(::sinf(XD_DEG2RAD(valA)));
				break;
			case exExprFunc::_sqrt:
				valA = pStk->pop_num();
				pStk->push_num(::sqrtf(valA));
				break;
			case exExprFunc::_tan:
				valA = pStk->pop_num();
				pStk->push_num(::tanf(XD_DEG2RAD(valA)));
				break;
			default:
				break;
			}
			break;
		case eOp::XOR:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num((float)((int)valA ^ (int)valB));
			break;
		case eOp::AND:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num((float)((int)valA & (int)valB));
			break;
		case eOp::OR:
			valB = pStk->pop_num();
			valA = pStk->pop_num();
			pStk->push_num((float)((int)valA | (int)valB));
			break;
		default:
			break;
		}

		++pCode;
	}
	float res = pStk->pop_num();
	ifc.set_result(res);
}

static const char* s_exprOpNames[] = {
	"NOP", "END", "NUM", "STR", "VAR", "CMP", "ADD", "SUB", "MUL", "DIV", "MOD", "NEG", "FUN", "XOR", "AND", "OR"
};

static const char* s_exprCmpNames[] = {
	"EQ", "NE", "LT", "LE", "GT", "GE"
};

static const char* s_exprFuncNames[] = {
	"abs", "acos", "asin", "atan", "atan2", "ceil",
	"ch", "clamp", "cos", "deg", "detail", "distance",
	"exp", "fit", "fit01", "fit10", "fit11",
	"floor", "frac", "if", "int", "length",
	"log", "log10", "max", "min", "pow", "rad",
	"rint", "round", "sign", "sin", "sqrt", "tan"
};

void sxCompiledExpression::disasm(FILE* pFile) const {
	int n = mCodeNum;
	const Code* pCode = get_code_top();
	char abuf[128];
	for (int i = 0; i < n; ++i) {
		int addr = i;
		eOp op = pCode->get_op();
		const char* pOpName = "<bad>";
		if ((uint32_t)op < (uint32_t)XD_ARY_LEN(s_exprOpNames)) {
			pOpName = s_exprOpNames[(int)op];
		}
		const char* pArg = "";
		if (op == eOp::CMP) {
			eCmp cmp = (eCmp)pCode->mInfo;
			pArg = "<bad>";
			if ((uint32_t)cmp < (uint32_t)XD_ARY_LEN(s_exprCmpNames)) {
				pArg = s_exprCmpNames[(int)cmp];
			}
		} else if (op == eOp::NUM) {
			float num = get_val(pCode->mInfo);
			::sprintf_s(abuf, sizeof(abuf), "%f", num);
			pArg = abuf;
		} else if (op == eOp::STR) {
			String str = get_str(pCode->mInfo);
			if (str.is_valid()) {
				::sprintf_s(abuf, sizeof(abuf), "%s", str.mpChars);
				pArg = abuf;
			}
		} else if (op == eOp::FUN) {
			int func = pCode->mInfo;
			pArg = "<bad>";
			if ((uint32_t)func < (uint32_t)XD_ARY_LEN(s_exprFuncNames)) {
				pArg = s_exprFuncNames[func];
			}
		}
		::fprintf(pFile, "0x%02X: %s %s\n", addr, pOpName, pArg);

		++pCode;
	}
}


int sxExprLibData::find_expr_idx(const char* pNodeName, const char* pChanName, const char* pNodePath, int startIdx) const {
	int idx = -1;
	sxStrList* pStrLst = get_str_list();
	if (ck_expr_idx(startIdx) && pStrLst) {
		int nameId = pStrLst->find_str(pNodeName);
		int chanId = pStrLst->find_str(pChanName);
		int pathId = pStrLst->find_str(pNodePath);
		bool doneFlg = (pNodeName && (nameId < 0)) || (pChanName && (chanId < 0)) || (pNodePath && pathId < 0);
		if (!doneFlg) {
			int n = get_expr_num();
			for (int i = startIdx; i < n; ++i) {
				const ExprInfo* pInfo = get_info(i);
				bool nameFlg = pNodeName ? pInfo->mNodeNameId == nameId : true;
				bool chanFlg = pChanName ? pInfo->mChanNameId == chanId : true;
				bool pathFlg = pNodePath ? pInfo->mNodePathId == pathId : true;
				if (nameFlg && chanFlg && pathFlg) {
					idx = i;
					break;
				}
			}
		}
	}
	return idx;
}

sxExprLibData::Entry sxExprLibData::get_entry(int idx) const {
	Entry ent;
	if (ck_expr_idx(idx)) {
		ent.mpLib = this;
		ent.mExprId = idx;
	} else {
		ent.mpLib = nullptr;
		ent.mExprId = -1;
	}
	return ent;
}

int sxExprLibData::count_rig_entries(const sxRigData& rig) const {
	int n = 0;
	sxStrList* pStrLst = get_str_list();
	for (int i = 0; i < rig.get_nodes_num(); ++i) {
		const char* pNodeName = rig.get_node_name(i);
		const char* pNodePath = rig.get_node_path(i);
		int nameId = pStrLst->find_str(pNodeName);
		int pathId = pStrLst->find_str(pNodePath);
		if (nameId >= 0) {
			for (int j = 0; j < get_expr_num(); ++j) {
				const ExprInfo* pInfo = get_info(j);
				bool nameFlg = pInfo->mNodeNameId == nameId;
				bool pathFlg = pathId < 0 ? true : pInfo->mNodePathId == pathId;
				if (nameFlg && pathFlg) {
					++n;
				}
			}
		}
	}
	return n;
}

