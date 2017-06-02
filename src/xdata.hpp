/*
 * Author: Sergey Chaban <sergey.chaban@gmail.com>
 */

enum class exAnimChan : int8_t {
	UNKNOWN = -1,
	TX = 0,
	TY = 1,
	TZ = 2,
	RX = 3,
	RY = 4,
	RZ = 5,
	SX = 6,
	SY = 7,
	SZ = 8
};

struct sxStrList {
	uint32_t mSize;
	uint32_t mNum;
	uint32_t mOffs[1];

	bool ck_idx(int idx) const { return (uint32_t)idx < mNum; }
	uint16_t* get_hash_top() const { return (uint16_t*)&mOffs[mNum]; }
	const char* get_str(int idx) const { return ck_idx(idx) ? reinterpret_cast<const char*>(this) + mOffs[idx] : nullptr; }
	int find_str(const char* pStr) const;
};

struct sxVecList {
	struct Entry {
		uint32_t mOrg : 30;
		uint32_t mLen : 2;
	};

	uint32_t mSize;
	uint32_t mNum;
	Entry mTbl[1];

	bool ck_idx(int idx) const { return (uint32_t)idx < mNum; }
	const float* get_ptr(int idx) const { return ck_idx(idx) ? reinterpret_cast<const float*>(&mTbl[mNum]) + mTbl[idx].mOrg : nullptr; }
	int get_elems(float* pDst, int idx, int num) const;
	xt_float2 get_f2(int idx);
	xt_float3 get_f3(int idx);
	xt_float4 get_f4(int idx);
};

struct sxData {
	uint32_t mKind;
	uint32_t mFlags;
	uint32_t mFileSize;
	uint32_t mHeadSize;
	uint32_t mOffsStr;
	int16_t mNameId;
	int16_t mPathId;
	uint32_t mFilePathLen;
	uint32_t mReserved;

	sxStrList* get_str_list() const { return mOffsStr ? (sxStrList*)XD_INCR_PTR(this, mOffsStr) : nullptr; }
	const char* get_str(int id) const { sxStrList* pStrLst = get_str_list(); return pStrLst ? pStrLst->get_str(id) : nullptr; }
	int find_str(const char* pStr) const { return pStr && mOffsStr ? get_str_list()->find_str(pStr) : -1; }
	bool has_file_path() const { return !!mFilePathLen; }
	const char* get_file_path() const { return has_file_path() ? (const char*)XD_INCR_PTR(this, mFileSize) : nullptr; }
	const char* get_name() const { return get_str(mNameId); }
	const char* get_base_path() const { return get_str(mPathId); }

	template<typename T> bool is() const { return mKind == T::KIND; }
	template<typename T> T* as() const { return is<T>() ? (T*)this : nullptr; }
};



struct sxValuesData : public sxData {
	uint32_t mOffsVec;
	uint32_t mOffsGrp;

	enum class eValType {
		UNKNOWN = 0,
		FLOAT   = 1,
		VEC2    = 2,
		VEC3    = 3,
		VEC4    = 4,
		INT     = 5,
		STRING  = 6
	};

	struct ValInfo {
		int16_t mNameId;
		int8_t mType; /* ValType */
		int8_t mReserved;
		union {
			float f;
			int32_t i;
		} mValId;

		eValType get_type() const { return (eValType)mType; }
	};

	struct GrpInfo {
		int16_t mNameId;
		int16_t mPathId;
		int16_t mTypeId;
		int16_t mReserved;
		int32_t mValNum;
		ValInfo mVals[1];
	};

	struct GrpList {
		uint32_t mNum;
		uint32_t mOffs[1];
	};

	class Group {
	private:
		const sxValuesData* mpVals;
		int32_t mGrpId;

		Group() {}

		friend struct sxValuesData;

	public:
		bool is_valid() const { return mpVals && mpVals->ck_grp_idx(mGrpId); }
		bool ck_val_idx(int idx) const { return (uint32_t)idx < (uint32_t)get_val_num(); }
		const char* get_name() const { return is_valid() ? mpVals->get_str(get_info()->mNameId) : nullptr; }
		const char* get_path() const { return is_valid() ? mpVals->get_str(get_info()->mPathId) : nullptr; }
		const char* get_type() const { return is_valid() ? mpVals->get_str(get_info()->mTypeId) : nullptr; }
		const GrpInfo* get_info() const { return is_valid() ? mpVals->get_grp_info(mGrpId) : nullptr; }
		int get_val_num() const { return is_valid() ? get_info()->mValNum : 0; }
		int find_val_idx(const char* pName) const;
		const char* get_val_name(int idx) const { return ck_val_idx(idx) ? mpVals->get_str(get_info()->mVals[idx].mNameId) : nullptr; }
		eValType get_val_type(int idx) const { return ck_val_idx(idx) ? get_info()->mVals[idx].get_type() : eValType::UNKNOWN; }
		const ValInfo* get_val_info(int idx) const { return ck_val_idx(idx) ? &get_info()->mVals[idx] : nullptr; }
		int get_val_i(int idx) const;
		float get_val_f(int idx) const;
		xt_float2 get_val_f2(int idx) const;
		xt_float3 get_val_f3(int idx) const;
		xt_float4 get_val_f4(int idx) const;
		const char* get_val_s(int idx) const;
		float get_float(const char* pName, const float defVal = 0.0f) const;
		int get_int(const char* pName, const int defVal = 0) const;
		cxVec get_vec(const char* pName, const cxVec& defVal = cxVec(0.0f)) const;
		cxColor get_rgb(const char* pName, const cxColor& defVal = cxColor(0.0f)) const;
		const char* get_str(const char* pName, const char* pDefVal = "") const;
	};

	bool ck_grp_idx(int idx) const { return (uint32_t)idx < (uint32_t)get_grp_num(); }
	int find_grp_idx(const char* pName, const char* pPath = nullptr, int startIdx = 0) const;
	sxVecList* get_vec_list() const { return mOffsVec ? reinterpret_cast<sxVecList*>(XD_INCR_PTR(this, mOffsVec)) : nullptr; }
	GrpList* get_grp_list() const { return mOffsGrp ? reinterpret_cast<GrpList*>(XD_INCR_PTR(this, mOffsGrp)) : nullptr; }
	int get_grp_num() const { GrpList* pGrpLst = get_grp_list(); return pGrpLst ? pGrpLst->mNum : 0; };
	GrpInfo* get_grp_info(int idx) const { return ck_grp_idx(idx) ? reinterpret_cast<GrpInfo*>(XD_INCR_PTR(this, get_grp_list()->mOffs[idx])) : nullptr; }
	Group get_grp(int idx) const;
	Group find_grp(const char* pName, const char* pPath = nullptr) const { return get_grp(find_grp_idx(pName, pPath)); }

	static const char* get_val_type_str(eValType typ);

	static const uint32_t KIND = XD_FOURCC('X', 'V', 'A', 'L');
};


struct sxRigData : public sxData {
	uint32_t mNodeNum;
	uint32_t mLvlNum;
	uint32_t mOffsNode;
	uint32_t mOffsWMtx;
	uint32_t mOffsIMtx;
	uint32_t mOffsLMtx;
	uint32_t mOffsLPos;
	uint32_t mOffsLRot;
	uint32_t mOffsLScl;

	struct Node {
		int16_t mSelfIdx;
		int16_t mParentIdx;
		int16_t mNameId;
		int16_t mPathId;
		int16_t mTypeId;
		int16_t mLvl;
		uint16_t mAttr;
		uint8_t mRotOrd;
		uint8_t mXfmOrd;

		exRotOrd get_rot_order() const { return (exRotOrd)mRotOrd; }
		exTransformOrd get_xform_order() const { return (exTransformOrd)mXfmOrd; }
		bool is_hrc_top() const { return mParentIdx < 0; }
	};

	bool ck_node_idx(int idx) const { return (uint32_t)idx < mNodeNum; }
	int get_nodes_num() const { return mNodeNum; }
	int get_levels_num() const { return mLvlNum; }
	Node* get_node_top() const { return mOffsNode ? reinterpret_cast<Node*>(XD_INCR_PTR(this, mOffsNode)) : nullptr; }
	Node* get_node_ptr(int idx) const;
	const char* get_node_name(int idx) const;
	const char* get_node_path(int idx) const;
	const char* get_node_type(int idx) const;
	int find_node(const char* pName, const char* pPath = nullptr) const;
	exRotOrd get_rot_order(int idx) const { return ck_node_idx(idx) ? get_node_ptr(idx)->get_rot_order() : exRotOrd::XYZ; }
	exTransformOrd get_xform_order(int idx) const { return ck_node_idx(idx) ? get_node_ptr(idx)->get_xform_order() : exTransformOrd::SRT; }
	int get_parent_idx(int idx) const { return ck_node_idx(idx) ? get_node_ptr(idx)->mParentIdx : -1; }
	cxMtx get_wmtx(int idx) const;
	cxMtx* get_wmtx_ptr(int idx) const;
	cxMtx get_imtx(int idx) const;
	cxMtx* get_imtx_ptr(int idx) const;
	cxMtx get_lmtx(int idx) const;
	cxMtx* get_lmtx_ptr(int idx) const;
	cxMtx calc_lmtx(int idx) const;
	cxVec calc_parent_offs(int idx) const;
	float calc_parent_dist(int idx) const { return calc_parent_offs(idx).mag(); }
	cxVec get_lpos(int idx) const;
	cxVec get_lscl(int idx) const;
	cxVec get_lrot(int idx, bool inRadians = false) const;
	cxQuat calc_lquat(int idx) const;
	cxMtx calc_wmtx(int idx, const cxMtx* pMtxLocal, cxMtx* pParentWMtx = nullptr) const;

	void dump_node_names_f(FILE* pFile = stdout) const;
	void dump_node_names(const char* pOutPath = nullptr) const;

	static const uint32_t KIND = XD_FOURCC('X', 'R', 'I', 'G');
};


struct sxGeometryData : public sxData {
	cxAABB mBBox;
	uint32_t mPntNum;
	uint32_t mPolNum;
	uint32_t mMtlNum;
	uint32_t mGlbAttrNum;
	uint32_t mPntAttrNum;
	uint32_t mPolAttrNum;
	uint32_t mPntGrpNum;
	uint32_t mPolGrpNum;
	uint32_t mSkinNodeNum;
	uint16_t mMaxSkinWgtNum;
	uint16_t mMaxVtxPerPol;

	uint32_t mPntOffs;
	uint32_t mPolOffs;
	uint32_t mMtlOffs;
	uint32_t mGlbAttrOffs;
	uint32_t mPntAttrOffs;
	uint32_t mPolAttrOffs;
	uint32_t mPntGrpOffs;
	uint32_t mPolGrpOffs;
	uint32_t mSkinNodesOffs;
	uint32_t mSkinOffs;
	uint32_t mBVHOffs;

	enum class eAttrClass {
		GLOBAL,
		POINT,
		POLYGON
	};

	struct AttrInfo {
		uint32_t mDataOffs;
		int32_t mNameId;
		uint16_t mElemNum;
		uint8_t mType;
		uint8_t mReserved;

		bool is_int() const { return mType == 0; }
		bool is_float() const { return mType == 1; }
		bool is_string() const { return mType == 2; }
		bool is_float2() const { return is_float() && mElemNum == 2; }
		bool is_float3() const { return is_float() && mElemNum == 3; }
	};

	struct QuadInfo {
		uint32_t mIsPlanar : 1;
		uint32_t mConvexMask : 2;
		uint32_t mHitIdx : 1;
		uint32_t mTri0V0 : 2;
		uint32_t mTri0V1 : 2;
		uint32_t mTri0V2 : 2;
		uint32_t mTri1V0 : 2;
		uint32_t mTri1V1 : 2;
		uint32_t mTri1V2 : 2;

		int get_hit_tri_vtx(int vidx) const {
			int idx = -1;
			if (mHitIdx == 0) {
				switch (vidx) {
					case 0: idx = mTri0V0; break;
					case 1: idx = mTri0V1; break;
					case 2: idx = mTri0V2; break;
				}
			} else {
				switch (vidx) {
					case 0: idx = mTri1V0; break;
					case 1: idx = mTri1V1; break;
					case 2: idx = mTri1V2; break;
				}
			}
			return idx;
		}
	};

	class Polygon {
	private:
		const sxGeometryData* mpGeom;
		int32_t mPolId;

		Polygon() {}

		friend struct sxGeometryData;

		uint8_t* get_vtx_lst() const;
		int get_mtl_id_size() const { return mpGeom->get_mtl_num() < (1 << 7) ? 1 : 2; }

	public:
		const sxGeometryData* get_geo() const { return mpGeom; }
		int get_id() const { return mPolId; }
		bool is_valid() const { return (mpGeom && mpGeom->ck_pol_idx(mPolId)); }
		bool ck_vtx_idx(int idx) const { return (uint32_t)idx < (uint32_t)get_vtx_num(); }
		int get_vtx_pnt_id(int vtxIdx) const;
		int get_vtx_num() const;
		int get_mtl_id() const;
		cxVec get_vtx_pos(int vtxIdx) const { return is_valid() ? mpGeom->get_pnt(get_vtx_pnt_id(vtxIdx)) : cxVec(0.0f); };
		cxVec calc_centroid() const;
		cxAABB calc_bbox() const;
		cxVec calc_normal_cw() const;
		cxVec calc_normal_ccw() const;
		cxVec calc_normal() const { return calc_normal_cw(); }
		bool is_tri() const { return get_vtx_num() == 3; }
		bool is_quad() const { return get_vtx_num() == 4; }
		bool is_planar(float eps = 1.0e-6f) const;
		bool intersect(const cxLineSeg& seg, cxVec* pHitPos = nullptr, cxVec* pHitNrm = nullptr, QuadInfo* pQuadInfo = nullptr) const;
		bool contains_xz(const cxVec& pos) const;
	};

	struct GrpInfo {
		int16_t mNameId;
		int16_t mPathId;
		cxAABB mBBox;
		int32_t mMinIdx;
		int32_t mMaxIdx;
		uint16_t mMaxWgtNum;
		uint16_t mSkinNodeNum;
		uint32_t mIdxNum;
	};

	class Group {
	private:
		const sxGeometryData* mpGeom;
		GrpInfo* mpInfo;

		Group() {}

		friend struct sxGeometryData;

		void* get_idx_top() const;
		int get_idx_elem_size() const;

	public:
		bool is_valid() const { return (mpGeom && mpInfo); }
		const char* get_name() const { return is_valid() ? mpGeom->get_str(mpInfo->mNameId) : nullptr; }
		const char* get_path() const { return is_valid() ? mpGeom->get_str(mpInfo->mPathId) : nullptr; }
		GrpInfo* get_info() const { return mpInfo; }
		cxAABB get_bbox() const { return is_valid() ? mpInfo->mBBox : nxGeom::mk_empty_bbox(); }
		int get_max_wgt_num() const { return is_valid() ? mpInfo->mMaxWgtNum : 0; }
		bool ck_idx(int at) const { return is_valid() ? (uint32_t)at < mpInfo->mIdxNum : false; }
		uint32_t get_idx_num() const { return is_valid() ? mpInfo->mIdxNum : 0; }
		int get_min_idx() const { return is_valid() ? mpInfo->mMinIdx : -1; }
		int get_max_idx() const { return is_valid() ? mpInfo->mMaxIdx : -1; }
		int get_rel_idx(int at) const;
		int get_idx(int at) const;
		bool contains(int idx) const;
		int get_skin_nodes_num() const { return is_valid() ? mpInfo->mSkinNodeNum : 0; }
		cxSphere* get_skin_spheres() const;
		uint16_t* get_skin_ids() const;
	};

	class HitFunc {
	public:
		HitFunc() {}
		virtual ~HitFunc() {}
		virtual bool operator()(const Polygon& pol, const cxVec& hitPos, const cxVec& hitNrm, float hitDist, const QuadInfo& quadInfo) { return false; }
	};

	class RangeFunc {
	public:
		RangeFunc() {}
		virtual ~RangeFunc() {}
		virtual bool operator()(const Polygon& pol) { return false; }
	};

	struct BVH {
		struct Node {
			cxAABB mBBox;
			int32_t mLeft;
			int32_t mRight;

			bool is_leaf() const { return mRight < 0; }
			int get_pol_id() const { return mLeft; }
		};

		uint32_t mNodesNum;
		uint32_t mReserved0;
		uint32_t mReserved1;
		uint32_t mReserved2;
	};

	int get_pnt_num() const { return mPntNum; }
	int get_pol_num() const { return mPolNum; }
	int get_mtl_num() const { return mMtlNum; }
	int get_pnt_grp_num() const { return mPntGrpNum; }
	int get_pol_grp_num() const { return mPolGrpNum; }
	bool is_same_pol_size() const { return !!(mFlags & 1); }
	bool is_same_pol_mtl() const { return !!(mFlags & 2); }
	bool has_skin_spheres() const { return !!(mFlags & 4); }
	bool all_quads_planar_convex() const { return !!(mFlags & 8); }
	bool is_all_tris() const { return is_same_pol_size() && mMaxVtxPerPol == 3; }
	bool has_skin() const { return mSkinOffs != 0; }
	bool has_skin_nodes() const { return mSkinNodesOffs != 0; }
	bool has_BVH() const { return mBVHOffs != 0; }
	bool ck_BVH_node_idx(int idx) const { return has_BVH() ? (uint32_t)idx < get_BVH()->mNodesNum : false; }
	bool ck_pnt_idx(int idx) const { return (uint32_t)idx < mPntNum; }
	bool ck_pol_idx(int idx) const { return (uint32_t)idx < mPolNum; }
	bool ck_mtl_idx(int idx) const { return (uint32_t)idx < mMtlNum; }
	bool ck_pnt_grp_idx(int idx) const { return (uint32_t)idx < mPntGrpNum; }
	bool ck_pol_grp_idx(int idx) const { return (uint32_t)idx < mPolGrpNum; }
	bool ck_skin_idx(int idx) const { return (uint32_t)idx < mSkinNodeNum; }
	int get_vtx_idx_size() const;
	cxVec* get_pnt_top() const { return mPntOffs ? reinterpret_cast<cxVec*>(XD_INCR_PTR(this, mPntOffs)) : nullptr; }
	cxVec get_pnt(int idx) const { return ck_pnt_idx(idx) ? get_pnt_top()[idx] : cxVec(0.0f); }
	Polygon get_pol(int idx) const;
	cxSphere* get_skin_sph_top() const { return mSkinNodesOffs ? reinterpret_cast<cxSphere*>(XD_INCR_PTR(this, mSkinNodesOffs)) : nullptr; }
	int get_skin_nodes_num() const { return mSkinNodeNum; }
	int32_t* get_skin_node_name_ids() const;
	const char* get_skin_node_name(int idx) const;
	int find_skin_node(const char* pName) const;
	uint32_t get_attr_info_offs(eAttrClass cls) const;
	uint32_t get_attr_info_num(eAttrClass cls) const;
	uint32_t get_attr_item_num(eAttrClass cls) const;
	int find_attr(const char* pName, eAttrClass cls) const;
	int find_glb_attr(const char* pName) const { return find_attr(pName, eAttrClass::GLOBAL); }
	int find_pnt_attr(const char* pName) const { return find_attr(pName, eAttrClass::POINT); }
	int find_pol_attr(const char* pName) const { return find_attr(pName, eAttrClass::POLYGON); }
	AttrInfo* get_attr_info(int attrIdx, eAttrClass cls) const;
	AttrInfo* get_glb_attr_info(int attrIdx) const { return get_attr_info(attrIdx, eAttrClass::GLOBAL); }
	AttrInfo* get_pnt_attr_info(int attrIdx) const { return get_attr_info(attrIdx, eAttrClass::POINT); }
	AttrInfo* get_pol_attr_info(int attrIdx) const { return get_attr_info(attrIdx, eAttrClass::POLYGON); }
	float* get_attr_data_f(int attrIdx, eAttrClass cls, int itemIdx, int minElem = 1) const;
	float* get_glb_attr_data_f(int attrIdx, int minElem = 1) const { return get_attr_data_f(attrIdx, eAttrClass::GLOBAL, 0, minElem); }
	float* get_pnt_attr_data_f(int attrIdx, int itemIdx, int minElem = 1) const { return get_attr_data_f(attrIdx, eAttrClass::POINT, itemIdx, minElem); }
	float* get_pol_attr_data_f(int attrIdx, int itemIdx, int minElem = 1) const { return get_attr_data_f(attrIdx, eAttrClass::POLYGON, itemIdx, minElem); }
	float get_pnt_attr_val_f(int attrIdx, int pntIdx) const;
	xt_float3 get_pnt_attr_val_f3(int attrIdx, int pntIdx) const;
	cxVec get_pnt_normal(int pntIdx) const;
	cxVec get_pnt_tangent(int pntIdx) const;
	cxVec get_pnt_bitangent(int pntIdx) const;
	cxVec calc_pnt_bitangent(int pntIdx) const;
	cxColor get_pnt_color(int pntIdx, bool useAlpha = true) const;
	xt_texcoord get_pnt_texcoord(int pntIdx) const;
	xt_texcoord get_pnt_texcoord2(int pntIdx) const;
	int get_pnt_wgt_num(int pntIdx) const;
	int get_pnt_skin_jnt(int pntIdx, int wgtIdx) const;
	float get_pnt_skin_wgt(int pntIdx, int wgtIdx) const;
	int find_mtl_grp_idx(const char* pName, const char* pPath = nullptr) const;
	Group find_mtl_grp(const char* pName, const char* pPath = nullptr) const { return get_mtl_grp(find_mtl_grp_idx(pName, pPath)); }
	GrpInfo* get_mtl_info(int idx) const;
	Group get_mtl_grp(int idx) const;
	GrpInfo* get_pnt_grp_info(int idx) const;
	Group get_pnt_grp(int idx) const;
	GrpInfo* get_pol_grp_info(int idx) const;
	Group get_pol_grp(int idx) const;
	void hit_query_nobvh(const cxLineSeg& seg, HitFunc& fun) const;
	void hit_query(const cxLineSeg& seg, HitFunc& fun) const;
	void range_query_nobvh(const cxAABB& box, RangeFunc& fun) const;
	void range_query(const cxAABB& box, RangeFunc& fun) const;
	BVH* get_BVH() const { return has_BVH() ? reinterpret_cast<BVH*>(XD_INCR_PTR(this, mBVHOffs)) : nullptr; }
	BVH::Node* get_BVH_node(int nodeId) const { return ck_BVH_node_idx(nodeId) ? &reinterpret_cast<BVH::Node*>(get_BVH() + 1)[nodeId] : nullptr; }
	cxAABB calc_world_bbox(cxMtx* pMtxW, int* pIdxMap = nullptr) const;

	static const uint32_t KIND = XD_FOURCC('X', 'G', 'E', 'O');
};

struct sxDDSHead {
	uint32_t mMagic;
	uint32_t mSize;
	uint32_t mFlags;
	uint32_t mHeight;
	uint32_t mWidth;
	uint32_t mPitchLin;
	uint32_t mDepth;
	uint32_t mMipMapCount;
	uint32_t mReserved1[11];
	struct PixelFormat {
		uint32_t mSize;
		uint32_t mFlags;
		uint32_t mFourCC;
		uint32_t mBitCount;
		uint32_t mMaskR;
		uint32_t mMaskG;
		uint32_t mMaskB;
		uint32_t mMaskA;
	} mFormat;
	uint32_t mCaps1;
	uint32_t mCaps2;
	uint32_t mReserved2[3];
};

struct sxTextureData : public sxData {
	uint32_t mWidth;
	uint32_t mHeight;
	uint32_t mPlaneNum;
	uint32_t mPlaneOffs;

	struct PlaneInfo {
		uint32_t mDataOffs;
		int16_t mNameId;
		uint8_t mTrailingZeroes;
		int8_t mFormat;
		float mMinVal;
		float mMaxVal;
		float mValOffs;
		uint32_t mBitCount;
		uint32_t mReserved0;
		uint32_t mReserved1;

		bool is_const() const { return mFormat == 0; }
		bool is_compressed() const { return mFormat == 1; }
	};

	class Plane {
	private:
		const sxTextureData* mpTex;
		int32_t mPlaneId;

		Plane() {}

		void expand(float* pDst, int pixelStride) const;

		friend struct sxTextureData;
	public:
		bool is_valid() const { return mPlaneId >= 0; }
		void get_data(float* pDst, int pixelStride = 1) const;
		float* get_data() const;
	};

	class DDS {
	private:
		sxDDSHead* mpHead;
		uint32_t mSize;

		friend struct sxTextureData;

	public:
		bool is_valid() const { return mpHead && mSize; }
		void release();
		sxDDSHead* get_data_ptr() const { return mpHead; }
		uint32_t get_data_size() const { return mSize; }
		int get_width() const { return is_valid() ? mpHead->mWidth : 0; }
		int get_height() const { return is_valid() ? mpHead->mHeight : 0; }
		void save(const char* pOutPath) const;
	};

	struct Pyramid {
		int32_t mBaseWidth;
		int32_t mBaseHeight;
		int32_t mLvlNum;
		int32_t mLvlOffs[1];

		bool ck_lvl_idx(int idx) const { return (uint32_t)idx < (uint32_t)mLvlNum; }
		cxColor* get_lvl(int idx) const { return ck_lvl_idx(idx) ? reinterpret_cast<cxColor*>(XD_INCR_PTR(this, mLvlOffs[idx])) : nullptr; }
		void get_lvl_dims(int idx, int* pWidth, int* pHeight) const;
		DDS get_lvl_dds(int idx) const;
	};

	bool ck_plane_idx(int idx) const { return (uint32_t)idx < mPlaneNum; }
	int get_width() const { return mWidth; }
	int get_height() const { return mHeight; }
	int calc_mip_num() const;
	int find_plane_idx(const char* pName) const;
	Plane find_plane(const char* pName) const;
	PlaneInfo* get_plane_info(int idx) const { return ck_plane_idx(idx) ? reinterpret_cast<PlaneInfo*>(XD_INCR_PTR(this, mPlaneOffs)) + idx : nullptr; }
	Plane get_plane(int idx) const;
	void get_rgba(float* pDst, bool cvtToSRGB = false) const;
	cxColor* get_rgba(bool cvtToSRGB = false) const;
	DDS get_dds(bool cvtToSRGB = false) const;
	Pyramid* get_pyramid() const;

	static const uint32_t KIND = XD_FOURCC('X', 'T', 'E', 'X');
};

namespace nxTexture {

sxDDSHead* alloc_dds128(int w, int h, uint32_t* pSize);

} // nxTexture


struct sxKeyframesData : public sxData {
	float mFPS;
	int32_t mMinFrame;
	int32_t mMaxFrame;
	uint32_t mFCurveNum;
	uint32_t mFCurveOffs;
	uint32_t mNodeInfoNum;
	uint32_t mNodeInfoOffs;

	enum class eFunc {
		CONSTANT = 0,
		LINEAR = 1,
		CUBIC = 2
	};

	struct NodeInfo {
		int16_t mPathId;
		int16_t mNameId;
		int16_t mTypeId;
		uint8_t mRotOrd;
		uint8_t mXfmOrd;

		exRotOrd get_rot_order() const { return (exRotOrd)mRotOrd; }
		exTransformOrd get_xform_order() const { return (exTransformOrd)mXfmOrd; }
	};

	struct FCurveInfo {
		int16_t mNodePathId;
		int16_t mNodeNameId;
		int16_t mChanNameId;
		uint16_t mKeyNum;
		float mMinVal;
		float mMaxVal;
		uint32_t mValOffs;
		uint32_t mLSlopeOffs;
		uint32_t mRSlopeOffs;
		uint32_t mFnoOffs;
		uint32_t mFuncOffs;
		int8_t mCmnFunc;
		uint8_t mReserved8;
		uint16_t mReserved16;
		uint32_t mReserved32[2];

		bool is_const() const { return mKeyNum == 0; }
		bool has_fno_lst() const { return mFnoOffs != 0; }
		eFunc get_common_func() const { return mCmnFunc < 0 ? eFunc::LINEAR : (eFunc)mCmnFunc; }
	};

	class FCurve {
	private:
		const sxKeyframesData* mpKfr;
		int32_t mFcvId;

		FCurve() {}

		friend struct sxKeyframesData;

	public:
		bool is_valid() const { return mpKfr && mpKfr->ck_fcv_idx(mFcvId); }
		FCurveInfo* get_info() const { return is_valid() ? mpKfr->get_fcv_info(mFcvId) : nullptr; }
		const char* get_node_name() const { return is_valid() ? mpKfr->get_fcv_node_name(mFcvId) : nullptr; }
		const char* get_node_path() const { return is_valid() ? mpKfr->get_fcv_node_path(mFcvId) : nullptr; }
		const char* get_chan_name() const { return is_valid() ? mpKfr->get_fcv_chan_name(mFcvId) : nullptr; }
		bool is_const() const { return is_valid() ? get_info()->is_const() : true; }
		int get_key_num() const { return is_valid() ? get_info()->mKeyNum : 0; }
		bool ck_key_idx(int fno) const { return is_valid() ? (uint32_t)fno < (uint32_t)get_key_num() : false; }
		int find_key_idx(int fno) const;
		int get_fno(int idx) const;
		float eval(float frm, bool extrapolate = false) const;
	};

	bool has_node_info() const;
	bool ck_node_info_idx(int idx) const { return has_node_info() && ((uint32_t)idx < mNodeInfoNum); }
	int get_node_info_num() const { return has_node_info() ? mNodeInfoNum : 0; }
	int find_node_info_idx(const char* pName, const char* pPath = nullptr, int startIdx = 0) const;
	NodeInfo* get_node_info_ptr(int idx) const { return ck_node_info_idx(idx) ? reinterpret_cast<NodeInfo*>(XD_INCR_PTR(this, mNodeInfoOffs)) + idx : nullptr; }

	float get_frame_rate() const { return mFPS; }
	int get_frame_count() const { return get_max_fno() + 1; }
	int get_max_fno() const { return mMaxFrame - mMinFrame; }
	int get_rel_fno(int fno) const { return (fno - mMinFrame) % get_frame_count(); }
	bool ck_fno(int fno) const { return (uint32_t)fno <= (uint32_t)get_max_fno(); }
	bool ck_fcv_idx(int idx) const { return (uint32_t)idx < mFCurveNum; }
	int find_fcv_idx(const char* pNodeName, const char* pChanName, const char* pNodePath = nullptr) const;
	int get_fcv_num() const { return mFCurveNum; }
	FCurveInfo* get_fcv_top() const { return mFCurveOffs ? reinterpret_cast<FCurveInfo*>XD_INCR_PTR(this, mFCurveOffs) : nullptr; }
	FCurveInfo* get_fcv_info(int idx) const { return mFCurveOffs && (ck_fcv_idx(idx)) ? get_fcv_top() + idx : nullptr; }
	FCurve get_fcv(int idx) const;
	FCurve find_fcv(const char* pNodeName, const char* pChanName, const char* pNodePath = nullptr) const { return get_fcv(find_fcv_idx(pNodeName, pChanName, pNodePath)); }
	const char* get_fcv_node_name(int idx) const { return ck_fcv_idx(idx) ? get_str(get_fcv_info(idx)->mNodeNameId) : nullptr; }
	const char* get_fcv_node_path(int idx) const { return ck_fcv_idx(idx) ? get_str(get_fcv_info(idx)->mNodePathId) : nullptr; }
	const char* get_fcv_chan_name(int idx) const { return ck_fcv_idx(idx) ? get_str(get_fcv_info(idx)->mChanNameId) : nullptr; }

	void dump_clip(const char* pOutPath = nullptr) const;

	static const uint32_t KIND = XD_FOURCC('X', 'K', 'F', 'R');
};


struct sxCompiledExpression {
	uint32_t mSig;
	uint32_t mLen;
	uint32_t mValsNum;
	uint32_t mCodeNum;
	uint32_t mStrsNum;

	enum class eOp : uint8_t {
		NOP = 0,
		END = 1,
		NUM = 2,
		STR = 3,
		VAR = 4,
		CMP = 5,
		ADD = 6,
		SUB = 7,
		MUL = 8,
		DIV = 9,
		MOD = 10,
		NEG = 11,
		FUN = 12,
		XOR = 13,
		AND = 14,
		OR  = 15
	};

	enum class eCmp : uint8_t {
		EQ = 0,
		NE = 1,
		LT = 2,
		LE = 3,
		GT = 4,
		GE = 5
	};

	struct Code {
		uint8_t mOp;
		int8_t mPrio;
		int16_t mInfo;

		eOp get_op() const { return (eOp)mOp; }
	};

	struct StrInfo {
		uint32_t mHash;
		uint16_t mOffs;
		uint16_t mLen;
	};

	struct String {
		const char* mpChars;
		uint32_t mHash;
		uint32_t mLen;

		bool is_valid() const { return mpChars != nullptr; }
	};

	template<typename T> struct TagsT {
		T* mpBitAry;
		int mNum;

		TagsT() : mpBitAry(nullptr), mNum(0) {}

		void init(void* pBits, int n) {
			mpBitAry = (T*)pBits;
			mNum = n;
		}

		void reset() {
			mpBitAry = nullptr;
			mNum = 0;
		}

		bool ck_idx(int idx) const { return (uint32_t)idx < mNum; }
		void set_num(int idx) { XD_BIT_ARY_CL(T, mpBitAry, idx); }
		bool is_num(int idx) const { return !XD_BIT_ARY_CK(T, mpBitAry, idx); }
		void set_str(int idx) { XD_BIT_ARY_ST(T, mpBitAry, idx); }
		bool is_str(int idx) const { return XD_BIT_ARY_CK(T, mpBitAry, idx); }

		static int calc_mem_size(int n) { return XD_BIT_ARY_SIZE(T, n); }
	};

	typedef TagsT<uint8_t> Tags;

	class Stack {
	public:
		union Val {
			float num;
			int32_t sptr;
		};
	protected:
		void* mpMem;
		Val* mpVals;
		Tags mTags;
		int mPtr;

	public:
		Stack() : mpMem(nullptr), mpVals(nullptr), mPtr(0) {}

		void alloc(int n);
		void free();

		int size() const { return mTags.mNum; }
		void clear() { mPtr = 0; }
		bool is_empty() const { return mPtr == 0; }
		bool is_full() const { return mPtr >= size(); }
		void push_num(float num);
		float pop_num();
		void push_str(int ptr);
		int pop_str();
	};

	class ExecIfc {
	public:
		virtual Stack* get_stack() { return nullptr; }
		virtual void set_result(float val) {}
		virtual float ch(const String& path) { return 0.0f; }
		virtual float detail(const String& path, const String& attrName, int idx) { return 0.0f; }
		virtual float var(const String& name) { return 0.0f; }
	};

	bool is_valid() const { return mSig == XD_FOURCC('C', 'E', 'X', 'P') && mLen > 0; }
	const float* get_vals_top() const { return reinterpret_cast<const float*>(this + 1); }
	bool ck_val_idx(int idx) const { return (uint32_t)idx < mValsNum; }
	float get_val(int idx) const { return ck_val_idx(idx) ? get_vals_top()[idx] : 0.0f; }
	const Code* get_code_top() const { return reinterpret_cast<const Code*>(&get_vals_top()[mValsNum]); }
	bool ck_str_idx(int idx) const { return (uint32_t)idx < mStrsNum; }
	const StrInfo* get_str_info_top() const { return reinterpret_cast<const StrInfo*>(&get_code_top()[mCodeNum]); }
	const StrInfo* get_str_info(int idx) const { return ck_str_idx(idx) ? get_str_info_top() + idx : nullptr; }
	void get_str(int idx, String* pStr) const;
	String get_str(int idx) const;
	void exec(ExecIfc& ifc) const;
	void disasm(FILE* pFile = stdout) const;
};

struct sxExprLibData : public sxData {
	uint32_t mExprNum;
	uint32_t mListOffs;

	struct ExprInfo {
		int16_t mNodeNameId;
		int16_t mNodePathId;
		int16_t mChanNameId;
		int16_t mReserved;

		const sxCompiledExpression* get_compiled_expr() const { return reinterpret_cast<const sxCompiledExpression*>(this + 1); }
	};

	class Entry {
	private:
		const sxExprLibData* mpLib;
		int32_t mExprId;

		Entry() {}

		friend struct sxExprLibData;

	public:
		bool is_valid() const { return mpLib && mpLib->ck_expr_idx(mExprId); }
		const ExprInfo* get_info() const { return is_valid() ? mpLib->get_info(mExprId) : nullptr; }
		const sxCompiledExpression* get_expr() const { return is_valid() ? mpLib->get_info(mExprId)->get_compiled_expr() : nullptr; }
		const char* get_node_name() const { return is_valid() ? mpLib->get_str(get_info()->mNodeNameId) : nullptr; }
		const char* get_node_path() const { return is_valid() ? mpLib->get_str(get_info()->mNodePathId) : nullptr; }
		const char* get_chan_name() const { return is_valid() ? mpLib->get_str(get_info()->mChanNameId) : nullptr; }
	};

	bool ck_expr_idx(int idx) const { return (uint32_t)idx < mExprNum; }
	int get_expr_num() const { return mExprNum; }
	int find_expr_idx(const char* pNodeName, const char* pChanName, const char* pNodePath = nullptr, int startIdx = 0) const;
	const ExprInfo* get_info(int idx) const { return ck_expr_idx(idx) ? (const ExprInfo*)XD_INCR_PTR(this, ((uint32_t*)XD_INCR_PTR(this, mListOffs))[idx]) : nullptr; }
	Entry get_entry(int idx) const;
	int count_rig_entries(const sxRigData& rig) const;

	static const uint32_t KIND = XD_FOURCC('X', 'C', 'E', 'L');
};


namespace nxDataUtil {

exAnimChan anim_chan_from_str(const char* pStr);
const char* anim_chan_to_str(exAnimChan chan);
exRotOrd rot_ord_from_str(const char* pStr);
const char* rot_ord_to_str(exRotOrd rord);
exTransformOrd xform_ord_from_str(const char* pStr);
const char* xform_ord_to_str(exTransformOrd xord);

float* alloc_sh_coefs_f32(int order);
double* alloc_sh_coefs_f64(int order);

inline XMVECTOR f3_load_x0yz(const xt_float3& src) { return _mm_loadh_pi(_mm_load_ss(&src.x), (__m64 const*)(&src.y)); }
inline void f3_store_x0yz(xt_float3* pDst, XMVECTOR xv) { _mm_store_ss(&pDst->x, xv); _mm_storeh_pi((__m64*)(&pDst->y), xv); }
inline XMVECTOR f4_load(const xt_float4& src) { return _mm_loadu_ps(&src.x); }
inline void f4_store(xt_float4* pDst, XMVECTOR xv) { _mm_storeu_ps(&pDst->x, xv); }

} // nxDataUtil


namespace nxData {

sxData* load(const char* pPath);
void unload(sxData* pData);

} // nxData

