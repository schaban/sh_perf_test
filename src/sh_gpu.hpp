#include "gpu/def.h"

struct GeomIn {
	xt_float3 nrm;
};

struct ClrOut {
	xt_float3 clr;
};

struct cbSH {
	xt_float4 sh[D_GPU_SH_MAX * D_GPU_SH_MAX]; // rgb.w
};

struct sGPUCompute {
	sxGfxDevice mDev;
	ID3D11ClassLinkage* mpLinkage;
	ID3D11ClassInstance* mpSHInst[D_GPU_SH_MAX+1];
	ID3D11ComputeShader* mpSHApply;
	ID3D11Buffer* mpGeomIn;
	ID3D11Buffer* mpClrOut;
	ID3D11ShaderResourceView* mpGeomSRV;
	ID3D11UnorderedAccessView* mpClrUAV;
	ID3D11Buffer* mpGeomReadBuf;
	ID3D11Buffer* mpClrReadBuf;
	ID3D11Buffer* mpSHCB;
	cbSH mSH;

	static const int BLK_SIZE = 1024*32;

	sGPUCompute()
	:
	mpLinkage(nullptr), mpSHApply(nullptr),
	mpGeomIn(nullptr), mpClrOut(nullptr), mpGeomSRV(nullptr), mpClrUAV(nullptr), mpSHCB(nullptr),
	mpGeomReadBuf(nullptr), mpClrReadBuf(nullptr)
	{ for (int i = 0; i < XD_ARY_LEN(mpSHInst); ++i) { mpSHInst[i] = nullptr; } }

	void init();
	void reset();

	bool is_valid() const { return mDev.is_valid() && mpSHApply; }

	void set_sh(int order, const float* pR, const float* pG, const float* pB, const float* pWgt);

	ID3D11Buffer* create_buffer(size_t elemSize, int elemCount);
	ID3D11Buffer* create_cpuread_buffer(ID3D11Buffer* pSrcBuf);
	ID3D11ShaderResourceView* create_buf_srv(ID3D11Buffer* pBuf);
	ID3D11UnorderedAccessView* create_buf_uav(ID3D11Buffer* pBuf);
	ID3D11Buffer* create_cb(size_t size);
	void copy_geom_in();
	void copy_clr_out();
	void update_geom_in(const GeomIn* pSrc);
	void exec_shapply(int order, int wksize = BLK_SIZE);
};
