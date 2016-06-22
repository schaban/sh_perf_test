struct GeomIn {
	xt_float3 nrm;
};

struct ClrOut {
	xt_float3 clr;
};

struct cbSH {
	xt_float4 sh8[8 * 8]; // rgb.w
};

struct sGPUCompute {
	sxGfxDevice mDev;
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
	mpSHApply(nullptr),
	mpGeomIn(nullptr), mpClrOut(nullptr), mpGeomSRV(nullptr), mpClrUAV(nullptr), mpSHCB(nullptr),
	mpGeomReadBuf(nullptr), mpClrReadBuf(nullptr)
	{}

	void init();
	void reset();

	bool is_valid() const { return mDev.is_valid() && mpSHApply; }

	void set_sh(const float* pR, const float* pG, const float* pB, const float* pWgt);

	ID3D11Buffer* create_buffer(size_t elemSize, int elemCount);
	ID3D11Buffer* create_cpuread_buffer(ID3D11Buffer* pSrcBuf);
	ID3D11ShaderResourceView* create_buf_srv(ID3D11Buffer* pBuf);
	ID3D11UnorderedAccessView* create_buf_uav(ID3D11Buffer* pBuf);
	ID3D11Buffer* create_cb(size_t size);
	void copy_geom_in();
	void copy_clr_out();
	void update_geom_in(const GeomIn* pSrc);
	void exec_shapply(int wksize = BLK_SIZE);
};
