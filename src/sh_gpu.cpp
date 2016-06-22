#include "xcore.hpp"
#include "xcalc.hpp"
#include "xdata.hpp"

#include "sh_gpu.hpp"
#include "gpu/def.h"
#include "gpu/cs_shapply.h"

void sGPUCompute::init() {
	mDev = nxCore::create_def_gfx_device();
	if (!mDev.is_valid()) return;
	if (!mDev.is_dx11_level()) {
		mDev.relese();
		return;
	}
	HRESULT hres = mDev.mpDev->CreateComputeShader(cs_shapply, sizeof(cs_shapply), nullptr, &mpSHApply);
	if (FAILED(hres)) {
		return;
	}
	mpGeomIn = create_buffer(sizeof(GeomIn), BLK_SIZE);
	mpClrOut = create_buffer(sizeof(ClrOut), BLK_SIZE);
	mpGeomSRV = create_buf_srv(mpGeomIn);
	mpClrUAV = create_buf_uav(mpClrOut);
	mpSHCB = create_cb(sizeof(cbSH));
	mpGeomReadBuf = create_cpuread_buffer(mpGeomIn);
	mpClrReadBuf = create_cpuread_buffer(mpClrOut);
}

void sGPUCompute::reset() {
	if (mDev.is_valid()) {
		mDev.mpCtx->Flush();
		mDev.mpCtx->ClearState();
	}
	if (mpGeomReadBuf) {
		mpGeomReadBuf->Release();
		mpGeomReadBuf = nullptr;
	}
	if (mpClrReadBuf) {
		mpClrReadBuf->Release();
		mpClrReadBuf = nullptr;
	}
	if (mpGeomSRV) {
		mpGeomSRV->Release();
		mpGeomSRV = nullptr;
	}
	if (mpGeomIn) {
		mpGeomIn->Release();
		mpGeomIn = nullptr;
	}
	if (mpClrUAV) {
		mpClrUAV->Release();
		mpClrUAV = nullptr;
	}
	if (mpClrOut) {
		mpClrOut->Release();
		mpClrOut = nullptr;
	}
	if (mpSHCB) {
		mpSHCB->Release();
		mpSHCB = nullptr;
	}
	if (mpSHApply) {
		mpSHApply->Release();
		mpSHApply = nullptr;
	}
	mDev.relese();
}

void sGPUCompute::set_sh(const float* pR, const float* pG, const float* pB, const float* pWgt) {
	for (int i = 0; i < 8 * 8; ++i) {
		mSH.sh8[i].set(pR[i], pG[i], pB[i], 0.0f);
	}
	for (int i = 0; i < 8; ++i) {
		mSH.sh8[i].w = pWgt[i];
	}
	if (mDev.is_valid() && mpSHCB) {
		D3D11_MAPPED_SUBRESOURCE map;
		HRESULT hres = mDev.mpCtx->Map(mpSHCB, 0, D3D11_MAP_WRITE_DISCARD, 0, &map);
		if (SUCCEEDED(hres)) {
			::memcpy(map.pData, &mSH, sizeof(cbSH));
			mDev.mpCtx->Unmap(mpSHCB, 0);
		}
	}
}

ID3D11Buffer* sGPUCompute::create_buffer(size_t elemSize, int elemCount) {
	ID3D11Buffer* pBuf = nullptr;
	if (mDev.is_valid()) {
		D3D11_BUFFER_DESC dsc;
		::ZeroMemory(&dsc, sizeof(D3D11_BUFFER_DESC));
		dsc.BindFlags = D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE;
		dsc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
		dsc.ByteWidth = (UINT)(elemSize*elemCount);
		dsc.StructureByteStride = (UINT)elemSize;
		mDev.mpDev->CreateBuffer(&dsc, nullptr, &pBuf);
	}
	return pBuf;
}

ID3D11Buffer* sGPUCompute::create_cpuread_buffer(ID3D11Buffer* pSrcBuf) {
	ID3D11Buffer* pReadBuf = nullptr;
	if (mDev.is_valid()) {
		D3D11_BUFFER_DESC dsc;
		::ZeroMemory(&dsc, sizeof(D3D11_BUFFER_DESC));
		pSrcBuf->GetDesc(&dsc);
		dsc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		dsc.Usage = D3D11_USAGE_STAGING;
		dsc.BindFlags = 0;
		dsc.MiscFlags = 0;
		mDev.mpDev->CreateBuffer(&dsc, nullptr, &pReadBuf);
	}
	return pReadBuf;
}

ID3D11ShaderResourceView* sGPUCompute::create_buf_srv(ID3D11Buffer* pBuf) {
	ID3D11ShaderResourceView* pSRV = nullptr;
	if (mDev.is_valid() && pBuf) {
		D3D11_BUFFER_DESC bufDsc;
		pBuf->GetDesc(&bufDsc);
		D3D11_SHADER_RESOURCE_VIEW_DESC dsc;
		::ZeroMemory(&dsc, sizeof(D3D11_SHADER_RESOURCE_VIEW_DESC));
		dsc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
		dsc.Format = DXGI_FORMAT_UNKNOWN;
		dsc.BufferEx.NumElements = bufDsc.ByteWidth / bufDsc.StructureByteStride;
		mDev.mpDev->CreateShaderResourceView(pBuf, &dsc, &pSRV);
	}
	return pSRV;
}

ID3D11UnorderedAccessView* sGPUCompute::create_buf_uav(ID3D11Buffer* pBuf) {
	ID3D11UnorderedAccessView* pUAV = nullptr;
	if (mDev.is_valid() && pBuf) {
		D3D11_BUFFER_DESC bufDsc;
		pBuf->GetDesc(&bufDsc);
		D3D11_UNORDERED_ACCESS_VIEW_DESC dsc;
		::ZeroMemory(&dsc, sizeof(D3D11_UNORDERED_ACCESS_VIEW_DESC));
		dsc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
		dsc.Format = DXGI_FORMAT_UNKNOWN;
		dsc.Buffer.NumElements = bufDsc.ByteWidth / bufDsc.StructureByteStride;
		mDev.mpDev->CreateUnorderedAccessView(pBuf, &dsc, &pUAV);
	}
	return pUAV;
}

ID3D11Buffer* sGPUCompute::create_cb(size_t size) {
	ID3D11Buffer* pCB = nullptr;
	if (mDev.is_valid()) {
		D3D11_BUFFER_DESC dsc;
		::ZeroMemory(&dsc, sizeof(D3D11_BUFFER_DESC));
		dsc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		dsc.Usage = D3D11_USAGE_DYNAMIC;
		dsc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		dsc.ByteWidth = (UINT)size;
		mDev.mpDev->CreateBuffer(&dsc, nullptr, &pCB);
	}
	return pCB;
}

void sGPUCompute::copy_geom_in() {
	if (mDev.is_valid() && mpGeomIn && mpGeomReadBuf) {
		mDev.mpCtx->CopyResource(mpGeomReadBuf, mpGeomIn);
	}
}

void sGPUCompute::copy_clr_out() {
	if (mDev.is_valid() && mpClrOut && mpClrReadBuf) {
		mDev.mpCtx->CopyResource(mpClrReadBuf, mpClrOut);
	}
}

void sGPUCompute::update_geom_in(const GeomIn* pSrc) {
	mDev.mpCtx->UpdateSubresource(mpGeomIn, 0, nullptr, pSrc, 0, 0);
}

void sGPUCompute::exec_shapply(int wksize) {
	mDev.mpCtx->CSSetShaderResources(0, 1, &mpGeomSRV);
	mDev.mpCtx->CSSetUnorderedAccessViews(0, 1, &mpClrUAV, nullptr);
	mDev.mpCtx->CSSetConstantBuffers(0, 1, &mpSHCB);
	mDev.mpCtx->CSSetShader(mpSHApply, nullptr, 0);
	int ngrp = (wksize / D_GPU_THREADS_NUM) + ((wksize % D_GPU_THREADS_NUM) != 0);
	mDev.mpCtx->Dispatch(ngrp, 1, 1);
}

