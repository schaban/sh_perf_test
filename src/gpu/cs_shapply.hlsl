#include "def.h"

struct GeomIn {
	float3 nrm;
};

struct ClrOut {
	float3 clr;
};

StructuredBuffer<GeomIn> in_geom : register(t0);
RWStructuredBuffer<ClrOut> out_clr : register(u0);

cbuffer cbSH : register(cb0) {
	float4 g_sh[8*8];
};

#include "sheval.h"

[numthreads(D_GPU_THREADS_NUM, 1, 1)]
void main(uint3 dtid : SV_DispatchThreadID, uint3 gid : SV_GroupID) {
	int idx = dtid.x;
	float3 nrm = in_geom[idx].nrm;
	float3 clr;
	const int order = 8;
	const int ncoef = order*order;
	float3 shc[ncoef];
	float shw[order];
	shGet8(shc, shw);
#if 1
	shApply8(clr, shc, shw, nrm.x, nrm.y, nrm.z);
#else
	float sh[ncoef];
	shEval8(sh, nrm.x, nrm.y, nrm.z);
	shWgtDot8(clr, shc, sh, shw);
#endif
	out_clr[idx].clr = clr;
}
