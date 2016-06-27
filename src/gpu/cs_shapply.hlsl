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
	float4 g_sh[D_GPU_SH_MAX*D_GPU_SH_MAX];
};

#include "sheval.h"

interface iSH {
	float3 exec(float3 nrm);
};

#define SH_IMPL(order) class cSH##order : iSH { \
	float3 exec(float3 nrm) { \
		float3 clr; \
		float3 shc[order*order]; \
		float shw[order]; \
		shGet##order(shc, shw); \
		shApply##order(clr, shc, shw, nrm.x, nrm.y, nrm.z); \
		return clr; \
	}};

SH_IMPL(2)
SH_IMPL(3)
SH_IMPL(4)
SH_IMPL(5)
SH_IMPL(6)
SH_IMPL(7)
SH_IMPL(8)
SH_IMPL(9)
SH_IMPL(10)
SH_IMPL(11)
SH_IMPL(12)
SH_IMPL(13)
SH_IMPL(14)
SH_IMPL(15)
SH_IMPL(16)
SH_IMPL(17)
SH_IMPL(18)
SH_IMPL(19)

iSH i_SH;

[numthreads(D_GPU_THREADS_NUM, 1, 1)]
void main(uint3 dtid : SV_DispatchThreadID, uint3 gid : SV_GroupID) {
	int idx = dtid.x;
	float3 nrm = in_geom[idx].nrm;
	out_clr[idx].clr = i_SH.exec(nrm);
}
