//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>

#include <vector>
#include <string>

#include "lib.h"
#include "Recast.h"
#include "InputGeom.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"
#include "DetourNavMesh.h"
#include "DetourCrowd.h"
#include "DetourDebugDraw.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourNode.h"

#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

static const int MAX_POLYS = 256;

dtQueryFilter m_filter;

struct NavMeshContext {
	Sample* sample;
	InputGeom* geom;
};

BuildNavMeshResult buildNavMesh(char* buf, long bufSize)
{
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	BuildContext ctx;
	BuildNavMeshResult result;
	result.success = 0;

	Sample* sample = new Sample_SoloMesh;
	sample->setContext(&ctx);

	InputGeom* geom = new InputGeom;

	if (!geom->load(&ctx, buf, bufSize))
	{
		delete geom;
		geom = 0;

		// Destroy the sample if it already had geometry loaded, as we've just deleted it!
		if (sample && sample->getInputGeom())
		{
			delete sample;
			sample = 0;
		}

		ctx.dumpLog("Geom load log:");
	}

	if (sample && geom)
	{
		sample->handleMeshChanged(geom);
		sample->handleBuild();
	}

	auto navmesh = sample->getNavMesh();
	if (!navmesh) {
		ctx.dumpLog("Error creating navmesh.");
		return result;
	}

	result.success = 1;
	auto navMeshContext = new NavMeshContext;
	navMeshContext->sample = sample;
	navMeshContext->geom = geom;
	result.navMeshContext = navMeshContext;

	return result;
}

FindPathResult findPath(float startX, float startY, float startZ, float endX, float endY, float endZ, void* navMeshContext)
{
	FindPathResult result;

	NavMeshContext* ctx = (NavMeshContext*)navMeshContext;

	auto sample = ctx->sample;

	dtNavMeshQuery *m_navQuery = sample->getNavMeshQuery();

	const float extent[3] = { 0.01f, 1.5f, 0.01f };
	const float start[3] = { startX, startY, startZ };
	const float end[3] = { endX, endY, endZ };

	dtPolyRef startPolyRef;
	float startPolyPoint[3];
	dtPolyRef endPolyRef;
	float endPolyPoint[3];

	m_navQuery->findNearestPoly(start, extent, &m_filter, &startPolyRef, startPolyPoint);
	m_navQuery->findNearestPoly(end, extent, &m_filter, &endPolyRef, endPolyPoint);

	int pathCount;
	dtPolyRef m_polys[MAX_POLYS];
	m_navQuery->findPath(startPolyRef, endPolyRef, start, end, &m_filter, m_polys, &pathCount, MAX_POLYS);

	float *straightPath = new float[MAX_POLYS * 3];
	dtPolyRef pathRef[MAX_POLYS];
	int straightPathCount;

	m_navQuery->findStraightPath(start, end, m_polys, pathCount, straightPath, 0, pathRef, &straightPathCount, MAX_POLYS);

	result.success = 1;
	result.points = straightPath;
	result.length = straightPathCount;

	return result;
}

void freeFloatArray(float *arr)
{
	delete arr;
}

void freeNavMeshContext(void* navMeshContext)
{
	NavMeshContext* ctx = (NavMeshContext*)navMeshContext;
	delete ctx;
}
