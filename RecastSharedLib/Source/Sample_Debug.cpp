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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Sample_Debug.h"
#include "InputGeom.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "RecastDump.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

/*
static int loadBin(const char* path, unsigned char** data)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;
	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	*data = new unsigned char[size];
	fread(*data, size, 1, fp);
	fclose(fp);
	return size;
} 
*/

Sample_Debug::Sample_Debug() :
	m_chf(0),
	m_cset(0),
	m_pmesh(0)
{
	resetCommonSettings();

	// Test
/*	m_chf = rcAllocCompactHeightfield();
	FileIO io;
	if (!io.openForRead("test.chf"))
	{
		delete m_chf;
		m_chf = 0;
	}
	else
	{
		if (!duReadCompactHeightfield(*m_chf, &io))
		{
			delete m_chf;
			m_chf = 0;
		}
	}*/
	
/*	if (m_chf)
	{
		unsigned short ymin = 0xffff;
		unsigned short ymax = 0;
		for (int i = 0; i < m_chf->spanCount; ++i)
		{
			const rcCompactSpan& s = m_chf->spans[i];
			if (s.y < ymin) ymin = s.y;
			if (s.y > ymax) ymax = s.y;
		}
		printf("ymin=%d ymax=%d\n", (int)ymin, (int)ymax);
		
		int maxSpans = 0;
		for (int i = 0; i < m_chf->width*m_chf->height; ++i)
		{
			maxSpans = rcMax(maxSpans, (int)m_chf->cells[i].count);
		}
		printf("maxSpans = %d\n", maxSpans);
	}*/
	

/*	const float orig[3] = {0,0,0};
	m_navMesh = new dtNavMesh;
	m_navMesh->init(orig, 133.333f,133.333f, 2048, 4096, 4096);

	unsigned char* data = 0;
	int dataSize = 0;
	
	// Tile_-13_-14.bin is basically just the bytes that was output by Detour. It should be loaded at X: -13 and Y: -14.
	
	dataSize = loadBin("Tile_-13_-13.bin", &data);
	if (dataSize > 0)
	{
		m_navMesh->addTileAt(-13,-13, data, dataSize, true);
		dtMeshHeader* header = (dtMeshHeader*)data;
		vcopy(m_bmin, header->bmin);
		vcopy(m_bmax, header->bmax);
	}

	dataSize = loadBin("Tile_-13_-14.bin", &data);
	if (dataSize > 0)
	{
		m_navMesh->addTileAt(-13,-14, data, dataSize, true);
	}

	dataSize = loadBin("Tile_-14_-14.bin", &data);
	if (dataSize > 0)
	{
		m_navMesh->addTileAt(-14,-14, data, dataSize, true);
	}
	
	const float halfExtents[3] = {40,100,40};
	const float center[3] = { -1667.9491f, 135.52649f, -1680.6149f };
	dtQueryFilter filter;
	m_ref = m_navMesh->findNearestPoly(center, halfExtents, &filter, 0);

	vcopy(m_halfExtents, halfExtents);
	vcopy(m_center, center);*/
	

	{
		m_cset = rcAllocContourSet();
		if (m_cset)
		{
			FileIO io;
			if (io.openForRead("PathSet_TMP_NA_PathingTestAReg1_1_2_CS.rc"))
			{
				duReadContourSet(*m_cset, &io);
				
				printf("bmin=(%f,%f,%f) bmax=(%f,%f,%f)\n",
					   m_cset->bmin[0], m_cset->bmin[1], m_cset->bmin[2],
					   m_cset->bmax[0], m_cset->bmax[1], m_cset->bmax[2]);
				printf("cs=%f ch=%f\n", m_cset->cs, m_cset->ch);
			}
			else
			{
				printf("could not open test.cset\n");
			}
		}
		else
		{
			printf("Could not alloc cset\n");
		}


/*		if (m_cset)
		{
			m_pmesh = rcAllocPolyMesh();
			if (m_pmesh)
			{
				rcBuildPolyMesh(m_ctx, *m_cset, 6, *m_pmesh);
			}
		}*/
	}
	
}

Sample_Debug::~Sample_Debug()
{
	rcFreeCompactHeightfield(m_chf);
	rcFreeContourSet(m_cset);
	rcFreePolyMesh(m_pmesh);
}

void Sample_Debug::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;
}

const float* Sample_Debug::getBoundsMin()
{
	if (m_cset)
		return m_cset->bmin;
	if (m_chf)
		return m_chf->bmin;
	if (m_navMesh)
		return m_bmin;
	return 0;
}

const float* Sample_Debug::getBoundsMax()
{
	if (m_cset)
		return m_cset->bmax;
	if (m_chf)
		return m_chf->bmax;
	if (m_navMesh)
		return m_bmax;
	return 0;
}

void Sample_Debug::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample_Debug::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

bool Sample_Debug::handleBuild()
{

	if (m_chf)
	{
		rcFreeContourSet(m_cset);
		m_cset = 0;
		
		// Create contours.
		m_cset = rcAllocContourSet();
		if (!m_cset)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
			return false;
		}
		if (!rcBuildContours(m_ctx, *m_chf, /*m_cfg.maxSimplificationError*/1.3f, /*m_cfg.maxEdgeLen*/12, *m_cset))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
			return false;
		}
	}
		
	return true;
}
