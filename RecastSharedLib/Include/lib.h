#pragma once

extern "C" {
	struct FindPathResult {
		int success;
		float *points;
		int length;
	};

	struct BuildNavMeshResult {
		int success;
		void* navMeshContext;
	};

	__declspec(dllexport) BuildNavMeshResult buildNavMesh(char* buf, long bufsSize);
	__declspec(dllexport) FindPathResult findPath(float startX, float startY, float startZ, float endX, float endY, float endZ, void* navMeshContext);
	__declspec(dllexport) void freeFloatArray(float *);
	__declspec(dllexport) void freeNavMeshContext(void* navMeshContext);
}
