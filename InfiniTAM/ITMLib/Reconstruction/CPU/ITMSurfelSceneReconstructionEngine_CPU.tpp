// Copyright 2014-2015 Isis Innovation Limited and the authors of InfiniTAM

#include "ITMSurfelSceneReconstructionEngine_CPU.h"

#include "../Shared/ITMSurfelSceneReconstructionEngine_Shared.h"

namespace ITMLib
{

//#################### CONSTRUCTORS ####################

template <typename TSurfel>
ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::ITMSurfelSceneReconstructionEngine_CPU(const Vector2i& depthImageSize)
: ITMSurfelSceneReconstructionEngine<TSurfel>(depthImageSize)
{}

//#################### PRIVATE MEMBER FUNCTIONS ####################

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::AddNewSurfels(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  // Calculate the prefix sum of the new points mask.
  const unsigned short *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU);
  unsigned int *newPointsPrefixSum = this->m_newPointsPrefixSumMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(this->m_newPointsMaskMB->dataSize - 1);

  newPointsPrefixSum[0] = 0;
  for(int i = 1; i <= pixelCount; ++i)
  {
    newPointsPrefixSum[i] = newPointsPrefixSum[i-1] + newPointsMask[i-1];
  }

  // Add the new surfels to the scene.
  const size_t newSurfelCount = static_cast<size_t>(newPointsPrefixSum[pixelCount]);
  TSurfel *newSurfels = scene->AllocateSurfels(newSurfelCount);
  if(newSurfels == NULL) return;

  const Vector4u *colourMap = view->rgb->GetData(MEMORYDEVICE_CPU);
  const unsigned int *correspondenceMap = this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CPU);
  const Matrix4f& depthToRGB = view->calib->trafo_rgb_to_depth.calib_inv;
  const Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const Vector4f& projParamsRGB = view->calib->intrinsics_rgb.projectionParamsSimple.all;
  const float *radiusMap = this->m_radiusMapMB->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);
  const Matrix4f T = trackingState->pose_d->GetInvM();
  const Vector3f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    add_new_surfel(locId, T, newPointsMask, newPointsPrefixSum, vertexMap, normalMap, radiusMap, colourMap, this->m_timestamp, newSurfels, surfels, correspondenceMap, view->rgb->noDims.x, view->rgb->noDims.y, depthToRGB, projParamsRGB);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::FindCorrespondingSurfels(const ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState,
                                                                               const ITMSurfelRenderState *renderState) const
{
  unsigned int *correspondenceMap = this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CPU);
  const float *depthMap = view->depth->GetData(MEMORYDEVICE_CPU);
  const int depthMapWidth = view->depth->noDims.x;
  const unsigned int *indexImageSuper = renderState->GetIndexImageSuper()->GetData(MEMORYDEVICE_CPU);
  const Matrix4f& invT = trackingState->pose_d->GetM();
  unsigned short *newPointsMask = this->m_newPointsMaskMB->GetData(MEMORYDEVICE_CPU);
  const Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    find_corresponding_surfel(locId, invT, depthMap, depthMapWidth, normalMap, indexImageSuper, surfels, correspondenceMap, newPointsMask);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::FuseMatchedPoints(ITMSurfelScene<TSurfel> *scene, const ITMView *view, const ITMTrackingState *trackingState) const
{
  const Vector4u *colourMap = view->rgb->GetData(MEMORYDEVICE_CPU);
  const int colourMapHeight = view->rgb->noDims.y;
  const int colourMapWidth = view->rgb->noDims.x;
  const unsigned int *correspondenceMap = this->m_correspondenceMapMB->GetData(MEMORYDEVICE_CPU);
  const Matrix4f& depthToRGB = view->calib->trafo_rgb_to_depth.calib_inv;
  const Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const Vector4f& projParamsRGB = view->calib->intrinsics_rgb.projectionParamsSimple.all;
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);
  const float *radiusMap = this->m_radiusMapMB->GetData(MEMORYDEVICE_CPU);
  const Matrix4f T = trackingState->pose_d->GetInvM();
  const Vector3f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);

#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    fuse_matched_point(locId, correspondenceMap, T, vertexMap, normalMap, radiusMap, colourMap, this->m_timestamp, surfels, colourMapWidth, colourMapHeight, depthToRGB, projParamsRGB);
  }
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::PreprocessDepthMap(const ITMView *view) const
{
  const float *depthMap = view->depth->GetData(MEMORYDEVICE_CPU);
  const int height = view->depth->noDims.y;
  const ITMIntrinsics& intrinsics = view->calib->intrinsics_d;
  Vector3f *normalMap = this->m_normalMapMB->GetData(MEMORYDEVICE_CPU);
  const int pixelCount = static_cast<int>(view->depth->dataSize);
  Vector3f *vertexMap = this->m_vertexMapMB->GetData(MEMORYDEVICE_CPU);
  const int width = view->depth->noDims.x;

  // Calculate the vertex map.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    calculate_vertex_position(locId, width, intrinsics, depthMap, vertexMap);
  }

  // Calculate the normal map.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int locId = 0; locId < pixelCount; ++locId)
  {
    calculate_normal(locId, vertexMap, width, height, normalMap);
  }

  // TODO: Calculate the radius map.
}

template <typename TSurfel>
void ITMSurfelSceneReconstructionEngine_CPU<TSurfel>::RemoveBadSurfels(ITMSurfelScene<TSurfel> *scene) const
{
  const int surfelCount = static_cast<int>(scene->GetSurfelCount());
  unsigned int *surfelRemovalMask = this->m_surfelRemovalMaskMB->GetData(MEMORYDEVICE_CPU);
  const TSurfel *surfels = scene->GetSurfels()->GetData(MEMORYDEVICE_CPU);

  // Clear the surfel removal mask.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    clear_removal_mask(surfelId, surfelRemovalMask);
  }

  // Mark long-term unstable surfels for removal.
#ifdef WITH_OPENMP
  #pragma omp parallel for
#endif
  for(int surfelId = 0; surfelId < surfelCount; ++surfelId)
  {
    mark_for_removal_if_unstable(surfelId, surfels, this->m_timestamp, surfelRemovalMask);
  }

  // Remove marked surfels from the scene.
  // TODO
}

}
