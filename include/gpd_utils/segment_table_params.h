/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef SEGMENT_TABLE_PARAMS_H
#define SEGMENT_TABLE_PARAMS_H

#include <ariles/adapters_all.h>
#include <ariles/ariles_all.h>
#include <ariles/ariles.h>

namespace pal {

struct PlanarSegmentationParams : public ariles::ConfigurableBase
{
  PlanarSegmentationParams() = default;

  void setDefaults() override
  {
  }
#define ARILES_SECTION_ID "PlanarSegmentationParams"
#define ARILES_CONSTRUCTOR PlanarSegmentationParams
#define ARILES_ENTRIES                                                                   \
  ARILES_ENTRY_(rate)                                                                    \
  ARILES_ENTRY_(processing_frame)                                                        \
  ARILES_ENTRY_(passthrough_zmin)                                                        \
  ARILES_ENTRY_(passthrough_zmax)                                                        \
  ARILES_ENTRY_(passthrough_xmin)                                                        \
  ARILES_ENTRY_(passthrough_xmax)                                                        \
  ARILES_ENTRY_(downsampling_size)
#include ARILES_INITIALIZE

  double rate_;
  std::string processing_frame_;
  double passthrough_zmin_;
  double passthrough_zmax_;
  double passthrough_xmin_;
  double passthrough_xmax_;
  double downsampling_size_;
};

}
#endif // SEGMENT_TABLE_PARAMS_H
