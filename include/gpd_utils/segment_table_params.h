/*
  @file
  
  @author victor
  
  @copyright (c) 2019 PAL Robotics SL. All Rights Reserved
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
