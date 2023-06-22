#include "cloud_processing/models/cloud_filter/no_filter.h"

namespace cloud_processing {

NoFilter::NoFilter() {}

bool NoFilter::Filter(const CloudData::CloudTypePtr& input_cloud_ptr,
                      CloudData::CloudTypePtr& filtered_cloud_ptr) {
  filtered_cloud_ptr.reset(new CloudData::CloudType(*input_cloud_ptr));
  return true;
}

}  // namespace cloud_processing
