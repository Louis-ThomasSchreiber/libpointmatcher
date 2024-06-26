/*

Copyright (c) 2010--2018,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "MinSurfaceVariation.h"

/*
  This filter is based on this paper:
Chao Chuan Jia, Chuan Jiang Wang, Ting Yang, Bing Hui Fan, Fu Gui He,
A 3D Point Cloud Filtering Algorithm based on Surface Variation Factor Classification,
Procedia Computer Science, 2019
https://www.sciencedirect.com/science/article/pii/S1877050919307793

The summary is:

Suppose a point cloud representing a scan of a surface.
Given the covariance matrix of a subset of the point cloud
(the knn of a point for example), the eigenvectors of this matrix are
the principal axes of the subset and represents its overall orienation. The principal axis
associated with the smallest eigenvalue gives an approximation of
the normal of the surface. For a perfectly flat surface, the smallest eigenvalue
 will be zero and the associated eigenvector may be ill defined (singular).

In order to approximate the surface variation, one can use

sigma = lambda_0 / (lambda_0 + lambda_1 + lambda_2)

where lambda_0 is defined as the smallest eigenvalue.

The number of knn used to evaluate the eigenvalues will have an impact on the result, choose this value carefully.
  */

// Constructor
template <typename T>
MinSurfaceVariationDataPointsFilter<T>::MinSurfaceVariationDataPointsFilter(
    const Parameters &params)
    : PointMatcher<T>::DataPointsFilter(
          "MaxDensityDataPointsFilter",
          MinSurfaceVariationDataPointsFilter::availableParameters(), params),
      minSurfaceVariation(Parametrizable::get<T>("minSurfaceVariation")) {}

template <typename T>
typename PointMatcher<T>::DataPoints
MinSurfaceVariationDataPointsFilter<T>::filter(const DataPoints &input) {
  DataPoints output(input);
  inPlaceFilter(output);
  return output;
}

template <typename T>
void MinSurfaceVariationDataPointsFilter<T>::inPlaceFilter(DataPoints &cloud) {
  typedef typename DataPoints::View View;
  typedef typename PointMatcher<T>::Vector Vector;

  // Force eigValues to be computed
  if (!cloud.descriptorExists("eigValues")) {
    throw InvalidField(
        "MinSurfaceVariationDataPointsFilter: Error, no eigValues found "
        "in descriptors.");
  }

  const int nbPointsIn = cloud.features.cols();
  View eigValues = cloud.getDescriptorViewByName("eigValues");

  // eval surface variation end keep points with value above threshold:
  int j = 0;
  for (int i = 0; i < nbPointsIn; ++i) {

    const Vector& eigenValuesForPointI = eigValues.col(i);

    Eigen::Index minIndex;
    eigenValuesForPointI.minCoeff(&minIndex);

    const T surfaceVariation = eigenValuesForPointI[minIndex] / eigenValuesForPointI.sum();

    const bool keepPt = surfaceVariation >= minSurfaceVariation;

    if(keepPt) {
      cloud.setColFrom(j, cloud, i);
      ++j;
    }
  }

  cloud.conservativeResize(j);
}

template struct MinSurfaceVariationDataPointsFilter<float>;
template struct MinSurfaceVariationDataPointsFilter<double>;
