// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
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

#include "pointmatcher/PointMatcher.h"
#include <iostream>

using namespace std;
using namespace PointMatcherSupport;

void validateArgs(int argc, char *argv[], bool &isCSV);

/**
 * Code example for filtering a point cloud from a file using SurfaceNormalDataPointsFilter
 */
int main(int argc, char *argv[]) {
  bool isCSV = true;
  validateArgs(argc, argv, isCSV);

  typedef PointMatcher<float> PM;
  typedef PM::DataPoints DP;

  // Load point clouds
  const DP ref(DP::load(argv[1]));

  // Test filter
  std::shared_ptr<PM::DataPointsFilter> densityFilter =
      PM::get().DataPointsFilterRegistrar.create(
          "SurfaceNormalDataPointsFilter", {{"knn", "10"},
                                            {"epsilon", "5"},
                                            {"keepNormals", "0"},
                                            {"keepEigenValues", "1"},
                                            {"keepDensities", "1"}});

  std::shared_ptr<PM::DataPointsFilter> minSurfaceVariationSubsample =
      PM::get().DataPointsFilterRegistrar.create(
          "MinSurfaceVariationDataPointsFilter", {{"minSurfaceVariation", toParam(0.002)}});

  const DP refWithDensity = densityFilter->filter(ref);
  const DP refFilteredWithSurfaceVariation = minSurfaceVariationSubsample->filter(refWithDensity);
  refWithDensity.save("refWithDensity.ply");
  refFilteredWithSurfaceVariation.save("refFilteredWithSurfaceVariation.ply");

  return 0;
}

void validateArgs(int argc, char *argv[], bool &isCSV) {
  if (argc != 2) {
    cerr << "Wrong number of arguments in " << argv[0] << endl <<"Please provide one point cloud file" << endl;
    exit(1);
  }
}
