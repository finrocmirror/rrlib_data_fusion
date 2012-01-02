//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    test_program.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>

#include "rrlib/math/tPose2D.h"
#include "rrlib/util/join.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/functions.h"
#include "rrlib/data_fusion/factory.h"
#include "rrlib/data_fusion/channels.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::math;
using namespace rrlib::util;
using namespace rrlib::data_fusion;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

int main(int argc, char **argv)
{
  rrlib::logging::default_log_description = basename(argv[0]);

  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainConfiguresSubTree(".", true);
//  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainMaxMessageLevel(".", rrlib::logging::eLL_DEBUG_VERBOSE_3);
  rrlib::logging::tLogDomainRegistry::GetInstance()->SetDomainPrintsLocation(".", false);

  const size_t number_of_samples = 5;
  double keys[number_of_samples] = { 2, 5, 3, 3, 1 };

  {
    std::cout << "=== double values ===" << std::endl;

    double data[number_of_samples] = { 0.4, 0.1, 0.2, 0.5, 0.8 };
    double result = 0;

    result = FuseValuesUsingMaximumKey<double>(data, data + number_of_samples, keys, keys + number_of_samples);
    std::cout << "The maximum by keys of [ " << Join(data, data + number_of_samples) << " ] with keys [ " << Join(keys, keys + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, 0.1));

    result = FuseValuesUsingAverage<double>(data, data + number_of_samples);
    std::cout << "The average of [ " << Join(data, data + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, 0.4));

    result = FuseValuesUsingWeightedAverage<double>(data, data + number_of_samples, keys, keys + number_of_samples);
    std::cout << "The weighted average of [ " << Join(data, data + number_of_samples) << " ] with weights [ " << Join(keys, keys + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, 0.3));

    result = FuseValuesUsingMedianVoter<double>(data, data + number_of_samples);
    std::cout << "The median of [ " << Join(data, data + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, 0.4));

    result = FuseValuesUsingMedianKeyVoter<double>(data, data + number_of_samples, keys, keys + number_of_samples);
    std::cout << "The median by keys of [ " << Join(data, data + number_of_samples) << " ] with keys [ " << Join(keys, keys + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, 0.2) || IsEqual(result, 0.5));
  }

  {
    std::cout << "=== tPose2D ===" << std::endl;

    std::vector<tPose2D> data;
    data.push_back(tPose2D(0.4, 0.4, 0.4));
    data.push_back(tPose2D(0.1, 0.1, 0.1));
    data.push_back(tPose2D(0.2, 0.2, 0.2));
    data.push_back(tPose2D(0.5, 0.5, 0.5));
    data.push_back(tPose2D(0.8, 0.8, 0.8));
    tPose2D result;

    result = FuseValuesUsingMaximumKey<tPose2D>(data.begin(), data.end(), keys, keys + number_of_samples);
    std::cout << "The maximum by keys of [ " << Join(data.begin(), data.end()) << " ] with keys [ " << Join(keys, keys + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, tPose2D(0.1, 0.1, 0.1)));

    result = FuseValuesUsingAverage<tPose2D>(data.begin(), data.end());
    std::cout << "The average of [ " << Join(data.begin(), data.end()) << " ] is " << result << std::endl;
    assert(IsEqual(result, tPose2D(0.4, 0.4, 0.4)));

    result = FuseValuesUsingWeightedAverage<tPose2D>(data.begin(), data.end(), keys, keys + number_of_samples);
    std::cout << "The weighted average of [ " << Join(data.begin(), data.end()) << " ] with weights [ " << Join(keys, keys + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, tPose2D(0.3, 0.3, 0.3)));

    result = FuseValuesUsingMedianVoter<tPose2D>(data.begin(), data.end());
    std::cout << "The median of [ " << Join(data.begin(), data.end()) << " ] is " << result << std::endl;
    assert(IsEqual(result, tPose2D(0.4, 0.4, 0.4)));

    result = FuseValuesUsingMedianKeyVoter<tPose2D>(data.begin(), data.end(), keys, keys + number_of_samples);
    std::cout << "The median by keys of [ " << Join(data.begin(), data.end()) << " ] with keys [ " << Join(keys, keys + number_of_samples) << " ] is " << result << std::endl;
    assert(IsEqual(result, tPose2D(0.2, 0.2, 0.2)) || IsEqual(result, tPose2D(0.5, 0.5, 0.5)));
  }

  InitializeFactory<tPose2D>();

  tDataFusion<tPose2D> *fusion = rrlib::data_fusion::tDataFusionFactory<tPose2D>::GetInstance().Create("Average");
  delete fusion;

  tAverage<double, channel::Average> cyclic_fusion;

  std::cout << "OK" << std::endl;

  return EXIT_SUCCESS;
}
