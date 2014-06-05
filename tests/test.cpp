//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    data_fusion/tests/test.cpp
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
#include "rrlib/util/tUnitTestSuite.h"

#include "rrlib/data_fusion/functions.h"
#include "rrlib/data_fusion/factory.h"
#include "rrlib/data_fusion/channels.h"

#include "rrlib/math/tPose2D.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace data_fusion
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
const size_t cNUMBER_OF_SAMPLES = 5;
const double keys[cNUMBER_OF_SAMPLES] = { 2, 5, 3, 3, 1 };

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------
class Test : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(Test);
  RRLIB_UNIT_TESTS_ADD_TEST(Double);
  RRLIB_UNIT_TESTS_ADD_TEST(Pose);
  RRLIB_UNIT_TESTS_ADD_TEST(Factory);
  RRLIB_UNIT_TESTS_ADD_TEST(Channels);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void Double()
  {
    double data[cNUMBER_OF_SAMPLES] = { 0.4, 0.1, 0.2, 0.5, 0.8 };

    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(0.1, FuseValuesUsingMaximumKey<double>(data, data + cNUMBER_OF_SAMPLES, keys, keys + cNUMBER_OF_SAMPLES), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(0.4, FuseValuesUsingAverage<double>(data, data + cNUMBER_OF_SAMPLES), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(0.3, FuseValuesUsingWeightedAverage<double>(data, data + cNUMBER_OF_SAMPLES, keys, keys + cNUMBER_OF_SAMPLES), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(0.4, FuseValuesUsingMedianVoter<double>(data, data + cNUMBER_OF_SAMPLES), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(0.2, FuseValuesUsingMedianKeyVoter<double>(data, data + cNUMBER_OF_SAMPLES, keys, keys + cNUMBER_OF_SAMPLES), 1E-6);
  }

  void Pose()
  {
    std::vector<math::tPose2D> data;
    data.push_back(math::tPose2D(0.4, 0.4, 0.4));
    data.push_back(math::tPose2D(0.1, 0.1, 0.1));
    data.push_back(math::tPose2D(0.2, 0.2, 0.2));
    data.push_back(math::tPose2D(0.5, 0.5, 0.5));
    data.push_back(math::tPose2D(0.8, 0.8, 0.8));
    assert(data.size() == cNUMBER_OF_SAMPLES);

    RRLIB_UNIT_TESTS_ASSERT(IsEqual(math::tPose2D(0.1, 0.1, 0.1), FuseValuesUsingMaximumKey<math::tPose2D>(data.begin(), data.end(), keys, keys + cNUMBER_OF_SAMPLES)));

    RRLIB_UNIT_TESTS_ASSERT(IsEqual(math::tPose2D(0.4, 0.4, 0.4), FuseValuesUsingAverage<math::tPose2D>(data.begin(), data.end())));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(math::tPose2D(0.3, 0.3, 0.3), FuseValuesUsingWeightedAverage<math::tPose2D>(data.begin(), data.end(), keys, keys + cNUMBER_OF_SAMPLES)));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(math::tPose2D(0.4, 0.4, 0.4), FuseValuesUsingMedianVoter<math::tPose2D>(data.begin(), data.end())));

    math::tPose2D result = FuseValuesUsingMedianKeyVoter<math::tPose2D>(data.begin(), data.end(), keys, keys + cNUMBER_OF_SAMPLES);
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(math::tPose2D(0.2, 0.2, 0.2), result) || IsEqual(math::tPose2D(0.5, 0.5, 0.5), result));
  }

  void Factory()
  {
    InitializeFactory<math::tPose2D>();

    tDataFusion<math::tPose2D> *fusion = rrlib::data_fusion::tDataFusionFactory<math::tPose2D>::Instance().Create("Average");
    delete fusion;
  }

  void Channels()
  {
    tAverage<double, channel::Average> cyclic_fusion;
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(Test);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
