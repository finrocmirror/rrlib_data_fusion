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
/*!\file    functions.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__functions_h__
#define __rrlib__data_fusion__functions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <stdexcept>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/tMaximumKey.h"
#include "rrlib/data_fusion/tAverage.h"
#include "rrlib/data_fusion/tWeightedAverage.h"
#include "rrlib/data_fusion/tWeightedSum.h"
#include "rrlib/data_fusion/tMedianVoter.h"
#include "rrlib/data_fusion/tMedianKeyVoter.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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
// Function declaration
//----------------------------------------------------------------------



template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingMaximumKey(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tMaximumKey<TSample> fuser;
  if (begin_samples == end_samples)
  {
    throw std::logic_error("Given empty list of samples!");
  }
  fuser.SetNumberOfChannels(std::distance(begin_samples, end_samples));
  fuser.UpdateAllChannels(begin_samples, end_samples, begin_keys, end_keys);
  return fuser.FusedValue();
}

template <typename TSample, typename TSampleIterator>
inline const TSample FuseValuesUsingAverage(TSampleIterator begin_samples, TSampleIterator end_samples)
{
  tAverage<TSample> fuser;
  if (begin_samples == end_samples)
  {
    throw std::logic_error("Given empty list of samples!");
  }
  fuser.SetNumberOfChannels(std::distance(begin_samples, end_samples));
  fuser.UpdateAllChannels(begin_samples, end_samples);
  return fuser.FusedValue();
}

template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingWeightedAverage(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tWeightedAverage<TSample> fuser;
  if (begin_samples == end_samples)
  {
    throw std::logic_error("Given empty list of samples!");
  }
  fuser.SetNumberOfChannels(std::distance(begin_samples, end_samples));
  fuser.UpdateAllChannels(begin_samples, end_samples, begin_keys, end_keys);
  return fuser.FusedValue();
}

template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingWeightedSum(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tWeightedSum<TSample> fuser;
  if (begin_samples == end_samples)
  {
    throw std::logic_error("Given empty list of samples!");
  }
  fuser.SetNumberOfChannels(std::distance(begin_samples, end_samples));
  fuser.UpdateAllChannels(begin_samples, end_samples, begin_keys, end_keys);
  return fuser.FusedValue();
}

template <typename TSample, typename TSampleIterator>
inline const TSample FuseValuesUsingMedianVoter(TSampleIterator begin_samples, TSampleIterator end_samples)
{
  tMedianVoter<TSample> fuser;
  if (begin_samples == end_samples)
  {
    throw std::logic_error("Given empty list of samples!");
  }
  fuser.SetNumberOfChannels(std::distance(begin_samples, end_samples));
  fuser.UpdateAllChannels(begin_samples, end_samples);
  return fuser.FusedValue();
}

template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingMedianKeyVoter(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tMedianKeyVoter<TSample> fuser;
  if (begin_samples == end_samples)
  {
    throw std::logic_error("Given empty list of samples!");
  }
  fuser.SetNumberOfChannels(std::distance(begin_samples, end_samples));
  fuser.UpdateAllChannels(begin_samples, end_samples, begin_keys, end_keys);
  return fuser.FusedValue();
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
