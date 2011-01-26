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
#ifndef _rrlib_data_fusion_functions_h_
#define _rrlib_data_fusion_functions_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/tMaximumKey.h"
#include "rrlib/data_fusion/tAverage.h"
#include "rrlib/data_fusion/tWeightedAverage.h"
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

namespace
{

template <typename TFuser, typename TSampleIterator, typename TKeyIterator>
inline const typename TFuser::tSample FuseValues(TFuser &fuser, TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  TSampleIterator sample = begin_samples;
  TKeyIterator key = begin_keys;
  while (sample != end_samples && key != end_keys)
  {
    fuser.AddSample(*sample, *key);
    ++sample;
    ++key;
  }

  if (sample != end_samples || key != end_keys)
  {
    throw std::runtime_error("Number of samples did not match number of keys!");
  }

  return fuser.GetFusedValue();
}

template <typename TFuser, typename TSampleIterator>
inline const typename TFuser::tSample FuseValues(TFuser &fuser, TSampleIterator begin_samples, TSampleIterator end_samples)
{
  TSampleIterator sample = begin_samples;
  while (sample != end_samples)
  {
    fuser.AddSample(*sample);
    ++sample;
  }

  return fuser.GetFusedValue();
}

}



template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingMaximumKey(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tMaximumKey<TSample> fuser;
  return FuseValues(fuser, begin_samples, end_samples, begin_keys, end_keys);
}

template <typename TSample, typename TSampleIterator>
inline const TSample FuseValuesUsingAverage(TSampleIterator begin_samples, TSampleIterator end_samples)
{
  tAverage<TSample> fuser;
  return FuseValues(fuser, begin_samples, end_samples);
}

template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingWeightedAverage(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tWeightedAverage<TSample> fuser;
  return FuseValues(fuser, begin_samples, end_samples, begin_keys, end_keys);
}

template <typename TSample, typename TSampleIterator>
inline const TSample FuseValuesUsingMedianVoter(TSampleIterator begin_samples, TSampleIterator end_samples)
{
  tMedianVoter<TSample> fuser;
  return FuseValues(fuser, begin_samples, end_samples);
}

template <typename TSample, typename TSampleIterator, typename TKeyIterator>
inline const TSample FuseValuesUsingMedianKeyVoter(TSampleIterator begin_samples, TSampleIterator end_samples, TKeyIterator begin_keys, TKeyIterator end_keys)
{
  tMedianKeyVoter<TSample> fuser;
  return FuseValues(fuser, begin_samples, end_samples, begin_keys, end_keys);
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
