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
/*!\file    tDataFusion.hpp
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

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/definitions.h"

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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tDataFusion constructors
//----------------------------------------------------------------------
template <typename TSample>
tDataFusion<TSample>::tDataFusion()
    : number_of_samples(0)
{}

//----------------------------------------------------------------------
// tDataFusion destructor
//----------------------------------------------------------------------
template <typename TSample>
tDataFusion<TSample>::~tDataFusion()
{}

//----------------------------------------------------------------------
// tDataFusion ResetState
//----------------------------------------------------------------------
template <typename TSample>
void tDataFusion<TSample>::ResetState()
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Resetting state.");
  this->ClearSamples();
  this->ResetStateImplementation();
}

//----------------------------------------------------------------------
// tDataFusion ClearSamples
//----------------------------------------------------------------------
template <typename TSample>
void tDataFusion<TSample>::ClearSamples()
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Clearing samples.");
  this->number_of_samples = 0;
  this->ClearSamplesImplementation();
}

//----------------------------------------------------------------------
// tDataFusion AddSample
//----------------------------------------------------------------------
template <typename TSample>
void tDataFusion<TSample>::AddSample(const tSample &sample, double key)
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_2, "Adding sample ", sample, " with key ", key);
  if (this->AddSampleImplementation(sample, key))
  {
    this->number_of_samples++;
  }
}

//----------------------------------------------------------------------
// tDataFusion GetFusedValue
//----------------------------------------------------------------------
template <typename TSample>
const TSample tDataFusion<TSample>::GetFusedValue() const
{
  RRLIB_LOG_STREAM(logging::eLL_DEBUG_VERBOSE_1, "Calculating result from ", this->number_of_samples, " samples.");
  if (this->number_of_samples == 0)
  {
    throw std::logic_error("Trying to calculate a fused value without providing sample data before!");
  }
  return this->GetFusedValueImplementation();
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
