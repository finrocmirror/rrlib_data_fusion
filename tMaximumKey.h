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
/*!\file    tMaximumKey.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tMaximumKey
 *
 * \b tMaximumKey
 *
 * A few words for tMaximumKey
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tMaximumKey_h__
#define __rrlib__data_fusion__tMaximumKey_h__

#include "rrlib/data_fusion/tDataFusion.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <limits>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
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
// Class declaration
//----------------------------------------------------------------------
//! Short description of tMaximumKey
/*! A more detailed description of tMaximumKey, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class tMaximumKey : public tDataFusion<TSample>
{

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  tMaximumKey()
      : current_maximum_key(-std::numeric_limits<double>::max())
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TSample current_result;
  double current_maximum_key;

  virtual const char *GetLogDescription() const
  {
    return "tMaximumKey";
  }

  virtual const void ResetStateImplementation()
  {}

  virtual const void ClearSamplesImplementation()
  {
    this->current_maximum_key = -std::numeric_limits<double>::max();
  }

  virtual const bool AddSampleImplementation(const TSample &sample, double key)
  {
    if (key > this->current_maximum_key)
    {
      this->current_maximum_key = key;
      this->current_result = sample;
    }
    return true;
  }

  virtual const TSample GetFusedValueImplementation() const
  {
    return this->current_result;
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
