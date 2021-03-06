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
template <
typename TSample,
         template <typename> class TChannel = channel::LastValue
         >
class tMaximumKey : public tDataFusion<TSample, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tMaximumKey";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  virtual const TSample CalculateFusedValue(const std::vector<TChannel<TSample>> &channels)
  {
    auto channel = channels.begin();
    TSample result = channel->GetSample();
    double maximum_key = channel->GetKey();
    while (channel != channels.end())
    {
      if (channel->GetKey() > maximum_key)
      {
        result = channel->GetSample();
        maximum_key = channel->GetKey();
      }
      ++channel;
    }
    return result;
  }

  virtual void ResetStateImplementation()
  {}

  virtual void EnterNextTimestepImplementation()
  {}

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
