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
/*!\file    tMedianVoter.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tMedianVoter
 *
 * \b tMedianVoter
 *
 * A few words for tMedianVoter
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tMedianVoter_h__
#define __rrlib__data_fusion__tMedianVoter_h__

#include "rrlib/data_fusion/tDataFusion.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <list>

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
//! Short description of tMedianVoter
/*! A more detailed description of tMedianVoter, which
 *  Tobias Foehst hasn't done yet !!
 */
template <
typename TSample,
         template <typename> class TChannel = channel::LastValue
         >
class tMedianVoter : public tDataFusion<TSample, TChannel>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  virtual const char *GetLogDescription() const
  {
    return "tMedianVoter";
  }

  virtual const bool HasValidState() const
  {
    return true;
  }

  static bool CompareSamples(const std::pair<TSample, double> &a, const std::pair<TSample, double> &b)
  {
    return a.second < b.second;
  }

  virtual const TSample CalculateFusedValue(const std::vector<TChannel<TSample>> &channels)
  {
    std::list<TSample> samples;
    for (typename std::vector<TChannel<TSample>>::const_iterator it = channels.begin(); it != channels.end(); ++it)
    {
      samples.push_back(it->GetSample());
    }
    samples.sort();
    typename std::list<TSample>::iterator it = samples.begin();
    std::advance(it, std::trunc(samples.size() / 2.0));
    return *it;
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
