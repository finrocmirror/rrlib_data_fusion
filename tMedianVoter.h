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
#ifndef _rrlib_data_fusion_tMedianVoter_h_
#define _rrlib_data_fusion_tMedianVoter_h_

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
template <typename TSample>
class tMedianVoter : public tDataFusion<TSample>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  mutable std::list<TSample> samples;

  virtual const char *GetLogDescription() const
  {
    return "tMedianVoter";
  }

  virtual const void ResetStateImplementation()
  {}

  virtual const void ClearSamplesImplementation()
  {
    this->samples.clear();
  }

  virtual const bool AddSampleImplementation(const TSample &sample, double key)
  {
    this->samples.push_back(sample);
    return true;
  }

  virtual const TSample GetFusedValueImplementation() const
  {
    this->samples.sort();
    std::cout << rrlib::util::Join(this->samples.begin(), this->samples.end()) << std::endl;
    typename std::list<TSample>::iterator it = this->samples.begin();
    std::advance(it, std::trunc(this->NumberOfSamples() / 2.0));
    return *it;
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
