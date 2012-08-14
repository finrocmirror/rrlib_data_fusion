//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    Average.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-02-14
 *
 * \brief   Contains Average
 *
 * \b Average
 *
 * A few words for Average
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__policies__channel__Average_h__
#define __rrlib__data_fusion__policies__channel__Average_h__

#include "rrlib/data_fusion/policies/channel/Base.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/data_fusion/functions.h"

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
namespace channel
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of Average
/*! A more detailed description of Average, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class Average : public Base<TSample>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  std::vector<TSample> samples;
  std::vector<double> keys;

  virtual void AddSampleImplementation(const TSample &sample, double key)
  {
    this->samples.push_back(sample);
    this->keys.push_back(key);
    this->SetValid(true);
  }

  virtual const TSample GetSampleImplementation() const
  {
    return FuseValuesUsingAverage<TSample>(this->samples.begin(), this->samples.end());
  }

  virtual const double GetKeyImplementation() const
  {
    return FuseValuesUsingAverage<TSample>(this->keys.begin(), this->keys.end());
  }

  virtual void ClearDataImplementation()
  {
    this->samples.clear();
    this->keys.clear();
    this->SetValid(false);
  }

  virtual void PrepareForNextTimestepImplementation()
  {
    this->ClearDataImplementation();
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
