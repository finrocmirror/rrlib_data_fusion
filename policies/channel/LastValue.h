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
/*!\file    LastValue.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-02-14
 *
 * \brief   Contains LastValue
 *
 * \b LastValue
 *
 * A few words for LastValue
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__policies__channel__LastValue_h__
#define __rrlib__data_fusion__policies__channel__LastValue_h__

#include "rrlib/data_fusion/policies/channel/Base.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
namespace channel
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of LastValue
/*! A more detailed description of LastValue, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class LastValue : public Base<TSample>
{

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TSample sample;
  double key;

  virtual void AddSampleImplementation(const TSample &sample, double key)
  {
    this->sample = sample;
    this->key = key;
    this->SetValid(true);
  }

  virtual const TSample GetSampleImplementation() const
  {
    return this->sample;
  }

  virtual const double GetKeyImplementation() const
  {
    return this->key;
  }

  virtual void ClearDataImplementation()
  {}

  virtual void PrepareForNextTimestepImplementation()
  {}

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
