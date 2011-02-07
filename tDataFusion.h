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
/*!\file    tDataFusion.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-25
 *
 * \brief   Contains tDataFusion
 *
 * \b tDataFusion
 *
 * A few words for tDataFusion
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__tDataFusion_h__
#define __rrlib__data_fusion__tDataFusion_h__

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of tDataFusion
/*! A more detailed description of tDataFusion, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class tDataFusion
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef TSample tSample;

  tDataFusion();

  virtual ~tDataFusion() = 0;

  void ResetState();

  void ClearSamples();

  void AddSample(const tSample &sample, double key = 1);

  const tSample GetFusedValue() const;

  inline const unsigned int NumberOfSamples() const
  {
    return this->number_of_samples;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  unsigned int number_of_samples;

  virtual const char *GetLogDescription() const
  {
    return "tDataFusion";
  }

  virtual const void ResetStateImplementation() = 0;
  virtual const void ClearSamplesImplementation() = 0;
  virtual const bool AddSampleImplementation(const tSample &sample, double key) = 0;
  virtual const tSample GetFusedValueImplementation() const = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/data_fusion/tDataFusion.hpp"

#endif
