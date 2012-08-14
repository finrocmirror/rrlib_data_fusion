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
/*!\file    Base.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-02-14
 *
 * \brief   Contains Base
 *
 * \b Base
 *
 * A few words for Base
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__data_fusion__policies__channel__Base_h__
#define __rrlib__data_fusion__policies__channel__Base_h__

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
//! Short description of Base
/*! A more detailed description of Base, which
 *  Tobias Foehst hasn't done yet !!
 */
template <typename TSample>
class Base
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  Base()
    : valid(0)
  {}

  inline const bool IsValid() const
  {
    return this->valid;
  }

  void AddSample(const TSample &sample, double key)
  {
    this->AddSampleImplementation(sample, key);
  }

  const TSample GetSample() const
  {
    if (!this->valid)
    {
      throw std::runtime_error("Trying to get sample from invalid channel");
    }
    return this->GetSampleImplementation();
  }

  const double GetKey() const
  {
    if (!this->valid)
    {
      throw std::runtime_error("Trying to get key from invalid channel");
    }
    return this->GetKeyImplementation();
  }

  void ClearData()
  {
    this->valid = false;
    this->ClearDataImplementation();
  }

  void PrepareForNextTimestep()
  {
    this->PrepareForNextTimestepImplementation();
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline void SetValid(bool valid)
  {
    this->valid = valid;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  bool valid;

  virtual void AddSampleImplementation(const TSample &sample, double key) = 0;
  virtual const TSample GetSampleImplementation() const = 0;
  virtual const double GetKeyImplementation() const = 0;
  virtual void ClearDataImplementation() = 0;
  virtual void PrepareForNextTimestepImplementation() = 0;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
