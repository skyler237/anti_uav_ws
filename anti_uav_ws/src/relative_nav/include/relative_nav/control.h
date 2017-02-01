/*! \file control.h
  * \author David Wheeler
  * \date Oct 2014
  *
  * \brief Useful control functions.
  *
*/

#ifndef relative_nav_CONTROL_H
#define relative_nav_CONTROL_H

#include <algorithm>

namespace relative_nav
{

double saturate(double value, double low, double high);

} //end namespace

#endif
