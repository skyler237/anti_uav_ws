#include "relative_nav/control.h"

namespace relative_nav
{

double saturate(double value, double low, double high)
{
	return std::min(std::max(value,low),high);
}


} // end namespace
