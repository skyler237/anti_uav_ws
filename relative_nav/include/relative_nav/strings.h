/*! \file strings.h
  * \author David Wheeler
  * \date Oct 2014
  *
  * \brief Useful string functions.
  *
*/

#ifndef relative_nav_STRINGS_H
#define relative_nav_STRINGS_H

#include <string>
#include <iostream>
#include <sys/stat.h> /// Provides the mkdir() function
#include <boost/filesystem.hpp>

namespace relative_nav
{
  inline std::string int2string(int input)
  {
    std::ostringstream int_to_string_converter;
    int_to_string_converter << input;
    std::string output = int_to_string_converter.str();
    return output;
  }

  inline void makeDirectory(std::string path, bool overwrite)
  {
    // rwx permissions for owner and group, rx persmissions for others
    if(mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
      if(errno == EEXIST)
      {
        if(overwrite)
        {
          ROS_WARN_STREAM("Overwritting directory: " << path);
          boost::filesystem::remove_all(path.c_str());
          mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }
        else
        {
          ROS_ERROR_STREAM("Directory '" << path << "' already exists!  Exiting!");
          exit(1);
        }
      }
      else
      {
        ROS_ERROR_STREAM("Unable to create folder for cache files at '" << path << ".'  Error code = " << errno << ".  Exiting!");
        exit(1);
      }
    }
  }

} //end namespace

#endif
