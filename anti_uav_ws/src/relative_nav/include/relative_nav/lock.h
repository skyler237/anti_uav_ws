// License info needed here!

/*! \file lock.h
  * \author David Wheeler
  * \date July 2014
  *
  * \brief Lock is a helper class using scope-based resource management around mutex threading.
  *
*/

#ifndef relative_nav_LOCK_H
#define relative_nav_LOCK_H

namespace relative_nav {

/*!
 *  \class Lock
 *  \brief Lock is a helper class using scope-based resource management around mutex threading.
 *  {
 *    Lock lock(mutex);
 *    ...
 *  }
 *
*/
class Lock
{
  pthread_mutex_t & m;

public:
  explicit Lock(pthread_mutex_t & _m) : m(_m) { pthread_mutex_lock(&m); }
  ~Lock() { pthread_mutex_unlock(&m); }
};

} // namespace relative_nav
#endif
