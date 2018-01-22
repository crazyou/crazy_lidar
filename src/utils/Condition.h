/*
 * Condition.h
 *
 *  Created on: 2016年10月17日
 *      Author: seeing
 */

#ifndef _THREAD_CONDITION_H_
#define _THREAD_CONDITION_H_

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace NS_NaviCommon
{
  
  class Condition
  {
  public:
    
    enum
    {
      COND_OK = 1, COND_TIMEOUT = -1, COND_FAILED = 0,
    };

    enum
    {
      INFINITE_WAIT = 0xFFFFFFFF,
    };

    Condition (bool isAutoReset = true, bool isSignal = false)
        : _is_signalled (isSignal), _isAutoReset (isAutoReset)
    {
      
    }
    
    ~ Condition ()
    {
      release ();
    }
    
    void
    set (bool isSignal = true)
    {
      if (isSignal)
      {
        _cond_locker.lock ();
        
        if (_is_signalled == false)
        {
          _is_signalled = true;
          _cond_var.notify_one ();
        }
        _cond_locker.unlock ();
      }
      else
      {
        _cond_locker.lock ();
        _is_signalled = false;
        _cond_locker.unlock ();
      }
    }
    
    unsigned long
    wait (unsigned long timeout = INFINITE_WAIT)
    {
      unsigned long ans = COND_OK;
      _cond_locker.lock ();
      
      if (!_is_signalled)
      {
        
        if (timeout == INFINITE_WAIT)
        {
          _cond_var.wait (_cond_locker);
        }
        else
        {
          
          if (_cond_var.timed_wait (
              _cond_locker,
              (boost::get_system_time () + boost::posix_time::seconds (timeout))))
          {
            // signalled
          }
          else
          {
            ans = COND_TIMEOUT;
            goto _final;
          }
          
        }
      }
      
      assert(_is_signalled);
      
      if (_isAutoReset)
      {
        _is_signalled = false;
      }
      _final: _cond_locker.unlock ();
      
      return ans;
      
    }
  protected:
    
    void
    release ()
    {
      
    }
    
    boost::condition _cond_var;
    boost::mutex _cond_locker;
    bool _is_signalled;
    bool _isAutoReset;
  };

}

#endif /* THREAD_CONDITION_H_ */
