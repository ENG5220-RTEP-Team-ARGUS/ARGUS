#ifndef __CPP_TIMER_STD_CALLBACK
#define __CPP_TIMER_STD_CALLBACK
#include "CppTimer.h"
#include <functional>

// This is a demo how to create a callback with std::function which allows
// calling methods in other classes by registering a lambda function!

class CppTimerCallback : public CppTimer {

public:

    using CallbackFunction = std::function<void(void)>;

    void registerEventCallback(CallbackFunction cf) {
        callbackFunction = cf;
    }

private:
    // overloading the abstract function
    inline void timerEvent() override {
        callbackFunction();
    }

private:
    CallbackFunction callbackFunction;
};


#endif
