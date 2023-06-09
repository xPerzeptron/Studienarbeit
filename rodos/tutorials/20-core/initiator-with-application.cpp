

#include "rodos.h"

class MyInitiator : public Initiator {
public:
    void init() {
        xprintf(" This will be done before any thread begins\n");
        xprintf(" Doing so you do not need to use MAIN\n");
    }
} myInitiator;

class MyInitiatorApplic : public Application {
public:
    MyInitiatorApplic() : Application("initiatorApplic", 2000) {}

    void init() {
        xprintf(" This is the initiator from applications\n");
    }
} myInitiatorApplic;
