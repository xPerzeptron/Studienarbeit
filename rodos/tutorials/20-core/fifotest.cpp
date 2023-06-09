#include "rodos.h"

static Application applic("FifoTest");

Fifo<int, 10> fifo;

class Sender : public StaticThread<> {
    void run() {
        int cnt = 0;
        xprintf("sender\n");
        while (1) {
            cnt++;
            bool ok = fifo.put(cnt);
            if (ok) {
                PRINTF("Sending %d\n", cnt);
            } else {
                PRINTF("Fifo full\n");
            }
            if ((cnt % 15) == 0) {
                PRINTF("Waiting 3 seconds\n");
                suspendCallerUntil(NOW() + 3 * SECONDS);
            }
        }
    }
};

class Receiver : public StaticThread<> {
    void run() {
        int cnt;
        xprintf("receiver\n");

        while (1) {
            bool ok = fifo.get(cnt);
            if (ok) {
                PRINTF("reading %d\n", cnt);
            } else {
                suspendCallerUntil(NOW() + 1 * SECONDS);
            }
        }
    }
};

/******************************/

Sender sender;
Receiver receiver;
