// Pin interrupt connections
// Aileron and elevator input uses hardware interrupts
// Mode switch uses pin change interrupts
#define AILPIN_INT 2
#define ELEVPIN_INT 3
#define MODEPIN_INT 4

#define AILPIN_OUT 9
#define ELEVPIN_OUT 10

#define INPUTRANGES 3
#define MODE_THRESHOLD 200
#define RECEIVER_LOW 1000
#define RECEIVER_MID 1500
#define RECEIVER_HIGH 2000

#define ROLL_LIMIT 50
#define PITCH_LIMIT 50

// Forbid copy of object
#define CLASS_NO_COPY(c)        \
    c(const c &other) = delete; \
    c &operator=(const c &) = delete;

#define DEBUG 1