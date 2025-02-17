#ifndef PUBLIC_FUNCTION
#define PUBLIC_FUNCTION

#include "Timer.hpp"
#include <unistd.h>
#include "/usr/include/boost/parameter.hpp"

#define SHOOT_PREPARATION 1
#define SHOOT 2
#define DRIBBLE 3
#define PASS_RECEIVE_BALL 4
#define PASS_RECEIVE_BALL_RESET 5
#define CANCEL_RESET_POSITION 6
#define PASS_BALL 7

#define TRANS2PAW 13
#define TEST 8
#define OPEN_PAW 9
#define CLOSE_PAW 10
#define DRIBBLE_RECEIVE_BALL 11
#define EMPTY 12


int8_t get_char();

#endif