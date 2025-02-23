#include "Public_function.hpp"


/*检测键盘输入*/
int8_t get_char()
{
  fd_set rfds;
  struct timeval tv;
  int ch = 0;
  FD_ZERO(&rfds);
  FD_SET(0, &rfds);
  tv.tv_sec = 0;
  tv.tv_usec = 5;                            // 设置等待超时时间
  if (select(1, &rfds, NULL, NULL, &tv) > 0) // 检测键盘是否有输入
  {
    ch = getchar();
  }
  return ch;
}