#include <iostream>
#include "util.h"
int main() {
  bool exit_ = false;
  char c;
  while ( !exit_ ) {
    int res = venom::wait_key(1,0,c);
    if (res < 0) {
      std::cout << "error: select fail\n";
      break;
    }

    std::cout << "Catch char " << c << std::endl;
    switch (c) {
      case 'a':
	std::cout << "Go left\n";
	break;
      case 'w':
      case 'W':
	std::cout << "Go up\n";
	break;
      case 'd':
      case 'D':
	std::cout << "Go right\n";
	break;
      case 's':
      case 'S':
	std::cout << "Go down\n";
	break;
      case 'q':
      case 'Q':
	std::cout << "Exit\n";
	exit_ = true;
	break;
    }
  }

  return 0;
}
