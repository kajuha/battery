#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "qucc.h"

#include <unistd.h>
#include <chrono>
#include <sys/time.h>
#include <ctime>

Qucc qucc;

int main() {
	int i = 0;

	qucc = Qucc("/dev/recipe.driver.battery", 9600);

	if (qucc.initSerial() == false) {
		return 0;
	}

	while(true) {
		qucc.receiveQuccState();

		if ((i++ % 1000) == 0) {
			printf("i: %d\n", i);
			qucc.sendQuccCmd();
		}

		usleep(1000);
	}

	qucc.closeSerial();

	return 0;
}