/**
 * @file Simulator/main.cpp
 *
 * The start point for the simulator interface that runs 
 * in the background.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jun 2017
 */
 
#include <assert.h>
#include <iostream>
#include <string.h>
#include "VREPInterface.h"

int main()
{
	VREPInterface* vrep = new VREPInterface();
	vrep->start();
	while (true)
		vrep->update();
	return 0;
}
