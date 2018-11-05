#include <iostream>
#include <Eigen/Dense>
#include "../include/DebugUtils.h"

int main()
{
  try {
    Eigen::Matrix<float, 4 , 4> test4x4;
    test4x4.setIdentity();
    LOG_INFO("Adding eigen matrix...")
		LOG_INFO(test4x4)
	} catch (exception& e) {
		cout << e.what();
	}
}
