/*
 * test_differentiator.cpp
 *
 *  Created on: 28-Feb-2015
 *      Author: nilxwam
 */

#include<iostream>

template<typename T>
T add(T a, T b) {
	T c;
	c = a + b;
	return c;
}

template<class A>
class Add {
public:
	A add(A a, A b) {
		A c = a + b;
		return c;
	}
};

int main() {

	Add<float> niluadd;
	int a, b;
	float c, d;
	a = 2.1;
	b = 3.1;
	c = 2.1;
	d = 3.1;
	std::cout << niluadd.add(2.3,2.1) << std::endl;
	std::cout << add(a, b) << std::endl;
	std::cout << add(c, d) << std::endl;
	return 0;
}
