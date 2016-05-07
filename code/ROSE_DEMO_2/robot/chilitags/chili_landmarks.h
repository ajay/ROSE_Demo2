// Written by:	Ajay Srivastava, Srihari Chekuri
// Tested by: 	Ajay Srivastava, Srihari Chekuri

#ifndef CHILI_LANDMARKS_H
#define CHILI_LANDMARKS_H

class chili_landmarks
{
	public:
		chili_landmarks();
		~chili_landmarks();
		void update();

		double tags [1024][4];
		// 0 -> detected
		// 1 -> x
		// 2 -> z
		// 3 -> y
};

#endif