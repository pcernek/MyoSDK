// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>

#include <windows.h>
#pragma comment(lib,"user32.lib") // wtf, HACK

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>



class MyoData {
public:
	MyoData()
		: isCalibrated(false), previousKey(0), rollPos(0), pitchPos(0), yawPos(0), currentPose() {}

	bool isCalibrated; // TODO base 
	// Last generated keystroke & current keystroke to generate
	int previousKey;
	// These values are set by onOrientationData() and onPose() below.
	int rollBase, pitchBase, yawBase, rollPos, pitchPos, yawPos;
	// String containing all single chars this Myo will map its orientation to 
	std::string keyset;
	myo::Pose currentPose;

	void calibrate() {
		rollBase = rollPos;
		pitchBase = pitchPos;
		yawBase = yawPos;
		isCalibrated = true;
		std::cout << "calibrated " << keyset << " with " << rollBase << ", " << pitchBase << ", " << yawBase << std::endl;
	}

	// next key generated by cross-based gesture strategy (hand rotation and up-down movement)
	char nextKeyCrossGesture() {
		return 
			(rollPos > rollBase + 2 && pitchPos < pitchBase + 4 && pitchPos > pitchBase - 3) ? keyset[0] :
			(rollPos != 0 && rollPos < rollBase && pitchPos < pitchBase + 4 && pitchPos > pitchBase - 3) ? keyset[1] :
			(pitchPos > 0 && pitchPos < pitchBase - 4) ? keyset[2] :
			pitchPos > pitchBase + 6 ? keyset[3] :
			previousKey;
	}

	// next key generated by drum stick gesture strategy 
	char nextKeyDrumStickGesture() {
		return
			pitchPos > pitchBase ? previousKey :  // hand on top, don't play sound
			// otherwise hand down, decide on sound depending on which direction hand points at
			yawPos > 12 ? keyset[0] :
			yawPos > 9 ? keyset[1] :
			yawPos > 6 ? keyset[2] :
			yawPos > 3 ? keyset[3] : keyset[4];
	}
};


// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:

	// We store each Myo pointer that we pair with in this list, so that we can keep track of the order we've seen
	// each Myo and give it a unique short identifier (see onPair() and identifyMyo() above).
	std::vector<myo::Myo*> knownMyos;

	// For each paired Myo we store a record that gets updated and printed as the orientation changes
	std::vector<MyoData> orientationData;

	void onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
	{
		knownMyos.push_back(myo);
		MyoData myoData;
		myoData.keyset = getKeySet(myo);
		orientationData.push_back(myoData);

		// Now that we've added it to our list, get our short ID for it and print it out.
		std::cout << "Paired with " << identifyMyo(myo) << " (" << myoData.keyset << ")." << std::endl;
	}

	std::string getKeySet(myo::Myo* myo) {
		size_t myoId = identifyMyo(myo);
		if (myoId == 0) return "qerty";
		if (myoId == 1) return "adfgh"; //TODO przywrocic przed commitem!!!;
		if (myoId == 2) return "zcvbn";
		throw "Up to three Myos are supported!";
	}

	// This is a utility function implemented for this sample that maps a myo::Myo* to a unique ID starting at 1.
	// It does so by looking for the Myo pointer in knownMyos, which onPair() adds each Myo into as it is paired.
	size_t identifyMyo(myo::Myo* myo) {
		// Walk through the list of Myo devices that we've seen pairing events for.
		for (size_t i = 0; i < knownMyos.size(); ++i) {
			// If two Myo pointers compare equal, they refer to the same Myo device.
			if (knownMyos[i] == myo) {
				return i;
			}
		}

		return -1;
	}


	void onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
	{
		std::cout << "Myo " << identifyMyo(myo) << " has connected." << std::endl;
	}

	void onDisconnect(myo::Myo* myo, uint64_t timestamp)
	{
		std::cout << "Myo " << identifyMyo(myo) << " has disconnected." << std::endl;
	}

	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		size_t myoId = identifyMyo(myo);


		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

		MyoData myoData = orientationData[myoId];

		// Convert the floating point angles in radians to a scale from 0 to 18.
		myoData.rollPos = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		myoData.pitchPos = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		myoData.yawPos = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);

		if (!myoData.isCalibrated) {
			myoData.calibrate();
		}

		orientationData[myoId] = myoData;
	}

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		orientationData[identifyMyo(myo)].currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
			// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
			// Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);

			// Notify the Myo that the pose has resulted in an action, in this case changing
			// the text on the screen. The Myo will vibrate.
			myo->notifyUserAction();
		}
		else {
			// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
			// are being performed, but lock after inactivity.
			myo->unlock(myo::Myo::unlockTimed);
		}
	}

	// We define this function to print the current values that were updated by the on...() functions above.
	void printAll() {
		for (size_t i = 0; i < knownMyos.size(); ++i) {
			print(i);
		}
	}

	void print(size_t myoId)
	{
		MyoData o = orientationData[myoId];
		
		// diagnostic info
		std::cout << "MYO " << myoId << ": ";
		std::cout << o.rollPos << " | " << o.pitchPos << " | " << o.yawPos << std::endl;

		// #justhackathonthings
		int currentKey = o.nextKeyCrossGesture();
		//int currentKey = o.nextKeyDrumStickGesture();
		std::cout << (char)currentKey << std::endl;
		
		if (currentKey != o.previousKey) {
			pressKey(currentKey);
			o.previousKey = currentKey;
		}

		orientationData[myoId] = o;
	}

	void pressKey(int which) {
		byte keyCode = which - 32;

		keybd_event(keyCode, 0, KEYEVENTF_EXTENDEDKEY, 0);
		keybd_event(keyCode, 0, KEYEVENTF_EXTENDEDKEY | KEYEVENTF_KEYUP, 0);
	};
};

int main(int argc, char** argv)
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		// Finally we enter our main loop.
		while (1) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			hub.run(1000 / 2);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			collector.printAll();
		}

		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
