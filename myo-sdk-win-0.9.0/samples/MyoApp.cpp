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
		: isCalibrated(false), previousKey(0), rollPos(0), pitchPos(0), currentPose() {}

	bool isCalibrated; // TODO base 
	// Last generated keystroke & current keystroke to generate
	int previousKey;
	// These values are set by onOrientationData() and onPose() below.
	int rollBase, pitchBase, rollPos, pitchPos;
	// String containing all single chars this Myo will map its orientation to 
	std::string keyset;
	myo::Pose currentPose;

	void calibrate() {
		rollBase = rollPos;
		pitchBase = pitchPos;
		isCalibrated = true;
		std::cout << "calibrated " << keyset << " with " << rollBase << ", " << pitchBase << std::endl;
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
		if (myoId == 0) return "qwert";//"cdega";
		if (myoId == 1) return "asdfg";//"12345";
		if (myoId == 2) return "zxcvb";
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

		MyoData myoData = orientationData[myoId];

		// Convert the floating point angles in radians to a scale from 0 to 18.
		myoData.rollPos = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		myoData.pitchPos = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);

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
		std::cout << o.rollPos << " | " << o.pitchPos << std::endl;

		int currentKey =
			o.rollPos > o.rollBase + 2 ? o.keyset[0] :
			(o.rollPos != 0 && o.rollPos < o.rollBase) ? o.keyset[1] :
			(o.pitchPos != 0 && o.pitchPos < o.pitchBase) ? o.keyset[2] :
			o.pitchPos > o.pitchBase + 6 ? o.keyset[3] :
			o.previousKey;

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
