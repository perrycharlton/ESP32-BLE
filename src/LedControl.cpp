#include <ArduinoJson.h>
#include <LedControl.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// boolean ledFader = false;
DynamicJsonDocument ledDoc(1024);
JsonArray leds = ledDoc["leds"].to<JsonArray>();

LedControl::LedControl()
{
}

void LedControl::Setup(int freq, int resolution, int pinsLed[], int sizeArray)
{

	_freq = freq;
	_resolution = resolution;
	_sizeArray = sizeArray;
	_pinsLed = pinsLed;

	// _ticker = new Ticker;

	for (int i = 0; i < _sizeArray; i++)
	{
		ledcSetup(i, _freq, resolution);
		Serial.printf("Original Pin: %d, copied pin: %d, Channel: %d\n", pinsLed[i], _pinsLed[i], i);
		ledcAttachPin(pinsLed[i], i);
	}
}


int LedControl::getLed(int channel, char arr[])
{
	StaticJsonDocument<64> rtnMsg;
	int brightness = ledcRead(channel);
	rtnMsg["brightness"] = brightness;
	rtnMsg["channel"] = channel;
	rtnMsg["state"] = (brightness == 0) ? "off" : "on";
	serializeJson(rtnMsg, arr, 64);
	serializeJson(rtnMsg, Serial);
	return channel;
}


int LedControl::setLed(JsonDocument& rcvdMsg, char arr[])
{
	JsonDocument msg = rcvdMsg;
	int brightness = msg["brightness"];
	int channel = msg["channel"];
	ledcWrite(channel, brightness);
	getLed(msg["channel"], arr);

	return channel;
}

void LedControl::getLeds(JsonArray& arr)
{
	// Serial.println("first part worked");
	arr = leds;
}