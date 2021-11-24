/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                  PLEASE READ THE FOLLOWING! or don't                                                    *
 *                                   Written by Liam Price 2020 and is free to use for non commerical purposes.                            *
 *                                                                                                                                         *
 * The purpose of this software is to compile and run on an Arduino based IoT device, like an ESP32. This is used to control an 'Automated *
 * Quad Bike' that I designed to be controlled from a computer steering wheel joystick, however you can use the code for any robot project *
 * if you want to see the AQB (Automated Quad Bike) Robot I designed this for software, visit my GitHub Repo for it:                       *
 * https://github.com/LeehamElectronics/AQB-Arduino                                                                                        *
 *                                                                                                                                         *
 * And here is the open source Python Control Panel I made for it:                                                                         *
 * https://github.com/LeehamElectronics/AQB-Control-Panel                                                                                  *
 *                                                                                                                                         *
 * If you'd like to support my open source work, buy me a coffee:                                                                          *
 * https://www.paypal.com/paypalme/liamproice/                                                                                             *
 *                                                                                                                                         *
 * Check out my website and portfolio at https://ldprice.com                                                                               *
 *                                                                                                                                         *
 * And if you need help, feel free to email me at liam@ldprice.com                                                                         *
 * Thanks!                                                                                                                                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 /*

 ██╗░░░░░██████╗░██████╗░██████╗░██╗░█████╗░███████╗     ░██████╗░█████╗░███████╗████████╗░██╗░░░░░░░██╗░█████╗░██████╗░███████╗
 ██║░░░░░██╔══██╗██╔══██╗██╔══██╗██║██╔══██╗██╔════╝     ██╔════╝██╔══██╗██╔════╝╚══██╔══╝░██║░░██╗░░██║██╔══██╗██╔══██╗██╔════╝
 ██║░░░░░██║░░██║██████╔╝██████╔╝██║██║░░╚═╝█████╗░░     ╚█████╗░██║░░██║█████╗░░░░░██║░░░░╚██╗████╗██╔╝███████║██████╔╝█████╗░░
 ██║░░░░░██║░░██║██╔═══╝░██╔══██╗██║██║░░██╗██╔══╝░░     ░╚═══██╗██║░░██║██╔══╝░░░░░██║░░░░░████╔═████║░██╔══██║██╔══██╗██╔══╝░░
 ███████╗██████╔╝██║░░░░░██║░░██║██║╚█████╔╝███████╗     ██████╔╝╚█████╔╝██║░░░░░░░░██║░░░░░╚██╔╝░╚██╔╝░██║░░██║██║░░██║███████╗
 ╚══════╝╚═════╝░╚═╝░░░░░╚═╝░░╚═╝╚═╝░╚════╝░╚══════╝     ╚═════╝░░╚════╝░╚═╝░░░░░░░░╚═╝░░░░░░╚═╝░░░╚═╝░░╚═╝░░╚═╝╚═╝░░╚═╝╚══════╝

 */

 /* ---------------------------------------------------------------------------------------------------------------------------
 --------------------------------------------------    Include Libraries   ----------------------------------------------------
 --------------------------------------------------------------------------------------------------------------------------- */
#include "include/configuration.h"
#include "include/mqttConfiguration.h"
#include "include/gpioConfiguration.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <WiFiManager.h> /* Used to easily connect to new WiFi networks */
#include <SoftwareSerial.h> /* use this library: https://github.com/plerup/espsoftwareserial/releases/tag/6.9.0 */
#include <PubSubClient.h> /* mqtt client */
#include <Arduino.h>
#include <ESP32Servo.h> /* Get it from https://github.com/madhephaestus/ESP32Servo */
#include <analogWrite.h> /* this was required for ESP32 chips to use analogWrite for some reason, look it up https://github.com/ERROPiX/ESP32_AnalogWrite */
#include <EEPROM.h> /* used to store information in non volitile memory */
#include <ArduinoJson.h>  /* download latest release here: https://github.com/bblanchon/ArduinoJson/releases */

/* ---------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------    Initialize Main Objects   -------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
WiFiClient espClient;
WiFiManager wifiManager;
PubSubClient client(espClient);
SoftwareSerial lcdDisaply;
IPAddress dns_server(8, 8, 8, 8);  // Google DNS Server IP...  

/* Create Servo Objects */
Servo steeringServo;
Servo throttleServo;
Servo rearBrakeServo;
#pragma endregion

/* ---------------------------------------------------------------------------------------------------------------------------
-------------------------------    Global Variables To Keep Track Of Electrical States   -------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
/* Steering */
bool isSteeringFullLockRight = false;
bool isSteeringFullLockLeft = false;
bool isSteeringDeadCenter = false;
/* Lighting */
bool isFrontLeftHeadLightOn = false;
bool isFrontRightHeadLightOn = false;
bool isRearHeadLightOn = false;
bool isLeftBlinkerEnabled = false;
bool isRightBlinkerEnabled = false;

/* These two values specify the fixed full lock values for push button left and right turning, currently deprecated... */
int steeringLeftFullLockManual = 40;
int steeringRightFullLockManual = -40;

int defaultSteeringVal = 90;
int steeringCenterVal = 90;
int steeringTurnRightIncrementVal = -15;
int steeringTurnLeftIncrementVal = 15;
int globalSteeringTurningValue;
int zeroEngineThrottleValue = 0;

/* unproccessed axis values from steering wheel */
float controllerAxis0Raw; // ranges between turning full lock left and full lock right
float controllerAxis1Raw; // foward acceleration
float controllerAxis2Raw; // backwards acceleration

/* These values define the absolute limit that the steering column servo can move too, these can be configured remotely to, TODO: add these to EEPROM*/
int steeringLeftConstraint = 45;
int steeringRightConstraint = 135;

/* Time management */
unsigned long previousMillisHeadLightBlinker = 0;
long blinkerInterval = 1000; // interval at which to blilk blinkers
unsigned long currentMillis = millis();

/* Ping Pong */
int pingPongCounter = 0;
long pingPongInterval = 0;
long pingPongDelayAllowence = 0;
bool pingPongEnabled = false;
unsigned long pingPongPreviousMillis = 0;
unsigned long pingPongCheckerPreviousMillis = 0;
bool ballInCourt = true;
String pingCharRef = "p";
long pingPongDelay;
#pragma endregion

/* ---------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------    Setup Function   ------------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
void setup() {
	/* We must set the spark plug relay output to LOW on boot to prevent engine from starting without warning */
	pinMode(ENGINE_SPARK_PLUG_PIN, OUTPUT);
	digitalWrite(ENGINE_SPARK_PLUG_PIN, LOW);
	Serial.begin(SERIAL_MONITOR_BAUD_RATE);

	Serial.print("AQB (Automated Quad Bike) by LDPrice 2020 all Systems Starting...");

	Serial.println("██╗░░░░░██████╗░██████╗░██████╗░██╗░█████╗░███████╗     ░██████╗░█████╗░███████╗████████╗░██╗░░░░░░░██╗░█████╗░██████╗░███████╗");
	Serial.println("██║░░░░░██╔══██╗██╔══██╗██╔══██╗██║██╔══██╗██╔════╝     ██╔════╝██╔══██╗██╔════╝╚══██╔══╝░██║░░██╗░░██║██╔══██╗██╔══██╗██╔════╝");
	Serial.println("██║░░░░░██║░░██║██████╔╝██████╔╝██║██║░░╚═╝█████╗░░     ╚█████╗░██║░░██║█████╗░░░░░██║░░░░╚██╗████╗██╔╝███████║██████╔╝█████╗░░");
	Serial.println("██║░░░░░██║░░██║██╔═══╝░██╔══██╗██║██║░░██╗██╔══╝░░     ░╚═══██╗██║░░██║██╔══╝░░░░░██║░░░░░████╔═████║░██╔══██║██╔══██╗██╔══╝░░");
	Serial.println("███████╗██████╔╝██║░░░░░██║░░██║██║╚█████╔╝███████╗     ██████╔╝╚█████╔╝██║░░░░░░░░██║░░░░░╚██╔╝░╚██╔╝░██║░░██║██║░░██║███████╗");
	Serial.println("╚══════╝╚═════╝░╚═╝░░░░░╚═╝░░╚═╝╚═╝░╚════╝░╚══════╝     ╚═════╝░░╚════╝░╚═╝░░░░░░░░╚═╝░░░░░░╚═╝░░░╚═╝░░╚═╝░░╚═╝╚═╝░░╚═╝╚══════╝");

	Serial.println("Starting Software Serial...");
	lcdDisaply.begin(SS_BAUD_RATE, SWSERIAL_8N1, SS_IN, SS_OUT, false, 95, 11);
	lcdDisaply.write('a'); // Other Arduino board will see this and display startup screen...
	delay(1000);

	/* Set the callback function that enters AP mode when no known WiFi networks are found */
	wifiManager.setAPCallback(configModeCallback);

	/* Attach Servos */
	Serial.println("Inititializing I/O...");
	steeringServo.attach(STEERING_SERVO_PIN);
	throttleServo.attach(THROTTLE_SERVO_PIN);
	rearBrakeServo.attach(REAR_BRAKE_SERVO_PIN);
	throttleServo.write(zeroEngineThrottleValue);

	/* Set Relay Outputs */
	pinMode(STARTER_MOTOR_RELAY_PIN, OUTPUT);
	AQB_SYS_SHUTDOWN();
	pinMode(FRONT_LEFT_HEAD_LIGHT_PIN, OUTPUT);
	pinMode(FRONT_RIGHT_HEAD_LIGHT_PIN, OUTPUT);
	pinMode(REAR_HEAD_LIGHT_PIN, OUTPUT);
	pinMode(WIFI_TRIGGER_PIN, INPUT);
	pinMode(RED_LED_RGB_PIN, OUTPUT);

	EEPROM.begin(EEPROM_SIZE);

	client.setServer(MQTT_SERVER, MQTT_PORT);
	client.setCallback(callback);

	/* Read EEPROM Values */
	Serial.println("Reading EEPROM Values...");
	blinkerInterval = EEPROM.read(0);
	blinkerInterval = blinkerInterval * 1000;

	Serial.println("Connecting to WiFi...");
	/* If no known WiFi networks are found, enter config AP portal mode... */
	if (!wifiManager.autoConnect(PORTAL_SSID, WIFI_PORTAL_PASS)) {
		Serial.println("failed to connect and hit timeout");
		/* Reset and try again */
		AQB_SYS_SHUTDOWN();
		lcdDisaply.write('f'); // Other Arduino board will see this and display fail screen...
		lcdDisaply.write('f'); // Other Arduino board will see this and display fail screen...
		ESP.restart();
		delay(1000);
	}

	//if you get here you have connected to the WiFi
	Serial.println("connected...yeey :)");
	lcdDisaply.write('w'); // Other Arduino board will see this and display WiFi connected screen...

}

/* ---------------------------------------------------------------------------------------------------------------------------
--------------------------------------------------    Main Loop Function   ---------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
void loop() {
	if (!client.connected()) {
		Serial.println("Not connected to MQTT, trying to connect to broker...");
		reconnect();
	}
	else {
		client.loop();
	}

	currentMillis = millis();
	if (pingPongEnabled == true) {
		if (ballInCourt == true) {
			if (currentMillis - pingPongPreviousMillis >= pingPongInterval) {
				// save the last time we checked the ping pong status
				pingPongPreviousMillis = currentMillis;
				pingPongCheckerPreviousMillis = currentMillis;
				Serial.println("Pong! Ball existing court...");
				toggleLED(); // flash LED
				client.publish("/AQB/out/pp", pingCharRef.c_str());
				ballInCourt = false;

			}
		}
		else {
			if (currentMillis - pingPongCheckerPreviousMillis >= pingPongDelayAllowence) {
				Serial.println("Pong did not return in time! Shutting down system...");
				AQB_SYS_SHUTDOWN();
				pingPongEnabled = false;
				lcdDisaply.write('7'); // Other Arduino board will see this and display Control Panel FAIL screen...
				lcdDisaply.write('7'); // Other Arduino board will see this and display Control Panel FAIL screen...
			}

		}

	}
	if (currentMillis - previousMillisHeadLightBlinker >= blinkerInterval) {
		// save the last time we checked the blinker status
		previousMillisHeadLightBlinker = currentMillis;

		if (isLeftBlinkerEnabled == true) {
			leftHeadLightToggle();
		}
		if (isRightBlinkerEnabled == true) {
			rightHeadLightToggle();
		}
	}

	if (digitalRead(WIFI_TRIGGER_PIN) == LOW) {
		WiFi.disconnect(true);

		//WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.11883
		WiFi.mode(WIFI_STA);

		if (!wifiManager.startConfigPortal("AQB WiFi Manual", "CiscoAQB123")) {
			Serial.println("failed to connect and hit timeout");
			delay(3000);
			//reset and try again, or maybe put it to deep sleep
			ESP.restart();
			delay(5000);
		}

		//if you get here you have connected to the WiFi
		Serial.println("connected...yeey :)");
	}

}


/* ---------------------------------------------------------------------------------------------------------------------------
--------------------------------------    Networking Callback and Reconnect Functions   --------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
void callback(char* topic, byte* payload, unsigned int length) {
	Serial.print("Topic is:");
	Serial.println(topic);
	if (strcmp(topic, "/AQB/in/pp") == 0) {
		Serial.println("Ping recieved");
		ballInCourt = true;
		toggleLED(); // flash LED
		lcdDisaply.write('9'); // Other Arduino board will see this and display PINGPONG screen...
		pingPongDelay = currentMillis - pingPongCheckerPreviousMillis;
		Serial.print("Delay was: ");
		Serial.println(pingPongDelay);
		client.publish("/AQB/out/delay", String(pingPongDelay).c_str());
	}
	else if (strcmp(topic, "/AQB/in/npps") == 0) {
		Serial.println("New Session Requested!");

		String json_temp = String((char*)payload);
		Serial.print("Raw JSON Data: ");
		Serial.println(json_temp);
		
		DynamicJsonBuffer  jsonBuffer(200);

		JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
		if (!json_obj.success()) {
			Serial.println("parseObject() failed for new_ping_pong_session");
			return;
		}

		/***      current reading / sending speed:    ***/
		pingPongInterval = int(json_obj["i"]);
		pingPongDelayAllowence = int(json_obj["a"]);

		Serial.println("New Ping Pong session");
		Serial.print("pingpong_interval: ");
		Serial.println(pingPongInterval);
		Serial.print("pingpong_delay_allowance: ");
		Serial.println(pingPongDelayAllowence);
		pingPongEnabled = true;
		ballInCourt = true;
		pingPongCounter = 0;  // not used as of yet...

	}

	else if (strcmp(topic, "/AQB/in") == 0) {
		Serial.print("Topic match 'in'");
		if (payload[0] == 'l') {
			Serial.println("Going Left!");
			steeringManualLeft();
		}
		else if (payload[0] == 'r') {
			Serial.println("Going Right!");
			steeringManualRight();
		}
		else if (payload[0] == 'c') {
			Serial.println("Returning to centre!");
			steeringManualCenter();
		}
		else if (payload[0] == 'w') {
			Serial.println("Sending WiFi stats!");
			sendWifiDetails();
		}
		else if (payload[0] == 'n') {
			Serial.println("Toggling left blinker!");
			toggleLeftBlinkers();
		}
		else if (payload[0] == 'm') {
			Serial.println("Toggling right blinker!");
			toggleRightBlinkers();
		}
		else if (payload[0] == 'a') {
			Serial.println("Toggling front lights");
			rightHeadLightToggle();
			leftHeadLightToggle();
		}
		else if (payload[0] == 'b') {
			Serial.println("Toggling rear light");
			rearLightToggle();
		}
		else if (payload[0] == 'e') {
			Serial.println("saving current blinker interval to eeprom!");
			wirteBlinkerIntervalEEPROM();
		}
		else if (payload[0] == 'j') {
			Serial.println("Resseting WiFi Manager creds...");
			wifiManager.resetSettings();
		}
		else if (payload[0] == '4') {
			Serial.println("Control Panel Connected (display to user)...");
			lcdDisaply.write('2'); // Other Arduino board will see this and display Control Panel connected screen...
			lcdDisaply.write('2'); // Other Arduino board will see this and display Control Panel connected screen...
			digitalWrite(RED_LED_RGB_PIN, LOW);
		}
	}
	else if (strcmp(topic, "/AQB/in/swaN") == 0) {
		Serial.println("Updated Steering WHeel Allignment!");
		Serial.print("Old Right Axis val: ");
		Serial.println(steeringRightConstraint);
		Serial.print("Old Left Axis val: ");
		Serial.println(steeringLeftConstraint);

		String json_temp = String((char*)payload);
		Serial.print("Raw JSON Data: ");
		Serial.println(json_temp);
		DynamicJsonBuffer  jsonBuffer(200);
		JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
		if (!json_obj.success()) {
			Serial.println("parseObject() failed for blinker_interval");
			return;
		}

		/***      current reading / sending speed:    ***/
		int tempSteeringRightConstraint = int(json_obj["r"]);
		int tempSteeringLeftConstraint = int(json_obj["l"]);

		Serial.print("NEW Right Axis val: ");
		Serial.println(tempSteeringRightConstraint);
		Serial.print("NEW Left Axis val: ");
		Serial.println(tempSteeringLeftConstraint);

		/* Range check below for safety... */
		if (tempSteeringRightConstraint > 145) {
			Serial.println("temp_steering_right_max TOO LARGE FAIL");
			return;
		}
		else if (tempSteeringRightConstraint < 90) {
			Serial.println("temp_steering_right_max TOO SMALL FAIL");
			return;
		}
		if (tempSteeringLeftConstraint < 45) {
			Serial.println("temp_steering_left_max TOO SMALL FAIL");
			return;
		}
		else if (tempSteeringLeftConstraint > 90) {
			Serial.println("temp_steering_left_max TOO LARGE FAIL");
			return;
		}
		steeringRightConstraint = tempSteeringRightConstraint;
		steeringLeftConstraint = tempSteeringLeftConstraint;
		Serial.print("Steering Allignment completed!");

	}
	else if (strcmp(topic, "/AQB/in/tu") == 0) {
		Serial.println("Tming Update topic in");
		Serial.print("Old blinker update timing val: ");
		Serial.println(blinkerInterval);

		Serial.print("recieving new timings values'");
		String json_temp = String((char*)payload);
		Serial.print("Raw JSON Data: ");
		Serial.println(json_temp);

		DynamicJsonBuffer  jsonBuffer(200);

		JsonObject& json_obj = jsonBuffer.parseObject(json_temp);
		if (!json_obj.success()) {
			Serial.println("parseObject() failed for blinker_interval");
			return;
		}

		/***      current reading / sending speed:    ***/
		blinkerInterval = int(json_obj["b"]);
		Serial.print("New blinker update timing val: ");
		Serial.println(blinkerInterval);
	}

	else if (strcmp(topic, "/AQB/in/SW_btn_d") == 0) {
		char button_temp = payload[0];
		Serial.println("butn DOWN topic");
		Serial.println(button_temp);
		if (button_temp == '4') {
			applyBrakes();
		}
		else if (button_temp == '5') {
			crankEngine();
		}
		else if (button_temp == '6') {
			leftHeadLightToggle();
			rightHeadLightToggle();
		}
		else if (button_temp == 'k') {
			rearLightToggle();
		}
		else if (button_temp == '7') {
			enableSparkPlug();
		}
		else if (button_temp == 's') {
			disableSparkPlug();
		}
	}
	else if (strcmp(topic, "/AQB/in/SW_btn_u") == 0) {
		Serial.println("butn up topic");
		char button_temp = payload[0];
		Serial.println(button_temp);
		if (button_temp == '4') {
			disableBrakes();
		}
		else if (button_temp == '5') {
			stopEngineCrank();
		}
	}
	else if (strcmp(topic, "/AQB/in/SW_axs_0") == 0) {
		String axs_0_temp = String((char*)payload);
		controllerAxis0Raw = axs_0_temp.toInt();
		controllerAxis0Raw = map(controllerAxis0Raw, -10, 10, steeringRightConstraint, steeringLeftConstraint);
		Serial.println(controllerAxis0Raw);
		steeringServo.write(controllerAxis0Raw);
	}
	else if (strcmp(topic, "/AQB/in/SW_axs_1") == 0) {
		String axs_1_temp = String((char*)payload);
		controllerAxis1Raw = axs_1_temp.toInt();
		controllerAxis1Raw = map(controllerAxis1Raw, -10, 10, 180, 0);
		Serial.println(controllerAxis1Raw);
		throttleServo.write(controllerAxis1Raw);
	}
	else if (strcmp(topic, "/AQB/in/SW_axs_2") == 0) {
		String axs_2_temp = String((char*)payload);
		controllerAxis2Raw = axs_2_temp.toInt();
		controllerAxis2Raw = map(controllerAxis2Raw, -10, 10, 180, 0);
		Serial.println(controllerAxis2Raw);
		throttleServo.write(controllerAxis2Raw);
	}
	else {
		Serial.print("Topic not relevant!");
	}
}


/* This function runs when we are trying to connect to the MQTT server */
void reconnect() {
	// Loop until we're reconnected
	if (!client.connected()) {
		Serial.print("Attempting MQTT connection...");
		// Attempt to connect
		if (client.connect("AQB-ESP32", MQTT_USERNAME, MQTT_PASSWORD)) {
			Serial.println("connected");

			for (int i = 0; i <= amountOfTopics; i++)
			{
				client.subscribe(mqttSubscriptions[i]);
			}

			sendWifiDetails();
			digitalWrite(RED_LED_RGB_PIN, LOW);
			Serial.println("MQTT Connected. Yayyy!");
			delay(2000);
			lcdDisaply.write('m'); // Other Arduino board will see this and display MQTT Connected screen...
			lcdDisaply.write('m'); // Other Arduino board will see this and display MQTT Connected screen...
		}
		else {
			Serial.print("failed to connect to MQTT! rc=");
			Serial.print(client.state());
			AQB_SYS_SHUTDOWN();
			Serial.println("try again in 1 seconds");
			lcdDisaply.write('g'); // Other Arduino board will see this and display MQTT FAIL screen...
			lcdDisaply.write('g'); // Other Arduino board will see this and display MQTT FAIL screen...
			// Wait 2 seconds before retrying
			delay(3000);
		}
	}
}

//gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager* myWiFiManager) {
	Serial.println("Entered config mode");
	//WiFi.disconnect(true);
	Serial.println(WiFi.softAPIP());
	//if you used auto generated SSID, print it
	Serial.println(myWiFiManager->getConfigPortalSSID());
	// Make LED turn on representing we are in WiFi config mode!
	// red_led_ticker.attach(.2, red_led_tick_func);
	lcdDisaply.write('p');
	lcdDisaply.write('p');
}
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
--------------------------------------------    Hardware Interaction Functions   ---------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */
#pragma region
void initializePingPongProtocol(void)
{
	Serial.println("initialize_ping_pong Function Running...");

}
void sendWifiDetails(void)
{
	Serial.println("Sending WiFi Function running...");
	Serial.println(WiFi.SSID());
	client.publish("/AQB/out/ssid", (char*)WiFi.SSID().c_str());
}
/* The following 3 functions are not yet used, but they are for turning steering column to a fixed pos when user is using a 
keyboard instead of a steering wheel joystick. Will need to be implemented with some sort of time delay to change turning 
rate like a video game would. */
void steeringManualCenter(void)
{
	Serial.println("turn_centre Function Running...");
	steeringServo.write(steeringCenterVal);
}
void steeringManualRight(void)
{
	Serial.println("turn_right Function Running...");
	globalSteeringTurningValue = steeringCenterVal + steeringRightFullLockManual;
	Serial.println("max_temp = " + globalSteeringTurningValue);
	defaultSteeringVal = defaultSteeringVal + steeringTurnRightIncrementVal;
	Serial.println("steering_val = " + defaultSteeringVal);

}
void steeringManualLeft(void)
{
	Serial.println("turn_left Function Running...");
	globalSteeringTurningValue = steeringCenterVal + steeringLeftFullLockManual;
	Serial.println("max_temp = " + globalSteeringTurningValue);
	defaultSteeringVal = defaultSteeringVal + steeringTurnLeftIncrementVal;
	Serial.println("steering_val = " + defaultSteeringVal);
}

/* The next few functions control the light relays */
void leftHeadLightToggle(void)
{
	Serial.println("leftHeadLightToggle function running");
	if (isFrontLeftHeadLightOn == false) {
		isFrontLeftHeadLightOn = true;
		Serial.println("left_light ON");
		digitalWrite(FRONT_LEFT_HEAD_LIGHT_PIN, HIGH);
	}
	else {
		isFrontLeftHeadLightOn = false;
		Serial.println("left_light OFF");
		digitalWrite(FRONT_LEFT_HEAD_LIGHT_PIN, LOW);
	}
}
void rightHeadLightToggle(void)
{
	Serial.println("rightHeadLightToggle function running");
	if (isRearHeadLightOn == false) {
		isRearHeadLightOn = true;
		Serial.println("right_light ON");
		digitalWrite(FRONT_RIGHT_HEAD_LIGHT_PIN, HIGH);
	}
	else {
		isRearHeadLightOn = false;
		Serial.println("right_light OFF");
		digitalWrite(FRONT_RIGHT_HEAD_LIGHT_PIN, LOW);
	}
}
void rearLightToggle(void)
{
	Serial.println("rear_light_toggle");
	if (isRearHeadLightOn == false) {
		isRearHeadLightOn = true;
		Serial.println("rear light ON");
		digitalWrite(REAR_HEAD_LIGHT_PIN, HIGH);
	}
	else {
		isRearHeadLightOn = false;
		Serial.println("rear light OFF");
		digitalWrite(REAR_HEAD_LIGHT_PIN, LOW);
	}
}
void toggleLeftBlinkers(void)
{
	if (isLeftBlinkerEnabled == false) {
		Serial.println("Left blinker turning on");
		isLeftBlinkerEnabled = true;
	}
	else {
		Serial.println("Left blinker turning off");
		isLeftBlinkerEnabled = false;
		digitalWrite(FRONT_LEFT_HEAD_LIGHT_PIN, LOW);
		isFrontLeftHeadLightOn = false;
	}
}

void toggleRightBlinkers(void)
{
	if (isRightBlinkerEnabled == false) {
		Serial.println("Right blinker turning on");
		isRightBlinkerEnabled = true;
	}
	else {
		isRightBlinkerEnabled = false;
		Serial.println("Right blinker turning off");
		digitalWrite(FRONT_RIGHT_HEAD_LIGHT_PIN, LOW);
		isFrontRightHeadLightOn = false;
	}
}

/* Kill the lights */
void haltLightSystem(void)
{
	/* Turn off blinkers */
	isRightBlinkerEnabled = false;
	isLeftBlinkerEnabled = false;
	/* Turn off light relay pins */
	digitalWrite(FRONT_LEFT_HEAD_LIGHT_PIN, LOW);
	digitalWrite(FRONT_RIGHT_HEAD_LIGHT_PIN, LOW);
	digitalWrite(REAR_HEAD_LIGHT_PIN, LOW);
	isFrontLeftHeadLightOn = false;
	isFrontRightHeadLightOn = false;
	isRearHeadLightOn = false;
}

/* The next two functions interact with the gearbox, this has not yet been completed on the hardware side. */
void shiftGearUp(void)
{
	Serial.println("shiftGearUp");
}
void shiftGearDown(void)
{
	Serial.println("shiftGearDown");
}

/* The next two function control the breaking system, this has not yet been implemented in the hardware. */
void applyBrakes(void)
{
	Serial.println("applyBrakes");
	int brake_val = 180;
	rearBrakeServo.write(brake_val);
}
void disableBrakes(void)
{
	Serial.println("disableBrakes");
	int brake_val = 0;
	rearBrakeServo.write(brake_val);
}

/* These two functions will enable and disable the starter motor relay*/
void crankEngine(void)
{
	Serial.println("crankEngine");
	digitalWrite(STARTER_MOTOR_RELAY_PIN, LOW);
}
void stopEngineCrank(void)
{
	Serial.println("stopEngineCrank");
	digitalWrite(STARTER_MOTOR_RELAY_PIN, HIGH);
}

/* The next two functions connect and disconnect the spark plug from its electrical source */
void enableSparkPlug(void)
{
	Serial.println("enableSparkPlug");
	digitalWrite(ENGINE_SPARK_PLUG_PIN, HIGH);
}
void disableSparkPlug(void)
{
	Serial.println("disableSparkPlug");
	digitalWrite(ENGINE_SPARK_PLUG_PIN, LOW);
}

/* The next three functions simply inform the Control Panel that full lock has been reached on steering. */
void steeringLeftFullLockReached() {
	client.publish("/AQB/out", "max_l");
}
void steeringRightFullLockReached() {
	client.publish("/AQB/out", "max_r");
}
#pragma endregion


/* ---------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------    Misc Functions   ---------------------------------------------------
--------------------------------------------------------------------------------------------------------------------------- */ 
#pragma region
void wirteBlinkerIntervalEEPROM(void)
{
	Serial.println("wirteBlinkerIntervalEEPROM");
	float temp = blinkerInterval / 1000;
	int rounded_val = round(temp);
	// diag: Serial.print("Saving rouded value as: ");
	Serial.println(rounded_val);
	EEPROM.write(0, temp);
	EEPROM.commit();
}

void toggleLED()
{
	int state = digitalRead(RED_LED_RGB_PIN);  // get the current state of GPIO pin
	digitalWrite(RED_LED_RGB_PIN, !state);     // set pin to the opposite state
}

/* Turn off the petrol engine */
void AQB_SYS_SHUTDOWN(void)
{
	Serial.println("AQB_SYS_SHUTDOWN");
	digitalWrite(ENGINE_SPARK_PLUG_PIN, LOW); // disable spark plug
	throttleServo.write(zeroEngineThrottleValue); // disable throttle servo
	digitalWrite(STARTER_MOTOR_RELAY_PIN, HIGH); // disable starter motor
	haltLightSystem(); // turn off lights
}
#pragma endregion












