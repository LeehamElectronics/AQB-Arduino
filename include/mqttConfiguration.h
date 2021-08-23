#pragma once
// mqttConfiguration.h
#define MQTT_SERVER "you IP address or Domain Name"
#define MQTT_PORT 1883  // Could be anything
#define MQTT_USERNAME "username_here"
#define MQTT_PASSWORD "password_here"  // Use a strong password, or don't

/* Define all of our MQTT topics here */
char mqttTopic1[] = "/AQB/in";
char mqttTopic2[] = "/AQB/in/SW_btn_d";
char mqttTopic3[] = "/AQB/in/SW_btn_u";
char mqttTopic4[] = "/AQB/in/SW_axs_0";
char mqttTopic5[] = "/AQB/in/SW_axs_1";
char mqttTopic6[] = "/AQB/in/SW_axs_2";
char mqttTopic7[] = "/AQB/in/swaN";
char mqttTopic8[] = "/AQB/in/tu";
char mqttTopic9[] = "/AQB/in/npps";
char mqttTopic10[] = "/AQB/in/pp";
char mqttTopic11[] = "/AQB/irrelevent";

/* Append all of our topics into an array of char arrays, sounds confusing right? */
char* mqttSubscriptions[] =
{
	mqttTopic1,
	mqttTopic2,
	mqttTopic3,
	mqttTopic4,
	mqttTopic5,
	mqttTopic6,
	mqttTopic7,
	mqttTopic8,
	mqttTopic9,
	mqttTopic10,
	mqttTopic11,
};

int amountOfTopics = 10;