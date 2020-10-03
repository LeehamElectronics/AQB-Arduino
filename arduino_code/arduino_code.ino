/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                                      PLEASE READ THE FOLLOWING!                                                         *
 *                               This open source software was written by Liam Price 2020 and is FREE to use...                            *
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
 * And if you need help, feel free to email me at liamisprice@gmail.com                                                                    *
 * Thanks!                                                                                                                                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 /*

 ██╗░░░░░███████╗███████╗██╗░░██╗░█████╗░███╗░░░███╗██╗░██████╗      ░██████╗░█████╗░███████╗████████╗░██╗░░░░░░░██╗░█████╗░██████╗░███████╗
 ██║░░░░░██╔════╝██╔════╝██║░░██║██╔══██╗████╗░████║╚█║██╔════╝      ██╔════╝██╔══██╗██╔════╝╚══██╔══╝░██║░░██╗░░██║██╔══██╗██╔══██╗██╔════╝
 ██║░░░░░█████╗░░█████╗░░███████║███████║██╔████╔██║░╚╝╚█████╗░      ╚█████╗░██║░░██║█████╗░░░░░██║░░░░╚██╗████╗██╔╝███████║██████╔╝█████╗░░
 ██║░░░░░██╔══╝░░██╔══╝░░██╔══██║██╔══██║██║╚██╔╝██║░░░░╚═══██╗      ░╚═══██╗██║░░██║██╔══╝░░░░░██║░░░░░████╔═████║░██╔══██║██╔══██╗██╔══╝░░
 ███████╗███████╗███████╗██║░░██║██║░░██║██║░╚═╝░██║░░░██████╔╝      ██████╔╝╚█████╔╝██║░░░░░░░░██║░░░░░╚██╔╝░╚██╔╝░██║░░██║██║░░██║███████╗
 ╚══════╝╚══════╝╚══════╝╚═╝░░╚═╝╚═╝░░╚═╝╚═╝░░░░░╚═╝░░░╚═════╝░     ╚═════╝░░╚════╝░╚═╝░░░░░░░░╚═╝░░░░░░╚═╝░░░╚═╝░░╚═╝░░╚═╝╚═╝░░╚═╝╚══════╝

 */

 /* Here are all the libraries we will use to write our program */
#include <WiFi.h>
#include <WiFiMulti.h> /* Using this library we can allow the AQB to connect to a list of AP's depending on their Wi-Fi strength */
/*
 * This 'PubSubClient' is actually the MQTT Arduino Library that we are using to make the Arduino
 * board communicate with the Control Panel I made, you might be wondering, why is it called
 * 'PubSubClient' and not ArduinoMQTT? Well there is a more 'official' Arduino MQTT library available,
 * however it is very new at the time of writing and I prefer using these more 'established' libraries
 * for stability.
 */
#include <PubSubClient.h> 
 /* #include <ESP32_Servo.h> having issues? Use the following ESP32 Library instead: */
#include <ESP32Servo.h> /* Get it from https://github.com/madhephaestus/ESP32Servo */
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <Arduino.h> /* Surprising? */
#include <analogWrite.h> /* this was required for ESP32 chips to use analogWrite for some reason, look it up */
/*
 * Used to store information even when the robot is turned off, dont abuse it, only has a set
 * amount of read / write cycles. If you need to use EEPROM A LOT try using a SD Card instead!
 */
#include <EEPROM.h> 
/* This is a very important library and takes a lot to learn properly, definitely worth it though, here is a link for it: */
/* https://github.com/bblanchon/ArduinoJson */
#include <ArduinoJson.h> 

 /* EEPROM Flash Memory setup: */
#define EEPROM_SIZE 17 // don't touch unless your adding more EEPROM functionality...

WiFiMulti wifiMulti;

/* MQTT Variables Below, make sure to fill them in with your MQTT credentials, or it won't work! */
#define mqtt_user "USERNAME"
#define mqtt_password "USE_A_STRONG_PASSWORD"
/* This is the IP address or hostname of your MQTT server... */
const char* mqtt_server = "REDACTED";

WiFiClient espClient;
PubSubClient client(espClient);

/* ESC pin setup code */
Servo steering_servo;
Servo throttle_servo;
Servo rear_brake_servo;

/*
 * 
 * Pin Out Menu 
 * 
 * 
/* Relay pinsouts */
#define starter_motor_relay 16
#define engine_spark_plug_relay 17
#define front_light_left 33
#define front_light_right 32
#define rear_light 27
/*
	Here for easy reference when wiring up circuits:
	steering_servo = 18
	throttle_servo = 19
	rear_brake_servo = 20
*/

/* define limit switch control pins */
boolean is_full_lock_right = false;
boolean is_full_lock_left = false;
boolean is_centre_lock = false;
boolean is_light_on = false;
boolean is_rear_light_on = false;
int steering_val = 90;
int max_left = 40;
int max_right = -40;
int centre_val = 90;
int increment_val_right = -15;
int increment_val_left = 15;
int max_temp;
int no_throttle_val = 0;
float axs_0_value;
float axs_1_value;
float axs_2_value;
int steering_left_max = 45;
int steering_right_max = 135;

void setup() {
	Serial.begin(9600);
	delay(10);
	/*
	* Replace your network information here, you can add as many networks
	* as you want and the strongest signal will be automatically chosen
	* for you! How cool is that?
	*/
	wifiMulti.addAP("SSID", "PASSWORD");
	wifiMulti.addAP("SSID", "PASSWORD");
	wifiMulti.addAP("SSID", "PASSWORD");

	Serial.println("Connecting Wifi...");
	if (wifiMulti.run() == WL_CONNECTED) {
		Serial.println("");
		Serial.println("WiFi connected");
		Serial.println("IP address: ");
		Serial.println(WiFi.localIP());
	}

	steering_servo.attach(18);
	throttle_servo.attach(19);
	rear_brake_servo.attach(20);
	throttle_servo.write(no_throttle_val);
	pinMode(starter_motor_relay, OUTPUT);
	pinMode(engine_spark_plug_relay, OUTPUT);
	pinMode(front_light_left, OUTPUT);
	pinMode(front_light_right, OUTPUT);
	pinMode(rear_light, OUTPUT);
	stop_engine_crank();

	client.setServer(mqtt_server, 53);
	client.setCallback(callback);
}

/********************************** RECEIVE DATA FROM MQTT ******************************************/
void callback(char* topic, byte* payload, unsigned int length) {
	Serial.print("Topic is:");
	Serial.println(topic);
	//Serial.print("Payload is:");
	//Serial.println(payload);
	if (strcmp(topic, "/AQB/in") == 0) {
		Serial.print("Topic match 'in'");
		if (payload[0] == 'l') {
			Serial.println("Going Left!");
			turn_left();
		}
		else if (payload[0] == 'r') {
			Serial.println("Going Right!");
			turn_right();
		}
		else if (payload[0] == 'c') {
			Serial.println("Returning to centre!");
			turn_centre();
		}
	}
	else if (strcmp(topic, "/AQB/in/swal") == 0) {
		Serial.println("Updated Left Steering WHeel Axis!");
		Serial.print("Old Left Axis val: ");
		Serial.println(steering_left_max);
		String temp_swal = String((char*)payload);
		steering_left_max = temp_swal.toInt();
		Serial.print("NEW Left Axis val: ");
		Serial.println(steering_left_max);
		String((char*)payload) = "";
	}
	else if (strcmp(topic, "/AQB/in/swar") == 0) {
		Serial.println("Updated Right Steering WHeel Axis!");
		Serial.print("Old LRight Axis val: ");
		Serial.println(steering_right_max);
		String temp_swar = String((char*)payload);
		steering_right_max = temp_swar.toInt();
		Serial.print("NEW Right Axis val: ");
		Serial.println(steering_right_max);
		String((char*)payload) = "";
	}
	else if (strcmp(topic, "/AQB/in/SW_btn_d") == 0) {
		char button_temp = payload[0];
		Serial.println("butn DOWN topic");
		Serial.println(button_temp);
		if (button_temp == '4') {
			apply_brakes();
		}
		else if (button_temp == '5') {
			crank_engine();
		}
		else if (button_temp == '6') {
			light_toggle();
		}
		else if (button_temp == 'k') {
			rear_light_toggle();
		}
		else if (button_temp == '7') {
			connect_spark_plug();
		}
		else if (button_temp == 's') {
			disconnect_spark_plug();
		}
	}
	else if (strcmp(topic, "/AQB/in/SW_btn_u") == 0) {
		Serial.println("butn up topic");
		char button_temp = payload[0];
		Serial.println(button_temp);
		if (button_temp == '4') {
			remove_brakes();
		}
		else if (button_temp == '5') {
			stop_engine_crank();
		}
	}
	else if (strcmp(topic, "/AQB/in/SW_axs_0") == 0) {
		String axs_0_temp = String((char*)payload);
		axs_0_value = axs_0_temp.toInt();
		axs_0_value = map(axs_0_value, -10, 10, steering_right_max, steering_left_max);
		Serial.println(axs_0_value);
		steering_servo.write(axs_0_value);
	}
	else if (strcmp(topic, "/AQB/in/SW_axs_1") == 0) {
		String axs_1_temp = String((char*)payload);
		axs_1_value = axs_1_temp.toInt();
		axs_1_value = map(axs_1_value, -10, 10, 180, 0);
		Serial.println(axs_1_value);
		throttle_servo.write(axs_1_value);
	}
	else if (strcmp(topic, "/AQB/in/SW_axs_2") == 0) {
		String axs_2_temp = String((char*)payload);
		axs_2_value = axs_2_temp.toInt();
		axs_2_value = map(axs_2_value, -10, 10, 180, 0);
		Serial.println(axs_2_value);
		throttle_servo.write(axs_2_value);
	}
	else {
		Serial.print("Topic not relevant!");
	}
}
/********************************** RECONNECT TO MQTT ******************************************/
void reconnect() {
	// Loop until we're reconnected
	if (!client.connected()) {
		Serial.print("Attempting MQTT connection...");
		// Attempt to connect
		if (client.connect("AQB-ESP32", mqtt_user, mqtt_password)) {
			Serial.println("connected");
			client.subscribe("/AQB/in");
			client.subscribe("/AQB/in/SW_btn_d");
			client.subscribe("/AQB/in/SW_btn_u");
			client.subscribe("/AQB/in/SW_axs_0");
			client.subscribe("/AQB/in/SW_axs_1");
			client.subscribe("/AQB/in/SW_axs_2");
			client.subscribe("/AQB/in/swal");
			client.subscribe("/AQB/in/swar");
			client.subscribe("/AQB/irrelevent");
		}
		else {
			Serial.print("failed, rc=");
			Serial.print(client.state());
			Serial.println(" try again in 1 second");
			// Wait 2 seconds before retrying
			delay(2000);
		}
	}
}

void loop() {
	if (wifiMulti.run() != WL_CONNECTED) {
		Serial.println("WiFi not connected!");
		delay(1000);
	}
	if (wifiMulti.run() == WL_CONNECTED) {
		if (!client.connected()) {
			Serial.println("Not connected to MQTT, trying to connect to broker...");
			reconnect();
		}
		client.loop();
	}
}

/********************************************* RIGHT *****************************************************/
void turn_centre(void)
{
	Serial.println("turn_centre Function Running...");
	steering_val = centre_val;
	steering_servo.write(steering_val);
}
/********************************************* RIGHT *****************************************************/
void turn_right(void)
{
	Serial.println("turn_right Function Running...");
	max_temp = centre_val + max_right;
	Serial.println("max_temp = " + max_temp);
	steering_val = steering_val + increment_val_right;
	Serial.println("steering_val = " + steering_val);

}
/********************************************* CENTRE *****************************************************/
void turn_left(void)
{
	Serial.println("turn_left Function Running...");
	max_temp = centre_val + max_left;
	Serial.println("max_temp = " + max_temp);
	steering_val = steering_val + increment_val_left;
	Serial.println("steering_val = " + steering_val);
}
/********************************************* light_toggle *****************************************************/
void light_toggle(void)
{
	Serial.println("light_toggle");
	if (is_light_on == false) {
		is_light_on = true;
		Serial.println("light ON");
		digitalWrite(front_light_left, HIGH);
		digitalWrite(front_light_right, HIGH);
	}
	else {
		is_light_on = false;
		Serial.println("light OFF");
		digitalWrite(front_light_left, LOW);
		digitalWrite(front_light_right, LOW);
	}
}
/********************************************* rear_light_toggle *****************************************************/
void rear_light_toggle(void)
{
	Serial.println("rear_light_toggle");
	if (is_rear_light_on == false) {
		is_rear_light_on = true;
		Serial.println("rear light ON");
		digitalWrite(rear_light, HIGH);
	}
	else {
		is_rear_light_on = false;
		Serial.println("rear light OFF");
		digitalWrite(rear_light, LOW);
	}
}
/********************************************* up_gear *****************************************************/
void up_gear(void)
{
	Serial.println("up_gear");
}
/********************************************* down_gear *****************************************************/
void down_gear(void)
{
	Serial.println("down_gear");
}
/********************************************* apply_brakes *****************************************************/
void apply_brakes(void)
{
	Serial.println("apply_brakes");
	int brake_val = 180;
	rear_brake_servo.write(brake_val);
}
/********************************************* remove_brakes *****************************************************/
void remove_brakes(void)
{
	Serial.println("remove_brakes");
	int brake_val = 0;
	rear_brake_servo.write(brake_val);
}
/********************************************* crank_engine *****************************************************/
void crank_engine(void)
{
	Serial.println("crank_engine");
	digitalWrite(starter_motor_relay, LOW);
}
/********************************************* stop_engine_crank *****************************************************/
void stop_engine_crank(void)
{
	Serial.println("stop_engine_crank");
	digitalWrite(starter_motor_relay, HIGH);
}
/********************************************* connect_spark_plug *****************************************************/
void connect_spark_plug(void)
{
	Serial.println("connect_spark_plug");
	digitalWrite(engine_spark_plug_relay, HIGH);
}
/********************************************* disconnect_spark_plug *****************************************************/
void disconnect_spark_plug(void)
{
	Serial.println("disconnect_spark_plug");
	digitalWrite(engine_spark_plug_relay, LOW);
}
void left_full_lock() {
	client.publish("/AQB/out", "max_l");
}

void right_full_lock() {
	client.publish("/AQB/out", "max_r");
}

void centre_lock() {
	client.publish("/AQB/out", "max_c");
}