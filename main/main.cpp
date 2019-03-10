#include "HardwareSerial.h"
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_ota_ops.h"

#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "string.h"
#include "SPI.h"
#include "MFRC522.h"					//MFRC522 Custom Library
#include "TFT_eSPI.h"					//TFT_eSPI Custom Library
//#include "FS.h"
#include "SPIFFS.h"
#include <JPEGDecoder.h>
#include "NormScreen.h"
#include "weather.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "weather_jpeg.h"
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include "DHT_U.h"
#include "time.h"
#include "WiFi.h"
#include "wifi_custom.h"

#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "blufi_security.h"
#include "blufi_wireless.h"
#include "esp_bt.h"
#include "HTTPClient.h"
#include "external.h"
#include "edhome_ota.h"
#include "version.h"

static const char *TAG = "BLUEFI_DEMO";
//me NTP Server
#include "Arduino.h"

const char* ntp_server1 = "pool.ntp.org";
const unsigned long hourOffset = 8;

//DHT22 Definition
#define DHTPIN		12
#define DHTTYPE		DHT22
DHT_Unified dht (DHTPIN, DHTTYPE);

//Relay Output Defination
#define RELAY0		22
#define RELAY1		23
#define RELAY2		21
#define RELAY3		19	

//Door Sensor Input Definition
#define INPUT0		25					//Main Door - As long as door open, trigger entrance light. Entrance light will off 30 seconds after door open
#define INPUT1		33					//Windows 1 - Temporary unused
#define INPUT2 		32					//Windows 2 - Temporary unused
#define INPUT3		35					//Windows 3 - Temporary unused

uint8_t relayStatus[4] = {0,0,0,0};

uint8_t relaynum[4] = {22, 23, 21, 19};

#define WEATHER_DATA_RETRIEVAL_PERIOD		60000
unsigned long reference_pressure = 01;
float reference_temperature = 30;
float reference_humidity = 50;
TFT_eSPI_Button key[15];

uint32_t targetTime = 0;

static uint8_t conv2d(const char* p);
uint8_t hh = 0, mm = 0, ss = 0; //Get H, M, S from compile time
int dd = 1, MM = 1, yy = 2018;
unsigned char omm = 99, oss = 99;
static int xcolon = 0, xsecs = 0;
unsigned int colour = 0;

int Hour=12, Minute=0, Second=0;
int SetHourON=0, SetMinuteON=0, SetSecondON=0;
int SetHourOFF=0, SetMinuteOFF=0, SetSecondOFF=0;

int SetHourON1=0, SetMinuteON1=0, SetSecondON1=0;
int SetHourOFF1=0, SetMinuteOFF1=0, SetSecondOFF1=0;

int SetHourON2=0, SetMinuteON2=0, SetSecondON2=0;
int SetHourOFF2=0, SetMinuteOFF2=0, SetSecondOFF2=0;

int SetHourON3=0, SetMinuteON3=0, SetSecondON3=0;
int SetHourOFF3=0, SetMinuteOFF3=0, SetSecondOFF3=0;

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;				//Saved H, M, S x and y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 360, osy = 120, omx = 360, omy = 120, ohx = 360, ohy = 120;	//Saved H, M, S x and y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
boolean initial = 1;

uint32_t icount = 0;

WiFiServer server(80);

float PM2_5 = 0;
float PM10 = 0;

static const int RX_BUF_SIZE 	= 1024;
#define TXD_PIN		(GPIO_NUM_17)
#define RXD_PIN		(GPIO_NUM_16)

static bool isNewMinute(uint8_t chour, uint8_t cminute);

typedef struct 
{
	float 	temperature;
	float	humidity; 
	float	airQuality_2_5; 
	float 	airQuality_10;
	float 	lightIntensity;
}Actual_Measurement_t;
Actual_Measurement_t measurement; 

uint8_t currentPage = 0;
uint8_t outputPage2[4] = {0,0,0,0};					//Connected on local esp32
uint8_t externalOutput[8] = {0,0,0,0,0,0,0,0};		//Bluetooth to remote esp32
char sDate[64];
char sTime[64];

void uart_init()
{
	const uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static uint8_t prevMinute = 0;
void printLocalTime()
{
	struct tm timeinfo;
	if(!getLocalTime(&timeinfo)){
		Serial.println("Failed to obtain time");		
		return;
	}
	Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
	size_t written = strftime(sDate, 64,  "%A, %d %B", &timeinfo);
	Serial.println(sDate);
	written = strftime(sTime, 64,  "%H:%M", &timeinfo);
	Serial.println(sTime);
	hh = (uint8_t)timeinfo.tm_hour;
	mm = (uint8_t)timeinfo.tm_min;
	ss = (uint8_t)timeinfo.tm_sec;
	dd = timeinfo.tm_mday;
	MM = timeinfo.tm_mon;
	yy = timeinfo.tm_year;
}


void showTime(uint32_t msTime)
{
	Serial.print(F(" JPEG drawn in "));
	Serial.print(msTime);
	Serial.println(F(" ms "));
}

static uint8_t inputPin[4] = {INPUT0, INPUT1, INPUT2, INPUT3};
static uint8_t inputState[4] = {0,0,0,0};
static uint8_t inputDebounce[4] = {0,0,0,0};
static uint8_t currentInput[4] = {0,0,0,0}; 
static void ProcessInput(void)
{
	for(int i = 0; i < 4; i++)
	{
		inputState[i] = digitalRead(inputPin[i]);
		if(currentInput[i] != inputState[i])
		{
			inputDebounce[i]++;
			if(inputDebounce[i] >= 5)
			{
				//Detect as actual input change
				currentInput[i] = inputState[i];
				inputDebounce[i] = 0;
			}
		}
	}
}

static void ProcessOutputByTime(void)
{
	//Process Output 0
	if((SetHourON == SetHourOFF) && (SetMinuteOFF == SetMinuteON))							//Do not ON/OFF if on and off time is same
		return;
	else
	{
		if((hh == SetHourON) && (mm == SetMinuteON))
		{
			outputPage2[0] = 1;
		}
		else if ((hh == SetHourOFF) && (mm == SetMinuteOFF))
		{
			outputPage2[0] = 0; 
		}
	}
	
	if((SetHourON1 == SetHourOFF1) && (SetMinuteOFF1 == SetMinuteON1))
		return;
	else
	{
		if((hh == SetHourON1) && (mm == SetMinuteON1))
		{
			outputPage2[1] = 1;
		}
		else if ((hh == SetHourOFF1) && (mm == SetMinuteOFF1))
		{
			outputPage2[1] = 0;
		}
	}
	
	if((SetHourON2 == SetHourOFF2) && (SetMinuteOFF2 == SetMinuteON2))
		return;
	else
	{
		if((hh == SetHourON2) && (mm == SetMinuteON2))
		{
			outputPage2[2] = 1;
		}
		else if ((hh == SetHourOFF2) && (mm == SetMinuteOFF2))
		{
			outputPage2[2] = 0;
		}
	}
	
	if((SetHourON3 == SetHourOFF3) && (SetMinuteOFF3 == SetMinuteON3))
		return;
	else
	{
		if((hh == SetHourON3) && (mm == SetMinuteON3))
		{
			outputPage2[3] = 1;
		}
		else if ((hh == SetHourOFF3) && (mm == SetMinuteOFF3))
		{
			outputPage2[3] = 0;
		}
	}
}

static uint8_t isDayTime(void)			//return true if 0700 to 1800
{
	if(hh > 6 && hh < 19)
		return 1; 
	else 
		return 0;
}

static bool isNewMinute(uint8_t chour, uint8_t cminute)
{
	static uint8_t prevHour = 0;
	static uint8_t prevMinute = 0;

	if(prevHour != chour)
	{
		prevHour = chour; 
		return true;
	}
	else if(prevMinute != cminute)
	{
		prevMinute = cminute;
		return true;
	}
	else
		return false; 
}

static uint8_t prevState[4] = {1,1,1,1};
static uint8_t prevExternalState[8] = {0,0,0,0,0,0,0,0};
static uint8_t mainDoorStatus = 0;
static uint8_t readyToTurnOff = 0; 
static uint16_t shutDownCounter = 0;
static void Sensor_task(void *pvParameter)
{
	static uint8_t scount =0;
	while(1)
	{
		scount++;
		if(scount >= 100)
		{
			printLocalTime();
			scount = 0;
			if(isNewMinute(hh, mm) == true)
				ProcessOutputByTime();
		}
		
		vTaskDelay(50 / portTICK_PERIOD_MS);	//Tick per second

		//Process internal output
		for(int i = 0; i < 4; i++)
		{
			if(outputPage2[i] != prevState[i])
			{
				prevState[i] = outputPage2[i];
				if(outputPage2[i] == 1)
				{
					digitalWrite(relaynum[i], LOW);
				}
				else
				{
					digitalWrite(relaynum[i], HIGH);
				}
			}
		}
		UpdateRelayStatus(externalOutput);

		//Process Input
		ProcessInput();
		if(currentInput[0] != mainDoorStatus)
		{
			mainDoorStatus = currentInput[0]; 
			if(mainDoorStatus == 1)
			{
				//Only open entrance light at night
				if(isDayTime() == 0)
					outputPage2[3] = 1;
			}
			else
			{
				//Ready to turn off Entrance light
				readyToTurnOff = 1;
				shutDownCounter = 250; //estimated 15 seconds
			}
		}

		if(readyToTurnOff)
		{
			shutDownCounter --; 
			if(shutDownCounter == 0)
			{
				//Turn off entrance light 
				outputPage2[3] = 0; 
				readyToTurnOff = 0; 
			}
		}
	}
}

char linebuf[80];
int charcount=0;

static void Webserver_task(void *pvParameter)
{
	char tempBuff[100];
	char sbuff[100];
	char tempPara[10];
    while(1) 
	{
		WiFiClient client = server.available();
		if(client) {
			Serial.println("New Client"); 
			memset(linebuf,0,sizeof(linebuf));
			charcount=0;
			// an http request ends with a blank line
			boolean currentLineIsBlank = true;
			while (client.connected()) {
				if (client.available()) {
					char c = client.read();
					Serial.write(c);
					//read char by char HTTP request
					linebuf[charcount]=c;
					if (charcount<sizeof(linebuf)-1) charcount++;
					if (c == '\n' && currentLineIsBlank) {
						client.println("HTTP/1.1 200 OK");
						client.println("Content-Type: text/html");
						client.println("Connection: close"); 
						client.println();
						client.println("<!DOCTYPE HTML><html><head>");
						
						client.println("<head>");
						client.println("<title>Edlane Technologies</title>");
						client.println("<meta charset=\"utf-8\">");
						client.println("<meta http-equiv=\"refresh\" content=\"10\">");									//Refresh every 10 seconds
						client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
						client.println("<link rel=\"stylesheet\" href=\"https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css\">");
						client.println("<link href=\"https://fonts.googleapis.com/css?family=Montserrat\" rel=\"stylesheet\" type=\"text/css\">");
						client.println("<link href=\"https://fonts.googleapis.com/css?family=Lato\" rel=\"stylesheet\" type=\"text/css\">");
						client.println("<link rel=\"stylesheet\" href=\"https://fonts.googleapis.com/icon?family=Material+Icons\">");
						client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
						client.println("<script src=\"https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js\"></script>");
						client.println("<style>");
						client.println("body {");
						client.println("font: 400 15px Lato, sans-serif;");
						client.println("line-height: 1.8;");
						client.println("color: #818181;");
						client.println("}");
						client.println("h2 {");
						client.println("font-size: 24px;");
						client.println("text-transform: uppercase;");
						client.println("color: #303030;");
						client.println("font-weight: 600;");
						client.println("margin-bottom: 30px;");
						client.println("}");
						client.println("h4 { font-size: 19px; line-height: 1.375em; color: #303030; font-weight: 400; margin-bottom: 30px; }  ");
						client.println(".jumbotron { background-color: #690E5A; color: #fff; padding: 100px 25px; font-family: Montserrat, sans-serif; }");
						client.println(".container-fluid { padding: 60px 50px;}");
						client.println(".bg-grey { background-color: #f6f6f6;}");
						client.println(".logo-small { color: #690E5A;font-size: 50px;}");
						client.println(".thumbnail {padding: 0 0 15px 0;border: none;border-radius: 0;}");
						client.println(".thumbnail img {width: 100%;height: 100%;margin-bottom: 10px;}");
						client.println(".carousel-control.right, .carousel-control.left { background-image: none; color: #f4511e; }");
						client.println(".carousel-indicators li { border-color: #f4511e;}");
						client.println(".carousel-indicators li.active { background-color: #f4511e; }");
						client.println(".item h4 { font-size: 19px; line-height: 1.375em; font-weight: 400; font-style: italic; margin: 70px 0; }");
						client.println(".item span {font-style: normal;}");
						client.println(".panel {      border: 1px solid #f4511e;  border-radius:0 !important; transition: box-shadow 0.5s;}");
						client.println("div.content {     position: relative;     padding: 10px;     width: 100%%;     max-width: 320px;     margin-left: auto;     margin-right: auto;     border: solid 1px black;   }   div.status {     margin-bottom: 10px;     display: inline-block;     width: 100%%;   }   div.status div {     display: inline-block;     width: auto;   }   div.int {     display: inline-block;    padding-right: 10px;     padding-bottom: 10px;     vertical-align: bottom;     font-size: 0;   }   div.int .value {     font-size: 28px;     line-height: 40px;     display: inline-block;     width:  70px;     height: 40px;     border: solid 1px black;     vertical-align: bottom;     text-align: center;   }   div.int .ctrl {     display: inline-block;     width:  21px;     vertical-align: bottom;   }   div.int .ctrl button {     display: inline-block;     width:  30px;     height: 21px;     border-left: none;     padding: 0;   }   div.settings {     display: inline-block;     width: 100%%;     padding-bottom: 10px;   }   div.settings .container {     display: block;     margin-bottom: 10px;   }   div.settings .container * {     display: inline-block;     font-size: 18px;   }   div.settings .container div {     width: 150px;    text-align: center;  }   div.settings .container button {     height: 30px;     padding: 0 0;   }");
						client.println("button.on {    background: green;     color: white;   }   button.off {     background: red;     color: white;   }   .on {     color: green;   }   .off {     color: red;   }   .left {     float: left;     text-align: left;   }   .right {     float: right;     text-align: right;}");
						client.println(".panel:hover { box-shadow: 5px 0px 40px rgba(0,0,0, .2); } .panel-footer .btn:hover { border: 1px solid #f4511e; background-color: #fff !important; color: #f4511e; } .panel-heading { color: #fff !important; background-color: #f4511e !important; padding: 25px;     border-bottom: 1px solid transparent;     border-top-left-radius: 0px;      border-top-right-radius: 0px;      border-bottom-left-radius: 0px;      border-bottom-right-radius: 0px;  }  .panel-footer {      background-color: white !important;  }  .panel-footer h3 {     font-size: 32px; }  .panel-footer h4 {     color: #aaa;     font-size: 14px; }  .panel-footer .btn {     margin: 15px 0;      background-color: #f4511e;      color: #fff;  }  .navbar {      margin-bottom: 0;      background-color: #690E5A;      z-index: 9999;      border: 0;      font-size: 12px !important;      line-height: 1.42857143 !important;      letter-spacing: 2px      border-radius: 0;      font-family: Montserrat, sans-serif;  }  .navbar li a, .navbar .navbar-brand {      color: #fff !important;  }  .navbar-nav li a:hover, .navbar-nav li.active a {      color: #f4511e !important;      background-color: #fff !important;  }  .navbar-default .navbar-toggle {      border-color: transparent;      color: #fff !important;  }  footer .glyphicon {      font-size: 20px;      margin-bottom: 20px;      color: #f4511e;  }  .slideanim {visibility:hidden;}  .slide {      animation-name: slide;      -webkit-animation-name: slide;      animation-duration: 1s;      -webkit-animation-duration: 1s;      visibility: visible;  }  .border-rounded {      border-radius: 10px;      border: 3px solid #ffffff;	  padding: 5px;  }  @keyframes slide {    0% {      opacity: 0;      transform: translateY(70%);    }     100% {      opacity: 1;      transform: translateY(0%);    }  }  @-webkit-keyframes slide {    0% {      opacity: 0;      -webkit-transform: translateY(70%);    }     100% {      opacity: 1;      -webkit-transform: translateY(0%);    }  }  @media screen and (max-width: 768px) {    .col-sm-4 {      text-align: center;      margin: 25px 0;    }    .btn-lg {        width: 100%;        margin-bottom: 35px;    }  }  @media screen and (max-width: 480px) {    .logo {        font-size: 150px;    }  }");
						client.println("</style>");
						client.println("</head>");
						client.println("<body id=\"myPage\" data-spy=\"scroll\" data-target=\".navbar\" data-offset=\"60\">");
						client.println("<nav class=\"navbar navbar-default navbar-fixed-top\">");
						client.println("<div class=\"container\">");
						client.println("<div class=\"navbar-header\">");
						client.println("<button type=\"button\" class=\"navbar-toggle\" data-toggle=\"collapse\" data-target=\"#myNavbar\">");
						client.println("<span class=\"icon-bar\"></span>");
						client.println("<span class=\"icon-bar\"></span>");
						client.println("<span class=\"icon-bar\"></span>");
						client.println("</button>");
						client.println("<a class=\"navbar-brand\" href=\"#myPage\"><span class=\"border-rounded\"><b>EDLANE TECHNOLOGIES</b></span></a>");
						client.println("</div>");
						client.println("<div class=\"collapse navbar-collapse\" id=\"myNavbar\">");
						client.println("<ul class=\"nav navbar-nav navbar-right\">");
						client.println("<li><a href=\"#autocontrol\">AUTO CONTROL</a></li>");
						client.println("<li><a href=\"#manualcontrol\">MANUAL CONTROL</a></li>");
						client.println("<li><a href=\"#sensor\">SENSOR</a></li>");
						client.println("<li><a href=\"#contact\">CONTACT</a></li>");
						client.println("</ul>");
						client.println("</div>");
						client.println("</div>");
						client.println("</nav>");

						client.println("<div class=\"jumbotron text-center\">");
						client.println("<h1>EH001 IoT Switch Controller</h1> ");
						//client.println(" <p>Home Switch Unit for Home Automation System version %s</p>");
						strcpy(tempBuff, " <p>Home Switch Unit for Home Automation System version %d</p>");
						sprintf(sbuff, tempBuff, SOFTWARE_VERSION); 
						client.println(sbuff);
						client.println("</div>");

						client.println("<div id=\"autocontrol\" class=\"container-fluid text-center\">");
						client.println("<h2>AUTO OUTPUT CONTROL</h2>");
						client.println("<div class=\"status\">");
						//client.println("<div class=\"left\">Status <b class=%s</b></div>");
						if(digitalRead(RELAY0))
							client.println("<div class=\"left\">Status <b class=\"on\">ON</b></div>");
						else
							client.println("<div class=\"left\">Status <b class=\"off\">OFF</b></div>");							
						strcpy(tempBuff,"<div class=\"right\">Current time <b>%s</b></div>");
						sprintf(tempPara, "%02d:%02d:%02d", hh, mm, ss);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						//client.println("<div class=\"right\">Current time <b>%s</b></div>");
						client.println("</div>");
						client.println("<div>");
						client.println("<div class=\"int\">");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d", Hour);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						//client.println("<div class=\"value\">%s</div>");
						client.println("<div class=\"ctrl\">");
						client.println("<a href=\"h+\"><button>+</button></a>");
						client.println("<a href=\"h-\"><button>-</button></a>");
						client.println("</div>");
						client.println("</div>");
						client.println("<div class=\"int\">");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d", Minute);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						//client.println("<div class=\"value\">%s</div>");
						client.println("<div class=\"ctrl\">");
						client.println("<a href=\"m+\"><button>+</button></a>");
						client.println("<a href=\"m-\"><button>-</button></a>");
						client.println("</div>");
						client.println("</div>");
						client.println("<div class=\"int\">");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d", Second);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("<div class=\"ctrl\">");
						client.println("<a href=\"s+\"><button>+</button></a>");
						client.println("<a href=\"s-\"><button>-</button></a>");
						client.println("</div>");
						client.println("</div>");

						client.println("<div class=\"settings\">");

						client.println("<div class=\"container\">");
						client.println("<a href=\"setON\"><button>Set ON Valve 0</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourON, SetMinuteON, SetSecondON);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");
						client.println("<div class=\"container\">");
						client.println("<a href=\"setOFF\"><button>Set OFF Valve 0</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourOFF, SetMinuteOFF, SetSecondOFF);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");

						client.println("<div class=\"container\">");
						client.println("<a href=\"set1ON\"><button>Set ON Valve 1</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourON1, SetMinuteON1, SetSecondON1);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");
						client.println("<div class=\"container\">");
						client.println("<a href=\"set1OFF\"><button>Set OFF Valve 1</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourOFF1, SetMinuteOFF1, SetSecondOFF1);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");

						client.println("<div class=\"container\">");
						client.println("<a href=\"set2ON\"><button>Set ON Valve 2</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourON2, SetMinuteON2, SetSecondON2);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");
						client.println("<div class=\"container\">");
						client.println("<a href=\"set2OFF\"><button>Set OFF Valve 2</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourOFF2, SetMinuteOFF2, SetSecondOFF2);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");

						client.println("<div class=\"container\">");
						client.println("<a href=\"set3ON\"><button>Set ON Valve 3</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourON3, SetMinuteON3, SetSecondON3);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");
						client.println("<div class=\"container\">");
						client.println("<a href=\"set3OFF\"><button>Set OFF Valve 3</button></a>");
						strcpy(tempBuff,"<div class=\"value\">%s</div>");
						sprintf(tempPara, "%02d:%02d:%02d", SetHourOFF3, SetMinuteOFF3, SetSecondOFF3);
						sprintf(sbuff, tempBuff, tempPara);
						client.println(sbuff);
						// client.println("<div class=\"value\">%s</div>");
						client.println("</div>");

						client.println("</div>");
						client.println("</div>");
						client.println("<p>");
						client.println("<a href=\"on\"><button class=\"on\">ON</button></a>");
						client.println("<a href=\"off\"><button class=\"off\">OFF</button></a>&nbsp;");
						client.println("</p>");
						client.println("</div>");

						client.println("<div id=\"manualcontrol\" class=\"container-fluid text-center\">");
						// client.println("<div class=\"row\">");
						// client.println("<div class=\"col-sm-8\">");
						client.println("<h2>MANUAL OUTPUT CONTROL</h2><br>");
						if(outputPage2[0])
							client.println("<p>OUTPUT #0 [ON]    <a href=\"on1\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off1\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");
						else
							client.println("<p>OUTPUT #0 [OFF]   <a href=\"on1\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off1\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");
						if(outputPage2[1])
							client.println("<p>OUTPUT #1 [ON]    <a href=\"on2\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off2\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");
						else
							client.println("<p>OUTPUT #1 [OFF]   <a href=\"on2\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off2\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");	
						if(outputPage2[2])
							client.println("<p>OUTPUT #2 [ON]    <a href=\"on3\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off3\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");
						else
							client.println("<p>OUTPUT #2 [OFF]   <a href=\"on3\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off3\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");
						if(outputPage2[3])
							client.println("<p>OUTPUT #3 [ON]    <a href=\"on4\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off4\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");
						else
							client.println("<p>OUTPUT #3 [OFF]   <a href=\"on4\"><button class=\"btn btn-default btn-lg\">ON</button></a>&nbsp;<a href=\"off4\"><button class=\"btn btn-default btn-lg\">OFF</button></a></p>");

						// client.println("</div>");
						// client.println("<div class=\"col-sm-4\">");
						// client.println("<span class=\"glyphicon glyphicon-signal logo\"></span>");
						// client.println("</div>");
						// client.println("</div>");
						client.println("</div>");
						
						client.println("<div id=\"sensor\" class=\"container-fluid text-center\">");
						client.println("<h2>Sensor</h2><br>");
						client.println("<div class=\"row slidenim\">");
						client.println("<div class=\"col-sm-4\">");
						client.println("<h4>Absolute Parameters</h4>");
						client.println(" <p>Temperature		: 30 degC</p>");
						client.println(" <p>Humidity		: 50 %</p>");
						client.println(" <p>Pressure		: 110 Pa</p>");
						client.println(" <p>Light Density	: 70 %</p>");
						client.println("</div>");

						client.println("<div class=\"col-sm-4\">");
						client.println("<h4>Actual Parameter</h4>");
						//client.println(" <p>Temperature		: 30 degC</p>");
						sprintf(sbuff, " <p>Temperature		: %.1f degC</p>", measurement.temperature);
						client.println(sbuff);
						//client.println(" <p>Humidity		: 50 %</p>");
						sprintf(sbuff, " <p> Humidity		: %1f </p>", measurement.humidity);
						client.println(sbuff);
						client.println(" <p>Pressure		: 110 Pa</p>");
						client.println(" <p>Light Density	: 70 %</p>");
						sprintf(sbuff, " <p>PM2.5			: %.1f </p>", PM2_5);
						client.println(sbuff);
						sprintf(sbuff, " <p>PM10			: %.1f </p>", PM10);
						client.println(sbuff);
						client.println("</div>");

						client.println("</div>");
						client.println("</div>");

						client.println("<div id=\"contact\" class=\"container-fluid bg-grey\">");
						client.println("<h2 class=\"text-center\">CONTACT US</h2>");
						client.println("<div class=\"row\">");
						client.println("<div class=\"col-sm-12\">");
						client.println("<p class=\"text-center\">If you have interest in contracting any of our services, please contact us and we'll get back to you shortly.</p>");
						client.println("<p class=\"text-center\"><span class=\"glyphicon glyphicon-map-marker\"></span> Kajang, Malaysia</p>");
						client.println("<p class=\"text-center\"><span class=\"glyphicon glyphicon-envelope\"></span> <a href=\"mailto:edlane.technologies@gmail.com\">info@edlanetech.com</a></p>");
						client.println("</div>");
						client.println("</div>");
						client.println("</div>");

						
						client.println("<footer class=\"container-fluid text-center\">");
						client.println("<a href=\"#myPage\" title=\"To Top\">");
						client.println("<span class=\"glyphicon glyphicon-chevron-up\"></span>");
						client.println("</a>");
						client.println("<p>Edlane Technologies & Solution © 2018</p>");
						client.println("</footer>");

						client.println("<script>");
						client.println("$(document).ready(function(){");
						client.println("  $(\".navbar a, footer a[href='#myPage']\").on('click', function(event) {");
						client.println("if (this.hash !== \"\") {");
						client.println("event.preventDefault();");
						client.println("var hash = this.hash;");
						client.println("$('html, body').animate({");
						client.println("scrollTop: $(hash).offset().top");
						client.println("}, 900, function(){");
						client.println("window.location.hash = hash;");
						client.println("});");
						client.println("}");
						client.println("});");
						client.println("$(window).scroll(function() {");
						client.println("$(\".slideanim\").each(function(){");
						client.println("var pos = $(this).offset().top;");
						client.println("var winTop = $(window).scrollTop();");
						client.println("if (pos < winTop + 600) {");
						client.println("$(this).addClass(\"slide\");");
						client.println("}");
						client.println("});");
						client.println("});");
						client.println("})");
						client.println("</script>");

						client.println("</body>");
						client.println("</html>");
						break;
					}
					if (c == '\n') {
						currentLineIsBlank = true;
						if (strstr(linebuf,"GET /on1") > 0){
            				Serial.println("RELAY 0 ON");
            				digitalWrite(RELAY0, LOW);
							outputPage2[0] = 1;
          				}
						else if(strstr(linebuf, "GET /off1") > 0){
							Serial.println("RELAY 0 OFF");
							digitalWrite(RELAY0, HIGH);
							outputPage2[0] = 0;
						}
						else if (strstr(linebuf,"GET /on2") > 0){
            				Serial.println("RELAY 1 ON");
            				digitalWrite(RELAY1, LOW);
							outputPage2[1] = 1;
          				}
          				else if (strstr(linebuf,"GET /off2") > 0){
            				Serial.println("RELAY 1 OFF");
            				digitalWrite(RELAY1, HIGH);
							outputPage2[1] = 0;
          				}
						else if (strstr(linebuf,"GET /on3") > 0){
							Serial.println("RELAY 2 ON");
							digitalWrite(RELAY2, LOW);
							outputPage2[2] = 1;
						}
						else if (strstr(linebuf,"GET /off3") > 0){
							Serial.println("RELAY 2 OFF");
							digitalWrite(RELAY2, HIGH);
							outputPage2[2] = 0;
						}
						else if (strstr(linebuf, "GET /on4") > 0){
							Serial.println("RELAY 3 ON");
							digitalWrite(RELAY3, LOW);
							outputPage2[3] = 1;
						}
						else if (strstr(linebuf, "GET /off4") > 0){
							Serial.println("RELAY 3 OFF");
							digitalWrite(RELAY3, HIGH);
							outputPage2[3] = 0;
						}
						else if (strstr(linebuf, "GET /setON") > 0){
							SetHourON = Hour;
							SetMinuteON = Minute;
							SetSecondON = Second;
						}
						else if (strstr(linebuf, "GET /setOFF") > 0){
							SetHourOFF = Hour;
							SetMinuteOFF = Minute;
							SetSecondOFF = Second;
						}
						else if (strstr(linebuf, "GET /set1ON") > 0){
							SetHourON1 = Hour;
							SetMinuteON1 = Minute;
							SetSecondON1 = Second;
						}
						else if (strstr(linebuf, "GET /set1OFF") > 0){
							SetHourOFF1 = Hour;
							SetMinuteOFF1 = Minute;
							SetSecondOFF1 = Second;
						}
						else if (strstr(linebuf, "GET /set2ON") > 0){
							SetHourON2 = Hour;
							SetMinuteON2 = Minute;
							SetSecondON2 = Second;
						}
						else if (strstr(linebuf, "GET /set2OFF") > 0){
							SetHourOFF2 = Hour;
							SetMinuteOFF2 = Minute;
							SetSecondOFF2 = Second;
						}
						else if (strstr(linebuf, "GET /set3ON") > 0){
							SetHourON3 = Hour;
							SetMinuteON3 = Minute;
							SetSecondON3 = Second;
						}
						else if (strstr(linebuf, "GET /set3OFF") > 0){
							SetHourOFF3 = Hour;
							SetMinuteOFF3 = Minute;
							SetSecondOFF3 = Second;
						}
						else if (strstr(linebuf, "GET /h+")>0){
							Hour = Hour + 1;
							if(Hour >= 24)
								Hour = 0;
						}
						else if (strstr(linebuf, "GET /h-")>0){
							Hour = Hour - 1;
							if(Hour < 0)
								Hour = 23;
						}
						else if (strstr(linebuf, "GET /m+")>0){
							Minute = Minute + 1;
							if(Minute >= 60)
								Minute = 0;
						}
						else if (strstr(linebuf, "GET /m-")>0){
							Minute = Minute - 1;
							if(Minute < 0)
								Minute = 59;
						}
						else if (strstr(linebuf, "GET /s+")>0){
							Second = Second + 1;
							if(Second >= 60)
								Second = 0;
						}
						else if (strstr(linebuf, "GET /s-")>0){
							Second = Second - 1;
							if(Second < 0)
								Second = 59;
						}
						currentLineIsBlank = true;
          					memset(linebuf,0,sizeof(linebuf));
          					charcount=0;
					}else if (c != '\r') {
						currentLineIsBlank = false;
					}
				}
			}
			delay(1);
			client.stop();
			Serial.println("client disconnected");
		}		
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void weather_data_retrieved(uint32_t *args)
{
	weather_data* weather = (weather_data*) args;
	reference_pressure = (unsigned long) (weather->pressure);
	reference_temperature = weather->temperature;
	reference_humidity = weather->humidity;
	ESP_LOGI(TAG, "Reference pressure: %lu Pa", reference_pressure);
}

int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *pvParameter)
{
	uint8_t updateCount = 0;
    static const char *TX_TASK_TAG = "TX_TASK";
	static uint8_t prevMinute = 0;
	static uint8_t isUpdateNow = 0;
	static uint8_t prevPage = 0;
	static uint8_t isPageChanged = 0;
	static uint8_t isStartup = 1;
	char tempBtStr[51]; 
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {

		if(isStartup)
		{
			char resetPage[30];
			resetPage[0] = 'p'; 
			resetPage[1] = 'a'; 
			resetPage[2] = 'g'; 
			resetPage[3] = 'e';
			resetPage[4] = ' ';
			resetPage[5] = '0';
			resetPage[6] = 0xFF; 
			resetPage[7] = 0xFF; 
			resetPage[8] = 0xFF;
			resetPage[9] = 0;
			sendData(TX_TASK_TAG, resetPage);
			isStartup = 0;
		}
		if(prevMinute != mm)
		{
			isUpdateNow = 1;
			prevMinute = mm;
		}

		if(updateCount == 0 && isUpdateNow == 1 && currentPage == 0)
		{
			char updateClock[30]; 
			updateClock[0] = 't';
			updateClock[1] = '0';
			updateClock[2] = '.';
			updateClock[3] = 't';
			updateClock[4] = 'x';
			updateClock[5] = 't';
			updateClock[6] = '=';
			updateClock[7] = 0x22;
			updateClock[8] = sTime[0];
			updateClock[9] = sTime[1];
			updateClock[10] = sTime[2];
			updateClock[11] = sTime[3];
			updateClock[12] = sTime[4];
			updateClock[13] = 0x22;
			updateClock[14] = 0xFF;
			updateClock[15] = 0xFF;
			updateClock[16] = 0xFF;
			updateClock[17] = 0;
			sendData(TX_TASK_TAG, updateClock);
			updateCount++;
			isUpdateNow = 0;
		}
		else if(updateCount == 1)
		{
			char updateDate[64];
			updateDate[0] = 't';
			updateDate[1] = '1';
			updateDate[2] = '.';
			updateDate[3] = 't';
			updateDate[4] = 'x';
			updateDate[5] = 't';
			updateDate[6] = '=';
			updateDate[7] = 0x22;
			unsigned char w = 0;
			while(sDate[w])
			{
				updateDate[8+w] = sDate[w];
				w++;
			}
			updateDate[8+w] = 0x22;
			updateDate[9+w] = 0xFF;
			updateDate[10+w] = 0xFF;
			updateDate[11+w] = 0xFF;
			updateDate[12+w] = 0;
			sendData(TX_TASK_TAG, updateDate);
			updateCount =  0;
		}

		//Check if there is page changed 
        if(currentPage != prevPage)
		{
			isPageChanged = 1;
			prevPage = currentPage;
		}
		if(isPageChanged)
		{
			switch(currentPage)
			{
				case 0: 
					isUpdateNow = 1; //Update page now
					break;

				case 1: 
					break;

				case 2: 
					//Update bt0 to bt4 val according to relay current value
					memset(&tempBtStr[0], 0xFF, sizeof(tempBtStr));
					memcpy(&tempBtStr[0], "bt0.val=", 8);
					tempBtStr[8] = outputPage2[0] + '0';
					memcpy(&tempBtStr[12], "bt1.val=", 8);
					tempBtStr[20] = outputPage2[1] + '0'; 
					memcpy(&tempBtStr[24], "bt2.val=", 8);
					tempBtStr[32] = outputPage2[2] + '0';
					memcpy(&tempBtStr[36], "bt3.val=", 8);
					tempBtStr[44] = outputPage2[3] + '0';
					tempBtStr[48] = 0;  
					sendData(TX_TASK_TAG, tempBtStr);
					ESP_LOGI(TX_TASK_TAG, "Send %d bytes: '%s'", 48, tempBtStr);
					ESP_LOG_BUFFER_HEXDUMP(TX_TASK_TAG, tempBtStr, 48, ESP_LOG_INFO);
					break;
				
				case 3: 
					//Update bt0 to bt4 val according to relay current value
					memset(&tempBtStr[0], 0xFF, sizeof(tempBtStr));
					memcpy(&tempBtStr[0], "bt4.val=", 8);
					tempBtStr[8] = externalOutput[0] + '0';
					memcpy(&tempBtStr[12], "bt5.val=", 8);
					tempBtStr[20] = externalOutput[1] + '0'; 
					memcpy(&tempBtStr[24], "bt6.val=", 8);
					tempBtStr[32] = externalOutput[2] + '0';
					memcpy(&tempBtStr[36], "bt7.val=", 8);
					tempBtStr[44] = externalOutput[3] + '0';
					tempBtStr[48] = 0;  
					sendData(TX_TASK_TAG, tempBtStr);
					ESP_LOGI(TX_TASK_TAG, "Send %d bytes: '%s'", 48, tempBtStr);
					ESP_LOG_BUFFER_HEXDUMP(TX_TASK_TAG, tempBtStr, 48, ESP_LOG_INFO);
					break;
				
				case 4: 
					//Update bt0 to bt4 val according to relay current value
					memset(&tempBtStr[0], 0xFF, sizeof(tempBtStr));
					memcpy(&tempBtStr[0], "bt8.val=", 8);
					tempBtStr[8] = externalOutput[4] + '0';
					memcpy(&tempBtStr[12], "bt9.val=", 8);
					tempBtStr[20] = externalOutput[5] + '0'; 
					memcpy(&tempBtStr[24], "bt10.val=", 9);
					tempBtStr[33] = externalOutput[6] + '0';
					memcpy(&tempBtStr[37], "bt11.val=", 9);
					tempBtStr[46] = externalOutput[7] + '0';
					tempBtStr[51] = 0;  
					sendData(TX_TASK_TAG, tempBtStr);
					ESP_LOGI(TX_TASK_TAG, "Send %d bytes: '%s'", 48, tempBtStr);
					ESP_LOG_BUFFER_HEXDUMP(TX_TASK_TAG, tempBtStr, 48, ESP_LOG_INFO);
					break;
					break;
				
				case 5: 
					break;
			}
			isPageChanged = 0;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);			//Update even 15 seconds only
    }
}

typedef union
{
	struct{
		uint8_t stx; 
		uint8_t page;
		uint8_t command; 
		uint8_t action;
		uint8_t etx[3];
	}Content;
	uint8_t bytes[7];
}RECEIVE_PACKET_T;
 
static void rx_task(void *pvParameter)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
	RECEIVE_PACKET_T receivedPacket[5];
	int i;
	int k;
	unsigned char isPacketStart = 0;
	unsigned char isValidPacket = 0;
	unsigned char packetData[20];
	char sbuff[50];
	unsigned char endFlagCount = 0;
	int packetCount = 0;
	Serial.println("Reading Nextion Data");
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
		for(int i = 0; i < 5; i++)
			memset(&receivedPacket[i].bytes[0], 0, sizeof(RECEIVE_PACKET_T));

        if (rxBytes > 0) {
			memcpy(&receivedPacket[0].bytes[0], data, rxBytes);
			for(int k = 0; k < 5; k++)
			{
				if(receivedPacket[k].Content.stx == 0x65)
				{
					switch(receivedPacket[k].Content.page)
					{
						case 0x00:
							if(receivedPacket[k].Content.command == 3)
							{
								currentPage = 1;
								Serial.println("Page 0 to Page 1");
							}
							else if(receivedPacket[k].Content.command == 4)
							{
								currentPage = 2;
								Serial.println("Page 0 to Page 2");
							}
						
							break;
						case 0x01:
							Serial.println("Page 1 action");
							break;
						case 0x02:
							//Toggle Output Page
							if(receivedPacket[k].Content.command < 5)
							{
								if(outputPage2[receivedPacket[k].Content.command-1] == 0)
									outputPage2[receivedPacket[k].Content.command-1] = 1;
								else
									outputPage2[receivedPacket[k].Content.command-1] = 0;
							}
							else if(receivedPacket[k].Content.command == 9)
							{
								currentPage = 0;
								Serial.println("Page 2 to Page 0");
							}
							else if(receivedPacket[k].Content.command == 10)
							{
								currentPage = 3;
								Serial.println("Page 2 to Page 3");
							}
							break;
						case 0x03:
							if(receivedPacket[k].Content.command == 9)
							{
								currentPage = 2;
							}
							else if(receivedPacket[k].Content.command == 10)
							{
								currentPage = 4;
							}
							else if(receivedPacket[k].Content.command == 2)
							{
								if(externalOutput[0] == 1)
									externalOutput[0] = 0;
								else
									externalOutput[0] = 1;
							}
							else if(receivedPacket[k].Content.command == 4)
							{
								if(externalOutput[1] == 1)
									externalOutput[1] = 0;
								else
									externalOutput[1] = 1;
							}
							else if(receivedPacket[k].Content.command == 6)
							{
								if(externalOutput[2] == 1)
									externalOutput[2] = 0;
								else
									externalOutput[2] = 1;
							}
							else if(receivedPacket[k].Content.command == 8)
							{
								if(externalOutput[3] == 1)
									externalOutput[3] = 0;
								else
									externalOutput[3] = 1;
							}
							Serial.println("Page 3 action");
							break;
						case 0x04: 
							if(receivedPacket[k].Content.command == 9)
							{
								currentPage = 3;
							}
							else if(receivedPacket[k].Content.command == 10)
							{
								currentPage = 0;		//Back to first page
							}
							else if(receivedPacket[k].Content.command == 2)
							{
								if(externalOutput[4] == 1)
									externalOutput[4] = 0;
								else
									externalOutput[4] = 1;
							}
							else if(receivedPacket[k].Content.command == 4)
							{
								if(externalOutput[5] == 1)
									externalOutput[5] = 0;
								else
									externalOutput[5] = 1;
							}
							else if(receivedPacket[k].Content.command == 6)
							{
								if(externalOutput[6] == 1)
									externalOutput[6] = 0;
								else
									externalOutput[6] = 1;
							}
							else if(receivedPacket[k].Content.command == 8)
							{
								if(externalOutput[7] == 1)
									externalOutput[7] = 0;
								else
									externalOutput[7] = 1;
							}
							Serial.println("Page 4 action");
							break;
						default:
							Serial.println("Other Page Detected!");
							break;
					}
				}
				else if(receivedPacket[k].Content.stx == 0x1A)
				{
					//Invalid Command  (To do: Implement handling)
					Serial.println("NEXTION ERROR!");
				}
			}
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
		vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    free(data);
}

extern "C" void app_main()
{
	esp_err_t ret;
	nvs_flash_init();
	initArduino();
	Serial.begin(115200);
	Serial.println("Socrates Reader Demostration");

	//Set Relay as output pin 
	pinMode(RELAY0, OUTPUT);
	pinMode(RELAY1, OUTPUT);
	pinMode(RELAY2, OUTPUT);
	pinMode(RELAY3, OUTPUT);

	//Set Input as input pin
	pinMode(INPUT0, INPUT);
	pinMode(INPUT1, INPUT); 
	pinMode(INPUT2, INPUT); 
	pinMode(INPUT3, INPUT);
	
	//Turn Low Relay 
	digitalWrite(RELAY0, HIGH);
	digitalWrite(RELAY1, HIGH);
	digitalWrite(RELAY2, HIGH);
	digitalWrite(RELAY3, HIGH);

	memset(relayStatus, 0, 4);
	initialise_wifi();	
	//Get Weather Info from OpenWeatherMap
	if(reference_pressure == 01) {
		initialise_weather_data_retrieval(6000);
		on_weather_data_retrieval(weather_data_retrieved);
		ESP_LOGW(TAG, "Weather data retrieval initialized");
	}
	delay(100);
	//Update time based on NTP Server
	configTime(hourOffset * 3600, 0, ntp_server1);
	delay(100);
	printLocalTime();
	delay(100);

	dht.begin();
	sensor_t sensor;
	dht.temperature().getSensor(&sensor);
  	Serial.println("------------------------------------");
  	Serial.println("Temperature");
  	Serial.print  ("Sensor:       "); Serial.println(sensor.name);
 	Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  	Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  	Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  	Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  	Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  	Serial.println("------------------------------------");
  	dht.humidity().getSensor(&sensor);
  	Serial.println("------------------------------------");
  	Serial.println("Humidity");
  	Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  	Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  	Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  	Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  	Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  	Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");  
	measurement.temperature = 30;
	measurement.humidity = 50; 
	measurement.airQuality_2_5 = 0;
	measurement.airQuality_10 = 0;
	measurement.lightIntensity = 0;
	
	uart_init();
	//Initialise Webserver
	server.begin();
	initialise_version_connection();
	initialise_external_connection();
	xTaskCreate(Webserver_task, "Webserver_Task", 2048, NULL, 5, NULL);
	xTaskCreate(Sensor_task, "Sensor_Task", 2048, NULL, 4, NULL);
	xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, 3, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, 2, NULL);
	return;
}
