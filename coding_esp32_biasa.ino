#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";

// Initialize Telegram BOT
#define BOTtoken "XXXXXXXXXX:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define CHAT_ID "XXXXXXXXXX"

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

const int MQ_PIN=34;                                //define which analog input channel you are going to use
int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                    //which is derived from the chart in datasheet
 
/********Software Related Macros*************/
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in 
                                                    //normal operation
 
/**********Globals****************/ 
float SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                          //with these two points, a line is formed which is "approximately equivalent" 
                                          //to the original curve.
                                          //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float Ro = 10;                            //Ro is initialized to 10 kilo ohms

String Data;
int Outputdata = 25;

int state = 0;


void setup()
{ 
  Serial.begin(9600);
  pinMode(Outputdata, OUTPUT);
  digitalWrite(Outputdata, LOW);
  Ro = MQCalibration(MQ_PIN);            
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());            
}
 
void loop()
{  
  long iPPM_Smoke = 0;

  iPPM_Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro);

  Data = String(iPPM_Smoke);

   Serial.print("Smoke: ");
   Serial.print(iPPM_Smoke);
   Serial.println(" ppm"); 

   if (iPPM_Smoke >= 20 && state == 0){
    digitalWrite(Outputdata, HIGH);
    delay(200);
    digitalWrite(Outputdata, LOW);
    bot.sendMessage(CHAT_ID, Data, "");
    state = 1;
    }

  else {
    state = 0;
    }

   delay(200);
  
}
 

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(4095-raw_adc)/raw_adc));
}
 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

}
 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
long MQGetGasPercentage(float rs_ro_ratio)
{
  return MQGetPercentage(rs_ro_ratio,SmokeCurve); 
 
  return 0;
}
 

long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
