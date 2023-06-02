#include "mbed.h"
#include "bbcar.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmIn servo0_f(D9), servo1_f(D10);
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);
DigitalInOut pin8(D8);
BusInOut qti_pin(D4,D5,D6,D7);
WiFiInterface *wifi;
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
int rightflag = 0;
int leftflag = 0;
int stopflag = 0;
int pattern;
int original_angle;

float distance1 = 0;
const char* topic = "Mbed";
Thread mqtt_thread(osPriorityNormal);
Thread car_thread(osPriorityHigh);
parallax_ping  ping1(pin8);
parallax_qti qti1(qti_pin);
EventQueue car_queue;
EventQueue mqtt_queue;

// GLOBAL VARIABLES

//InterruptIn btn3(SW3);

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(2000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    distance1 = (car.servo0.angle-original_angle) / 360*6.5*3.14;
    message_num++;
    MQTT::Message message;
    if(pattern == 0b0111){
        char buff[100];
        sprintf(buff, "next branch right turn, distance runned: %f cm\n", distance1);
        message.qos = MQTT::QOS0;
        message.retained = false;
        message.dup = false;
        message.payload = (void*) buff;
        message.payloadlen = strlen(buff) + 1;
        int rc = client->publish(topic, message);
        printf("rc:  %d\r\n", rc);
        printf("Puslish message: %s\r\n", buff);
    }
    else if(pattern == 0b1110){
        char buff[100];
        sprintf(buff, "next branch left turn, distance runned: %f cm\n", distance1);
        message.qos = MQTT::QOS0;
        message.retained = false;
        message.dup = false;
        message.payload = (void*) buff;
        message.payloadlen = strlen(buff) + 1;
        int rc = client->publish(topic, message);
        printf("rc:  %d\r\n", rc);
        printf("Puslish message: %s\r\n", buff);
    }
    else if((pattern == 0b1111) &&(rightflag == 0)&&(leftflag == 0)){
        char buff[100];
        sprintf(buff, "STOP!, distance runned: %f cm\n", distance1);
        message.qos = MQTT::QOS0;
        message.retained = false;
        message.dup = false;
        message.payload = (void*) buff;
        message.payloadlen = strlen(buff) + 1;
        int rc = client->publish(topic, message);
        printf("rc:  %d\r\n", rc);
        printf("Puslish message: %s\r\n", buff);
    }
    else {
        char buff[100];
        sprintf(buff, "Running!, distance runned: %f cm\n", distance1);
        message.qos = MQTT::QOS0;
        message.retained = false;
        message.dup = false;
        message.payload = (void*) buff;
        message.payloadlen = strlen(buff) + 1;
        int rc = client->publish(topic, message);
        printf("rc:  %d\r\n", rc);
        printf("Puslish message: %s\r\n", buff);
    }
}
void car_mode(){
    pattern = (int)qti1;
    printf("%d\n",pattern);
      switch (pattern) {
         case 0b1000: car.turn(65, 0.1); break;
         case 0b1100: car.turn(65, 0.4); break;
         case 0b1110: 
         //car.turn(60, 0.8);
            leftflag = 1;
            rightflag = 0;
            break;
         case 0b0100: car.turn(65, 0.7); break;
         case 0b0110: car.goStraight(65); break;
         case 0b0010: car.turn(65, -0.7); break;
         case 0b0111: 
            car.turn(60, -0.8);
            rightflag = 1; 
            leftflag = 0;
            break;
         case 0b0011: car.turn(65, -0.4); break;
         case 0b0001: car.turn(65, -0.1); break;
         case 0b1111:
         if (stopflag == 1){
             car.stop();
         }
            if(rightflag==1){
                car.branchright(80);
                ThisThread::sleep_for(1400ms);
                rightflag = 0;
                leftflag = 0;
            } 
            else if(leftflag==1){
                car.branchleft(80);
                ThisThread::sleep_for(1400ms);
                leftflag = 0;
                rightflag = 0;
            }
            else car.stop(); 
            break;
        case 0b0000:
            leftflag = 0;
            rightflag = 0;
            stopflag = 1;
            break;
         default: car.goStraight(65);
      }
    if ((float)ping1 < 10) {
        car.stop();
        car.rotate(100);
        ThisThread::sleep_for(900ms);
        car.stop();
        ThisThread::sleep_for(1000ms);
        if((float) ping1 > 15){
            car.goStraight(65);
        }
        else{
            car.rotate(-100);
            ThisThread::sleep_for(2300ms);
            car.stop();
            ThisThread::sleep_for(900ms);
            if((float)ping1 > 15){
                car.goStraight(65);
            }
            else{
                car.rotate(-100);
                ThisThread::sleep_for(800ms);
            }
        }

    }
}

void close_mqtt() {
    closed = true;
}


int main() {
   original_angle = car.servo0.angle;
   wifi = WiFiInterface::get_default_instance();
   wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }
    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "172.20.10.6";
    const int port=1883;
    printf("Connecting to TCP network...\r\n");
    printf("address is %s/%d\r\n", host, port);

    int rc = mqttNetwork.connect(host, port);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    mqtt_queue.call_every(500ms, &publish_message, &client);
    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    car_queue.call_every(50ms, &car_mode);
    car_thread.start(callback(&car_queue, &EventQueue::dispatch_forever));
    car.goStraight(50);
    int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }
    while (1) {
            if (closed) break;
            client.yield(500);
            ThisThread::sleep_for(1500ms);
    }
   printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");

    return 0;
}