#include "mbed.h"
#include "mbed_rpc.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
#include "stm32l475e_iot01_accelero.h"
#include "mbed_events.h"
using namespace std::chrono;

#include "uLCD_4DGL.h"
uLCD_4DGL uLCD(D1, D0, D2);

#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

#include <math.h>       /* atan */

MQTT::Client<MQTTNetwork, Countdown> *global_client;

WiFiInterface *wifi;
//InterruptIn btn2(USER_BUTTON);
//InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Mbed";

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalIn mypin(USER_BUTTON);

BufferedSerial pc(USBTX, USBRX);
void capture(Arguments *in, Reply *out);
RPCFunction rpcAcc(&capture, "capture");

Thread captureThread(osPriorityLow);

int pre_angle = 0;
int choose_angle = 30;//30 45 60
int angle = 0;
int angle_set = 0;//flag
double angle_result, angle_reference;
int over_threshold = 0;//flag
int over_count = 0;

int gesture_id;
int gestures[12];
int event_num;
int is_gasture = 0;//flag
int accData_index = 0;
float a, b, c;
int features[12][5]={0};
int plot_fig = 0;//flag

// Store x, y, z data
int16_t pDataXYZ[3] = {0};

// A buffer holding the last 200 sets of 3-channel values
static float save_data[600] = {0.0};
// Most recent position in the save_data buffer
int begin_index = 0;
// True if there is not yet enough data to run inference
bool pending_initial_data = true;
// How often we should save a measurement during downsampling
int sample_every_n = 1;
// The number of measurements since we last saved one
int sample_skip_counter = 1;

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    int16_t pDataXYZ[3] = {0};
    MQTT::Message message;
    char buff[200];
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    if(is_gasture){
        sprintf(buff, "The gesture ID is %d. event sequence number:#%d", gesture_id, event_num);
        message.qos = MQTT::QOS0;
        message.retained = false;
        message.dup = false;
        message.payload = (void*) buff;
        message.payloadlen = strlen(buff) + 1;
        int rc = client->publish(topic, message);

        printf("rc:  %d\r\n", rc);
        printf("Puslish message: %s\r\n", buff);
        ThisThread::sleep_for(500ms);
    }
    else if(plot_fig){
        for(int k=0; k<3; k++){
           if(k==0) sprintf(buff, "The classified gesture events");
           else if(k==1) sprintf(buff, "No 1 2 3 4 5 6 7 8 9 10");
           else if(k==2) sprintf(buff, "ID %d %d %d %d %d %d %d %d %d %d",gestures[0],gestures[1],gestures[2],
               gestures[3],gestures[4],gestures[5],gestures[6],gestures[7],gestures[8],gestures[9]);

           message.qos = MQTT::QOS0;
           message.retained = false;
           message.dup = false;
           message.payload = (void*) buff;
           message.payloadlen = strlen(buff) + 1;
           int rc = client->publish(topic, message);
           
           printf("rc:  %d\r\n", rc);
           printf("Puslish message: %s\r\n", buff);
           ThisThread::sleep_for(500ms);
        }
        for(int k=0; k<6; k++){
          if(k==0) sprintf(buff, "The extracted features");
           else if(k==1) sprintf(buff, "No     1   2   3   4   5   6   7   8   9   10");
           else if(k==2) sprintf(buff, "ang=0  %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d", features[0][0],features[1][0],features[2][0],
           features[3][0],features[4][0],features[5][0],features[6][0],features[7][0],features[8][0],features[9][0]);
           else if(k==3) sprintf(buff, "ang>0  %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d", features[0][1],features[1][1],features[2][1],
           features[3][1],features[4][1],features[5][1],features[6][1],features[7][1],features[8][1],features[9][1]);
           else if(k==4) sprintf(buff, "ang>30 %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d", features[0][2],features[1][2],features[2][2],
           features[3][2],features[4][2],features[5][2],features[6][2],features[7][2],features[8][2],features[9][2]);
           else if(k==5) sprintf(buff, "ang>60 %3d %3d %3d %3d %3d %3d %3d %3d %3d %3d", features[0][3],features[1][3],features[2][3],
           features[3][3],features[4][3],features[5][3],features[6][3],features[7][3],features[8][3],features[9][3]);

           message.qos = MQTT::QOS0;
           message.retained = false;
           message.dup = false;
           message.payload = (void*) buff;
           message.payloadlen = strlen(buff) + 1;
           int rc = client->publish(topic, message);
           
           printf("rc:  %d\r\n", rc);
           printf("Puslish message: %s\r\n", buff);
           ThisThread::sleep_for(500ms);
        }
    }
    // message.qos = MQTT::QOS0;
    // message.retained = false;
    // message.dup = false;
    // message.payload = (void*) buff;
    // message.payloadlen = strlen(buff) + 1;
    // int rc = client->publish(topic, message);

    // printf("rc:  %d\r\n", rc);
    // printf("Puslish message: %s\r\n", buff);
    // ThisThread::sleep_for(500ms);
}

void close_mqtt() {
    closed = true;
}

int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

int main() {

    BSP_ACCELERO_Init();

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
    global_client = &client;

    //TODO: revise host to your IP
    const char* host = "172.20.10.3";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
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

    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    //mqtt end

    // receive commands, and send back the responses
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
}

void display_gesture(){
        uLCD.cls();
        uLCD.text_width(2);
        uLCD.text_height(2);
        uLCD.textbackground_color(BLACK);
        uLCD.color(WHITE);
        uLCD.locate(2,1);
        uLCD.printf("Gesture");
        uLCD.locate(2,3);
        uLCD.printf("%d",(int)gesture_id);
}


void AccCapture_mode(){

  int16_t pDataXYZ[3] = {0};
   myled1 = 1;
   myled2 = 1;
   ThisThread::sleep_for(2000ms);
   BSP_ACCELERO_AccGetXYZ(pDataXYZ);
   angle_reference = atan ((double)pDataXYZ[0]/(double)pDataXYZ[2]) * 180.0 / 3.141592653589793238462;
   myled1 = 0;
   myled2 = 0;

    // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }

  //error_reporter->Report("Set up successful...\n");

  event_num = 1;
  memset(features, 0, sizeof(features));
  while (true) {
    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
        is_gasture = 1;
        gesture_id = gesture_index;
        gestures[event_num-1] = gesture_id;
        mqtt_queue.call(&publish_message, global_client);
        global_client->yield(500);
        display_gesture();
        is_gasture = 0;
        event_num++;
    }
    //extract feature
    accData_index = 0;
    for(int k=0; k<200; k++){
      a = save_data[accData_index++];
      b = save_data[accData_index++];
      c = save_data[accData_index++];
      angle_result = atan ((double)a/(double)c) * 180.0 / 3.141592653589793238462;
      if(angle_reference>0) angle_result = angle_result - angle_reference;
      else angle_result = angle_result + abs(angle_reference);
      if(angle_result==0.0) features[event_num-1][0] += 1;
      else if(angle_result>0.0 && angle_result<30.0) features[event_num-1][1] += 1;
      else if(angle_result>=30.0 && angle_result<60.0) features[event_num-1][2] += 1;
      else if(angle_result>=60.0 && angle_result<90.0) features[event_num-1][3] += 1;
    }
    if(event_num>=11){
        plot_fig = 1;
        mqtt_queue.call(&publish_message, global_client);
        global_client->yield(500);
        plot_fig = 0;
        //break;
        captureThread.terminate();
    }
  }
}

void capture(Arguments *in, Reply *out){
    captureThread.start(&AccCapture_mode);
}

TfLiteStatus SetupAccelerometer(tflite::ErrorReporter* error_reporter) {
  // Init accelerometer
  BSP_ACCELERO_Init();
  return kTfLiteOk;
}

bool ReadAccelerometer(tflite::ErrorReporter* error_reporter, float* input,
                       int length, bool reset_buffer) {
  // Clear the buffer if required, e.g. after a successful prediction
  if (reset_buffer) {
    memset(save_data, 0, 600 * sizeof(float));
    begin_index = 0;
    pending_initial_data = true;
  }

  // Obtain a sample
  while(sample_skip_counter <= sample_every_n) {
     BSP_ACCELERO_AccGetXYZ(pDataXYZ);
     sample_skip_counter += 1;
  }

  // Write samples to our buffer, converting to milli-Gs
  save_data[begin_index++] = (float)pDataXYZ[0];
  save_data[begin_index++] = (float)pDataXYZ[1];
  save_data[begin_index++] = (float)pDataXYZ[2];

  // Since we took a sample, reset the skip counter
  sample_skip_counter = 1;

  // If we reached the end of the circle buffer, reset
  if (begin_index >= 600) {
    begin_index = 0;
  }

  // Check if we are ready for prediction or still pending more initial data
  if (pending_initial_data && begin_index >= 200) {
    pending_initial_data = false;
  }

  // Return if we don't have enough data
  if (pending_initial_data) {
    return false;
  }

  // Copy the requested number of bytes to the provided input tensor
  for (int i = 0; i < length; ++i) {
    int ring_array_index = begin_index + i - length;
    if (ring_array_index < 0) {
      ring_array_index += 600;
    }
    input[i] = save_data[ring_array_index];
  }

  return true;
}