1.how to setup and run your program  
&emsp;&emsp;(1)Clone the repo: $ git clone https://github.com/Lyds099/exam2.git  
&emsp;&emsp;(2)Compile the program: $ sudo mbed compile --source . --source ~/ee2405/mbed-os-build/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f  
&emsp;&emsp;(3)Open the screen: $ sudo screen /dev/ttyACM0  
&emsp;&emsp;(4)Execute python client: $ sudo python3 wifi_mqtt/mqtt_client.py  
&emsp;&emsp;(5)Type in RPC command. First, put the mbed on the table. After the initialization process, you can tilt the mbed. The gesture will be displayed on uLCD. The event will be published through WiFi/MQTT. After 10 gesture events, 2 figures will be published through WiFi/MQTT. The mbed will be back to RPC loop.    

2.what are the results  
&emsp;&emsp;circuit: https://github.com/Lyds099/exam2/blob/master/832498521.jpg?raw=true  
&emsp;&emsp;terminal(event): https://github.com/Lyds099/exam2/blob/master/Screen%20Shot%202021-05-12%20at%206.10.09%20PM.png?raw=true    
&emsp;&emsp;terminal(figure): https://github.com/Lyds099/exam2/blob/master/Screen%20Shot%202021-05-12%20at%206.29.59%20PM.png?raw=true
