// * Type "AT" (without the quotes) on the serial monitor and press enter. if "OK" appears then everything is all right and the module is ready to take command. Now you can change the name of the module, retrieve address or version or even reset to factory settings. To see the default name, type AT+NAME. The name will be prompted, by default it is HC-05 or JY_MCU or something like that. To change the name just type AT+NAME=your desired name.
// * Here is an important note, if the key pin is not high, i.e. not connected to Vcc while receiving AT commands(if you did not solder the wire and released it after the module entered AT mode), it will not show the default name even after giving right command. But you can still change the name by the command mentioned above. To verify if the name has really changed, search the device from your pc/mobile. The changed name will appear. To change baud rate, type AT+UART=desired baud rate. Exit by sending AT+RESET command.
// * Most useful AT commands are
// *
// * AT : Ceck the connection.
// * AT+NAME : See default name
// * AT+ADDR : see default address
// * AT+VERSION : See version
// * AT+UART : See baudrate
// * AT+ROLE: See role of bt module(1=master/0=slave)
// * AT+RESET : Reset and exit AT mode
// * AT+ORGL : Restore factory settings
// * AT+PSWD: see default password

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {

Serial.begin(9600);

pinMode(9,OUTPUT); digitalWrite(9,HIGH);

Serial.println("Enter AT commands:");

mySerial.begin(38400);

}

void loop()

{

if (mySerial.available())

Serial.write(mySerial.read());

if (Serial.available())

mySerial.write(Serial.read());

}
