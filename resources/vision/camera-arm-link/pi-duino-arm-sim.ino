#include <Arduino.h>

// ARM Simulator (communication link)
// 1. Poll Serial for information
// 2. From message determine whether to pick or place
// 3. If picking, get coordinate info from message
// 4. Pass coordinates to IK function
// 5. Return SUCCESS if SUCCESS, else return FAILURE
// 6. If SUCCESS, send message to Pi via Serial

int pickStatus = 0;

int pick(int x, int y, int z);
String getValue(String data, char separator, int index);

void setup() {
  Serial.begin(9600);
  // Serial.println("Waiting for message...");
}

void loop() {

  // 1. Poll Serial for information
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    // String data = "0|10|20|30\n";
    // Serial.print("[Arduino Rx]> ");
    // Serial.print(data);
    // Serial.println();

    // 2. Determine whether to pick or place from message

    // Message Format
    // "action|x|y|z" -> "0|10|20|30"

    // 0 - pick , 1 - place
    String action = getValue(data,'|',0);
    // Get - Coordinates
    String x_coord = getValue(data,'|',1);
    String y_coord = getValue(data,'|',2);
    String z_coord = getValue(data,'|',3);

    // Serial.print("Parsed Message: ");
    // Serial.print(action);
    // Serial.print(" ");
    // Serial.print(x_coord);
    // Serial.print(" ");
    // Serial.print(y_coord);
    // Serial.print(" ");
    // Serial.print(z_coord);
    // Serial.println();

  

  // typecast action to int
  int a = action.toInt();

  // 3. If picking, get coordinate info from message
  if (a == 0){
    int x = x_coord.toInt();
    int y = y_coord.toInt();
    int z = z_coord.toInt();

    // 4. Send coordinates to IK function
    pickStatus = pick(x, y, z);

    // 6. Send message to Pi
    if (pickStatus == 1) {
      Serial.println("SUCCESS");
    } else {
      Serial.println("FAILURE");
    }
    pickStatus = 0;
  } else{
      Serial.println("PLACING");
  }
  }

  // delay(3000);
  // Serial.println("Waiting for message...");

}

// IK Function
int pick(int x, int y, int z) {

  // Serial.print("Picking at: ");
  // Serial.print(x);
  // Serial.print(" ");
  // Serial.print(y);
  // Serial.print(" ");
  // Serial.print(z);
  // Serial.println();

  delay(3000);

  // 5. Return SUCCESS if SUCCESS, else return FAILURE (after picking)
  return 1;
}


// https://stackoverflow.com/questions/29671455/how-to-split-a-string-using-a-specific-delimiter-in-arduino
// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}