#include <RocksettaTinyML.h>
// sine_model.h contains the array you exported from the previous step with xxd or tinymlgen
#include "sine_model.h"

#define NUMBER_OF_INPUTS 1
#define NUMBER_OF_OUTPUTS 1
// in future projects you may need to tweek this value: it's a trial and error process
#define TENSOR_ARENA_SIZE 2*1024

Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

float myCount, myX, myY = 0;


void setup() {
    Serial.begin(115200);
    ml.begin(sine_model);
}

void loop() {
    // pick up a random x and predict its sine
    myCount += 1;
    myX = sin(myCount*180/3.14);   
    float input[1] = { myX };
    float myY = ml.predict(input);
   // Serial.print(x);
   // Serial.print(",");
    Serial.println(myY);
    delay(33);
}
