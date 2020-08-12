/*
* RocksettaTinyML by Jeremy Ellis,keyfreemusic@gmail.com
*
* Based on the Arduino Library
* EloquentTinyML by Simone Salerno,eloquentarduino@gmail.com
* https://github.com/eloquentarduino/EloquentTinyML
* 
* Passed from the main program
* #define NUMBER_OF_INPUTS 8
* #define TYPE_OF_INPUTS int32             // uint8_t | int32 | float | bool | double | String 
* 
* #define NUMBER_OF_OUTPUTS 4
* #define TYPE_OF_OUTPUTS float            // uint8_t |int32 | float | bool | double | String 
* 
* #define RETURN_TYPE_FROM_PREDICT float    // void | uint8_t | int32 | float | bool | double | String 
* 
*/




#pragma once

#include <Arduino.h>
#include <math.h>
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"


namespace Eloquent {
    namespace TinyML {

        /**
         * Eloquent interface to Tensorflow Lite for Microcontrollers
         *
         * @tparam inputSize
         * @tparam outputSize
         * @tparam tensorArenaSize how much memory to allocate to the tensors
         */
        template<size_t inputSize, size_t outputSize, size_t tensorArenaSize>
        class TfLite {
        public:
            /**
             * Contructor
             * @param modelData a model as exported by tinymlgen
             */
            TfLite() :
                failed(false) {
            }

            bool begin(const unsigned char *modelData) {
                static tflite::MicroErrorReporter microReporter;
                static tflite::ops::micro::AllOpsResolver resolver;

                reporter = &microReporter;
                model = tflite::GetModel(modelData);

                // assert model version and runtime version match
                if (model->version() != TFLITE_SCHEMA_VERSION) {
                    failed = true;
                    Serial.println("Version mismatch");
                    reporter->Report(
                            "Model provided is schema version %d not equal "
                            "to supported version %d.",
                            model->version(), TFLITE_SCHEMA_VERSION);

                    return false;
                }

                static tflite::MicroInterpreter interpreter(model, resolver, tensorArena, tensorArenaSize, reporter);

                if (interpreter.AllocateTensors() != kTfLiteOk) {
                    failed = true;
                    Serial.println("Allocation failed");
                    reporter->Report("AllocateTensors() failed");

                    return false;
                }

                input = interpreter.input(0);
                output = interpreter.output(0);

                this->interpreter = &interpreter;

                return true;
            }

            /**
             * Test if the initialization completed fine
             */
            bool initialized() {
                return !failed;
            }

            uint8_t predict(uint8_t *input, uint8_t *output = NULL) {
                // abort if initialization failed
                if (!initialized())
                    return sqrt(-1);

                memcpy(input, this->input->data.uint8, sizeof(uint8_t) * inputSize);

                if (interpreter->Invoke() != kTfLiteOk) {
                    reporter->Report("Inference failed");

                    return sqrt(-1);
                }

                // copy output
                if (output != NULL)
                    memcpy(output, this->output->data.uint8, sizeof(uint8_t) * outputSize);

                return this->output->data.uint8[0];
            }

            /**
             * Run inference
             * @return output[0], so you can use it directly if it's the only output
             */

            float predict(int32 *input, float *output = NULL) {
                // abort if initialization failed
                if (!initialized())
                    return sqrt(-1);

                // copy input
                for (size_t i = 0; i < inputSize; i++)
                    this->input->data.f[i] = input[i];

                if (interpreter->Invoke() != kTfLiteOk) {
                    reporter->Report("Inference failed");

                    return sqrt(-1);
                }

                 //copy output  output is a pointer which might be a better way to deal with this
                if (output != NULL) {
                    memcpy(output, this->output->data.f, sizeof(float) * outputSize);
                }

                return this->output->data.f[0]; // just send back the first value for testing
            }



        float predict(float *input, float *output = NULL) {
                // abort if initialization failed
                if (!initialized())
                    return sqrt(-1);

                // copy input
                for (size_t i = 0; i < inputSize; i++)
                    this->input->data.f[i] = input[i];

                if (interpreter->Invoke() != kTfLiteOk) {
                    reporter->Report("Inference failed");

                    return sqrt(-1);
                }

                 //copy output  output is a pointer which might be a better way to deal with this
                if (output != NULL) {
                    memcpy(output, this->output->data.f, sizeof(float) * outputSize);
                }

                return this->output->data.f[0]; // just send back the first value for testing
            }


        float predict(String *input, float *output = NULL) {
                // abort if initialization failed
                if (!initialized())
                    return sqrt(-1);

                // copy input
                for (size_t i = 0; i < inputSize; i++)
                    this->input->data.f[i] = input[i];

                if (interpreter->Invoke() != kTfLiteOk) {
                    reporter->Report("Inference failed");

                    return sqrt(-1);
                }

                 //copy output  output is a pointer which might be a better way to deal with this
                if (output != NULL) {
                    memcpy(output, this->output->data.f, sizeof(float) * outputSize);
                }

                return this->output->data.f[0]; // just send back the first value for testing
            }


        float predict(double *input, float *output = NULL) {
                // abort if initialization failed
                if (!initialized())
                    return sqrt(-1);

                // copy input
                for (size_t i = 0; i < inputSize; i++)
                    this->input->data.f[i] = input[i];

                if (interpreter->Invoke() != kTfLiteOk) {
                    reporter->Report("Inference failed");

                    return sqrt(-1);
                }

                 //copy output  output is a pointer which might be a better way to deal with this
                if (output != NULL) {
                    memcpy(output, this->output->data.f, sizeof(float) * outputSize);
                }

                return this->output->data.f[0]; // just send back the first value for testing
            }


        float predict(bool *input, float *output = NULL) {
                // abort if initialization failed
                if (!initialized())
                    return sqrt(-1);

                // copy input
                for (size_t i = 0; i < inputSize; i++)
                    this->input->data.f[i] = input[i];

                if (interpreter->Invoke() != kTfLiteOk) {
                    reporter->Report("Inference failed");

                    return sqrt(-1);
                }

                 //copy output  output is a pointer which might be a better way to deal with this
                if (output != NULL) {
                    memcpy(output, this->output->data.f, sizeof(float) * outputSize);
                }

                return this->output->data.f[0]; // just send back the first value for testing
            }


        protected:
            bool failed;
            uint8_t tensorArena[tensorArenaSize];
            tflite::ErrorReporter *reporter;
            tflite::MicroInterpreter *interpreter;
            TfLiteTensor *input;
            TfLiteTensor *output;
            const tflite::Model *model;
        };
    }
}
