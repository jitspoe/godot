/*************************************************************************/
/*  tensorflow.cpp                                                       */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2018 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2018 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#include "tensorflow.h"

#include "iris.h"

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/optional_debug_tools.h>
#include <tensorflow/lite/stderr_reporter.h>

//#include <cstdio>
//#include <cstring>
//
//using namespace std;
//using namespace tflite;
void Tensorflow::inference() {
	// Based on
	// https://tomaxent.com/2018/01/17/PCA-With-Tensorflow/
	// Tensorflow minimal example
	const char *filename = "iris.json";

	//// Load model
	//std::unique_ptr<tflite::FlatBufferModel> model =
	//		tflite::FlatBufferModel::BuildFromFile(filename);
	//ERR_FAIL_COND(model == nullptr);
	//// Build the interpreter
	//tflite::ops::builtin::BuiltinOpResolver resolver;
	//tflite::InterpreterBuilder builder(*model, resolver);
	//std::unique_ptr<tflite::Interpreter> interpreter;
	//builder(&interpreter);
	//ERR_FAIL_COND(interpreter == nullptr);

	//// Allocate tensor buffers.
	//ERR_FAIL_COND(interpreter->AllocateTensors() != kTfLiteOk);
	//print_line("=== Pre-invoke Interpreter State ===");
	//tflite::PrintInterpreterState(interpreter.get());

	//// Fill input buffers
	//// TODO(user): Insert code to fill input tensors

	//// Run inference
	//ERR_FAIL_COND(interpreter->Invoke() != kTfLiteOk);
	//print_line("\n\n=== Post-invoke Interpreter State ===");
	//tflite::PrintInterpreterState(interpreter.get());
}

void Tensorflow::add(int value) {

	count += value;
}

void Tensorflow::reset() {

	count = 0;
}

int Tensorflow::get_total() const {

	return count;
}

void Tensorflow::_bind_methods() {

	ClassDB::bind_method(D_METHOD("add", "value"), &Tensorflow::add);
	ClassDB::bind_method(D_METHOD("reset"), &Tensorflow::reset);
	ClassDB::bind_method(D_METHOD("get_total"), &Tensorflow::get_total);
	ClassDB::bind_method(D_METHOD("inference"), &Tensorflow::inference);
}

Tensorflow::Tensorflow() {
	count = 0;
}
