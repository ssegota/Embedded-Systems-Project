#!/usr/bin/env node

'use strict';
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
var connectionString = 'HostName=traffic-sign-recognition.azure-devices.net;DeviceId=MyNodeDevice;SharedAccessKey=R9R936zpAvNJVkgVT+rYKVySjio300+oBx1LEsTk1oE=';
var Mqtt = require('azure-iot-device-mqtt').Mqtt;
var DeviceClient = require('azure-iot-device').Client
var Message = require('azure-iot-device').Message;
var client = DeviceClient.fromConnectionString(connectionString, Mqtt);

function printResultFor(op) {
	  return function printResult(err, res) {
		      if (err) console.log(op + ' error: ' + err.toString());
		          if (res) console.log(op + ' status: ' + res.constructor.name);
			    };
}

function listener() {
  rosnodejs.initNode('/listener_node')
    .then((rosNode) => {
      let sub = rosNode.subscribe('/iothub', std_msgs.String,
        (data) => { 
          rosnodejs.log.info('I heard: [' + data.data + ']');
	  var data = JSON.stringify({ sign: data.data});
	  var message = new Message(data);

	  console.log('Sending message: ' + message.getData());
	  client.sendEvent(message, printResultFor('send'));
	}
      );
    });
}

if (require.main === module) {
  listener();
}
