"use strict";

// Basic vars
var HTTPPORT = 81,
    USERBAUD = 115200,
    USERHTREFERENCE = { wref: null, xref: null, yref: null, zref: null, status: false },
    operatingSystem = process.platform, // 'darwin'=mac, 'freebsd', 'linux', 'sunos' 'win32'=windows
    LOGGING = false,
    USERPORT = process.argv[3] || "\\\\.\\COM7"

process.argv.forEach((val, index) => {
    switch (val) {
        case 'logging':
            LOGGING = true
            console.log("* Logging is turned on")
            break;
        case 'port':
            HTTPPORT = process.argv[index + 1]
            break;
        default:
            //console.log('');
    }
});

console.dir(process.version)
console.dir(process.versions)

// Service
const express = require('express')
const expApp = express()
const http = require('http').Server(expApp)
const io = require('socket.io')(http)
var ascii85 = require('ascii85').ZeroMQ

// Serial
const SerialPort = require('serialport')

// BLE
if(operatingSystem == 'win32')
{
    var noble = require('noble-uwp')
    var USERPORTPRE = "\\\\.\\"
}
else
{
    var noble = require('noble')
    var USERPORTPRE = ""
}

// Transform stream
const Transform = require('stream').Transform;
const StringDecoder = require('string_decoder').StringDecoder;
const decoder = new StringDecoder('utf8');

const eodBreaker = new Transform({
    transform(chunk, encoding, cb)
    {
        if ( this._last === undefined ) { this._last = "" }
        this._last += decoder.write(chunk);
        var list = this._last.split(/;\r\n/);
        this._last = list.pop();
        for (var i = 0; i < list.length; i++) {
            this.push( list[i] );
        }
        cb();
    },
    flush(cb)
    {
        this._last += decoder.end()
        if (this._last) { push(this, this._last) }
        cb()
    }
});

// ========================
// SERIAL
function SerialGo(pPort)
{
	this.usrPort = pPort;
	this.usrBaud = 115200;
    this.port = '';
    this.usbPortsFound = []
}

SerialGo.prototype.listAvailablePorts = function()
{
    SerialPort.list(function(error, ports)
    {
        if (error)
        {
            console.log("Port scan error, ",error)
        }
        else
        {
            this.usbPortsFound = ports
            console.dir(this.usbPortsFound)
            return this.usbPortsFound
        }
    })
}

SerialGo.prototype.start = function()
{
	console.log("Start serial watch: ", this.usrPort)

	this.port = new SerialPort(this.usrPort, {
		baudrate: this.usrBaud,
		autoOpen: true,
		parser: SerialPort.parsers.raw //.readline('"') SerialPort.parsers.readline('\;')
	}, (err) => {
		console.log("Could not open serialport")
		console.log(err)
	})

    this.port.on('error', (err) =>
    {
		console.log("+ ",err)
	})

    this.port.on('open', () =>
    {
		console.log("+ Port is open")
	})

	var _chunks = [];
	var _chunk = ""
    this.port.pipe( eodBreaker ).on('data', (datachunk) =>
    {
		io.emit('htq',datachunk.toString('utf-8').split(',').map(function(t){return parseFloat(t)}))
	})
};

SerialGo.prototype.stop = function()
{
	console.log("Stop serial watch: ", this.usrPort)

    if (typeof this.port.close  !== 'undefined')
    {
        this.port.close(function()
        {
            console.log("Successfully closed port watch");
        })
    }
};

// TODO: Do it on th eobject instead???
var checkPeriperals = function()
{
    return new Promise((resolve, reject) =>
    {
        SerialPort.list(function(err, ports)
        {
            if(err)
            {
				console.log(err)
				reject(err)
			}
			resolve(ports)
		})
	})
}

// export the class
module.exports = SerialGo;

// ========================
// BLUETOOTH
function BluetoothGo(noble)
{
    this.ble = noble
    this.bleState = noble.state
    this.bleDeviceState = "disconnected"
    this.bleDeviceConnectedTo = { 'uuid' : '', 'localname' : '', 'subscription' : false }
    this.bleError = "";
    this.bleScanning = false;
    this.bleSubscribedCharacteristics = []
    this.bleControlWriteChannel = ""
    this.bleUnitsFound = {}
    this.bleUnitsFoundReadable = []

    this.ble.on('stateChange', (state) =>
    {
        console.log('\x1b[47m%s\x1b[0m: ', 'BLE stateChange', state) //<"unknown" | "resetting" | "unsupported" | "unauthorized" | "poweredOff" | "poweredOn">
        this.bleState = state

        if (state!="poweredOn")
        {
            this.ble.stopScanning()
        }
        this.emit()
    })

    this.ble.on('scanStart', (state) =>
    {
        console.log('\x1b[42m%s\x1b[0m: ', 'BLE scanStart','true')
        this.bleScanning = true
        this.bleUnitsFoundReadable = []
        this.emit()
    })

    this.ble.on('scanStop', (peripherals) =>
    {
        console.log('\x1b[45m%s\x1b[0m: ', 'BLE scanStop','true')
        this.bleScanning = false
        this.emit()
    })

    this.ble.on('discover', (peripheral) =>
    {
        console.log("Discovering", peripheral.advertisement.localName)

        this.bleUnitsFound[peripheral.advertisement.localName] = { unit: peripheral }
        this.bleUnitsFoundReadable.push(peripheral.advertisement)
        io.emit("scanned-ports ble", peripheral.advertisement)

        //this.connectTo(peripheral)
    })
}

BluetoothGo.prototype.connectToWithName = function(deviceName)
{
    if (this.bleUnitsFound.hasOwnProperty(deviceName))
    {
        this.connectTo(this.bleUnitsFound[deviceName].unit);
    }
    else
    {
        return "Could not connect to device, no such device";
    }
}

BluetoothGo.prototype.connectTo = function(devicePheriperal)
{
    var that = this
    if (devicePheriperal.advertisement.localName == 'OHTI')
    {
        console.log("Trying to Connect with "+devicePheriperal.advertisement.localName+" unit: ", devicePheriperal.advertisement)

        devicePheriperal.disconnect((error) =>
        {
            if (error)
            {
                console.log("Error devicePheriperal disconnect: ", error)
            }
            this.bleDeviceState = "disconnected"
            this.bleDeviceConnectedTo = { 'uuid' : '', 'localname' : '', 'subscription' : false };
            this.emit()
        })

        devicePheriperal.connect((error) =>
        {
            if (error)
            {
                console.log("Error devicePheriperal connect: ", error)
                this.bleDeviceState = "disconnected"
                this.bleDeviceConnectedTo = { 'uuid' : '', 'localname' : '', 'subscription' : false };
                this.emit()
            }
            else
            {
                console.log('\x1b[36m%s\x1b[0m', 'Connected to UUID: ' , devicePheriperal.uuid)
                this.bleDeviceState = "connected"
                this.bleDeviceConnectedTo = { 'uuid' : devicePheriperal.uuid, 'localname' : devicePheriperal.advertisement.localName, 'subscription' : false };
                this.emit()

                devicePheriperal.discoverAllServicesAndCharacteristics((error, services, characteristic) => {

                    console.log("Services: " , services.length)

                    for(var y = 0; y < services.length; y++)
                    {
                        console.log('\x1b[45m%s\x1b[0m: ', "========================")
                        for(var x = 0; x < services[y].characteristics.length; x++) {

                            console.log("Service Name: " + services[y].characteristics[x].name + 
                                        ", UUID: " + services[y].characteristics[x].uuid + 
                                        ", Type: " + services[y].characteristics[x].type + 
                                        ", Properties: " + services[y].characteristics[x].properties)

                            /*services[y].characteristics[x].read(function (error, data)
                            {
                                // TODO is this the same as below
                                if(typeof data != 'undefined' && data != null)
                                {
                                    console.log("@Characteristics read: ", data.toString())
                                }
                                else
                                {
                                    console.log("@Characteristics read undefined ")
                                }

                                if(typeof error != 'undefined' && error != null)
                                {
                                    console.log("@Characteristics read error: " + error.toString())
                                }
                            })*/
                            console.log('\x1b[42m%s\x1b[0m: ', "_____________________")
                            //.pipe( eodBreaker )
                            var _chunks = [];
                            var _chunk = ""

                            services[y].characteristics[x].on('data', function (data, isNotification, more)
                            {
                                if (typeof data != 'undefined' && data != null)
                                {
                                    if (isNotification)
                                    {
                                        let stringData = data.toString()
                                        let stringDataNoWhite = stringData.replace(/^\s+|\s+$/g, '')

                                        if (stringDataNoWhite.length > 0)
                                        {
                                            let dataDecode = ascii85.decode(stringData)

                                            // Little endian
                                            let quat16b1 = parseFloat(dataDecode.readInt16LE(0, true) / 16384 )
                                            let quat16b2 = parseFloat(dataDecode.readInt16LE(2, true) / 16384 )
                                            let quat16b3 = parseFloat(dataDecode.readInt16LE(4, true) / 16384 )
                                            let quat16b4 = parseFloat(dataDecode.readInt16LE(6, true) / 16384 )

                                            io.emit('htq',[quat16b1,quat16b2,quat16b3,quat16b4])

                                            if(LOGGING)
                                            {
                                                console.log("===================")
                                                console.log("QUAT Decoded:",quat16b1,",",quat16b2,",",quat16b3,",",quat16b4,
                                                            "Stringdata: " + stringData + ", Incoming lenght: ",stringData.length, ", Decode lenght:", dataDecode.length)
                                            }
                                        }
                                    }
                                    else
                                    {
                                        console.log("@Data Info: ", data.toString());
                                    }
                                }
                            }.bind(this))

                            // Check if the characteristics is writable
                            if (services[y].characteristics[x].uuid == '6e400002b5a3f393e0a9e50e24dcca9e')
                            {
                                that.bleControlWriteChannel = services[y].characteristics[x];
                                console.dir(that.bleControlWriteChannel);
                                console.log("> Found characteristic for writing");
                            }

                            // to enable notify
                            // todo: not on every...
                            if ( services[y].characteristics[x].properties == 'notify' )
                            { //if( services[y].characteristics[x].uuid == '6e400003b5a3f393e0a9e50e24dcca9e' ) {
                                let tmpCharacteristic = services[y].characteristics[x]
                                that.bleSubscribedCharacteristics.push(tmpCharacteristic)

                                tmpCharacteristic.subscribe(function(error) {
                                    if (error)
                                    {
                                        that.bleDeviceConnectedTo.subscription = false;
                                        console.log("Error subscribing to characteristic: ")
                                        console.dir(error);
                                    }
                                    that.bleDeviceConnectedTo.subscription = true;
                                    console.log('Notification on characteristics is on, Roger check if this is sent before data starts coming in from sensor...');
                                });
                            }

                        }
                    }
                })
            }

        })
    }
}

BluetoothGo.prototype.emit = function()
{
    // emit general status message
    io.emit('feedback ble-info', { 
        state: this.bleState, 
        scanning: this.bleScanning, 
        connection: this.bleDeviceState, 
        connectedTo: this.bleDeviceConnectedTo, 
        error: this.bleError, 
        result: this.bleUnitsFoundReadable })
}

BluetoothGo.prototype.startScanning = function()
{
    var serviceUUIDs = [] // default: [] => all
    var allowDuplicates = true // allow duplicate peripheral to be returned (default false) on discovery event

    if (this.ble.state == "poweredOn")
    {
        this.ble.startScanning(serviceUUIDs, allowDuplicates[ (error) =>
        {
            console.log("Error start scanning: ", error)
            this.bleError = "Error while starting to scan"
            this.emit()
        }])
    }
    else if(this.ble.state == "poweredOff")
    {
        this.ble.stopScanning()
    }
}

BluetoothGo.prototype.stopScanning = function()
{
    if(this.ble.state == "poweredOn" && this.bleScanning) {
        this.ble.stopScanning()
    }
}

BluetoothGo.prototype.stopCharacteristics = function()
{
    var that = this;
    this.bleSubscribedCharacteristics.forEach(function(element)
    {
        element.unsubscribe(function(error)
        {
            if (error)
            {
                console.log('Error: Unsubscribing to notification on characteristics is unchanged')
                console.dir(error)
            }
            else
            {
                that.bleDeviceConnectedTo.subscription = false;
                console.log('Notification on characteristics is off, unsubscribed')
            }
        });
    }, this);
}

BluetoothGo.prototype.startCharacteristics = function()
{
    var that = this;
    this.bleSubscribedCharacteristics.forEach(function(element)
    {
        element.subscribe(function(error)
        {
            if (error)
            {
                that.bleDeviceConnectedTo.subscription = false;
                console.log('Error: Subscribing to notification on characteristics is unchanged')
                console.dir(error)
            }
            else
            {
                that.bleDeviceConnectedTo.subscription = true;
                console.log('Notification on characteristics is on, subscribed')
            }
        });
    }, this);
}

BluetoothGo.prototype.startCalibration = function(payload)
{
    console.log("Start calibration, command: ", payload)
    //var payload = '2,2;';
    var payload = Buffer.from(payload, 'utf8');
    //console.dir(this.bleControlWriteChannel._noble._bindings);
    if (this.bleControlWriteChannel != undefined)
    {
        this.bleControlWriteChannel.write(payload, function(error)
        {
            if (!error)
            {
                console.log('Calibration information sent')
            }
            else
            {
                console.log('Could not trigger calibration')
            }
        });
    }
}

// export the class
module.exports = BluetoothGo;

// ========================
// USER CONNECTION & SERVICE - APP

var SerialObj = new SerialGo(USERPORT);
var BluetoothObj = new BluetoothGo(noble);

expApp.use(express.static('node_modules'))
expApp.use(express.static('static'))

expApp.get('/', function(req,res){
    res.sendFile(__dirname + '/index.html')
})

expApp.get('/omni', function(req,res){
    res.sendFile(__dirname + '/index_blebr.html')
})

expApp.get('/ambi', function(req,res){
    res.sendFile(__dirname + '/index_blebr_politis.html')
})

expApp.get('/model.json', function(req,res){
    res.sendFile(__dirname + '/ladies-head.json')
})

expApp.get('/get/:what', function (req, res) {
    if(req.params.what == 'ble-units') {
        res.setHeader('Content-Type', 'application/json');
        res.send(JSON.stringify({ a: 1 }));
    }
})

console.log("Application started, Port:",USERPORT,", Baud:",USERBAUD,", Http-port:",HTTPPORT)

io.on('connection', function(Socket)
{
	console.log('> WS client connected')
	// Init values
	Socket.send('#################');
    io.emit('feedback serial-info', { port: USERPORT, baud: USERBAUD })
    /*io.emit('feedback ble-info', { state: BluetoothObj.bleState, 
        scanning: BluetoothObj.bleScanning, 
        connection: BluetoothObj.bleDeviceState, 
        connectedTo: BluetoothObj.bleDeviceConnectedTo, 
        result: BluetoothObj.bleUnitsFoundReadable } )*/
    BluetoothObj.emit()
    io.emit('feedback headtrack-info', { htreference: USERHTREFERENCE } )
    // TODO send
    //io.emit("scanned-ports ble", BluetoothObj.bleUnitsFound)

    Socket.on('disconnect', function()
    {
		console.log('> User disconnected')
	});

	// Check ports and emit
	checkPeriperals()
        .then((data) =>
        {
			// Com Port is found
			console.log(data)
			io.emit('scanned-ports', data)
		})
        .catch((err) =>
        {
			console.log("Error ports" +err)
			io.emit('scanned-ports', {})
		})


    // ------------------------
    // BLE CONTROL
    Socket.on('ble-start-scan', (msg) =>
    {
        // User starts BLE discovering
        console.log("> Starting BLE discovering: " + msg)
        BluetoothObj.startScanning();
    })

    Socket.on('ble-stop-scan', (msg) =>
    {
        // User stops BLE discovering
        console.log("> Stopping BLE discovering: " + msg)
        BluetoothObj.stopScanning();
    })

    Socket.on('ble-connect-to', (msg) =>
    {
        // User connects to device
        console.log("> Connecting to device: " + msg)
        BluetoothObj.connectToWithName(msg);
    })

    Socket.on('ble-start-notify-sensor', (msg) =>
    {
		// User starts tracking
		console.log("> Starting watch BLE: " + msg)
		BluetoothObj.startCharacteristics();
    })

    Socket.on('ble-stop-notify-sensor', (msg) =>
    {
		// User stops characteristics subscription tracking
		console.log("> Stopping watch BLE: " + msg)
		BluetoothObj.stopCharacteristics();
    })

    Socket.on('ble-calibrate-sensor', (msg) =>
    {
		// User starts calibration
		console.log("> Sending instructions to start calibration BLE: " + msg)
		BluetoothObj.startCalibration(msg);
    })

    // ------------------------
    // SERIAL CONTROL
    Socket.on('usb-start-scan', (msg) =>
    {
        // User starts Serial USB discovering
        console.log("> Starting Serial USB discovering: " + msg)
        SerialObj.listAvailablePorts();
    })

    Socket.on('usb-start-track', (msg) =>
    {
		// User starts tracking
		console.log("> Starting Serial USB watch: " + msg + ", on port: " + USERPORT)
		SerialObj.usrPort = USERPORT
		SerialObj.start();
    })

    Socket.on('usb-stop-track', (msg) =>
    {
		// User stops tracking
		console.log("> Stopping Serial USB watch: " + msg + ", on port: " + USERPORT)
		if(SerialObj !== null) {
			SerialObj.stop();
		} else {
			console.log("Serial OBJ is null")
		}
	})

    Socket.on('usb-save-settings', (msg) =>
    {
		// User saves COM-port
		console.log("> Saving usb and serial: " + msg)

        if ( msg.hasOwnProperty('input_headtrack_port') )
        {
            USERPORT = USERPORTPRE + msg.input_headtrack_port
            SerialObj.usrPort = USERPORTPRE + msg.input_headtrack_port
        }

        if ( msg.hasOwnProperty('input_headtrack_baud') )
        {
            USERBAUD = msg.input_headtrack_baud
            SerialObj.usrBaud = msg.input_headtrack_baud
        }

		io.emit('feedback serial-info', { port: USERPORT, baud: USERBAUD })
	})

    // ------------------------
    // CONFIG - REF VALUE
    Socket.on('save-user config-options', function(msg)
    {
		console.dir('> Save-user config-options: ' + msg)

        if ( msg.hasOwnProperty('wref') )
        {
            // HT reference auto
            // { wref: null, xref: null, yref: null, zref: null, status: false },
            USERHTREFERENCE = msg
            console.dir(msg);
        }
        else
        {
			console.log("> Nothing to save")
		}
	})
})

http.listen(HTTPPORT, function()
{
	console.log('> server listening on :' + HTTPPORT)
})

http.on('error', function (e)
{
  // Error starting web-server
  console.log("HTTPPORT is occupied, start as sudo")
  console.log(e);
})