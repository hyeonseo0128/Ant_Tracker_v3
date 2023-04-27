const mqtt = require('mqtt');
const fs = require('fs');
const { nanoid } = require("nanoid");
const { SerialPort } = require('serialport');

let drone_info = {};
try {
    drone_info = JSON.parse(fs.readFileSync('../drone_info.json', 'utf8'));
} catch (e) {
    console.log('can not find [ ../tracker_info.json ] file');
    drone_info.host = "gcs.iotocean.org"
    drone_info.drone = "Flight_2"
    drone_info.gcs = "KETI_MUV"
    drone_info.type = "ardupilot"
    drone_info.system_id = 1
    drone_info.update = "disable"
    drone_info.mission = {}
    drone_info.id = ae_name.flight

    fs.writeFileSync('../drone_info.json', JSON.stringify(drone_info, null, 4), 'utf8');
}

// ---------- set values ---------- 
const PAN_CAN_ID = '000000010000';
const TILT_CAN_ID = '000000020000';

const P_MIN = -12.500;
const P_MAX = 12.500;
const V_MIN = -65.000;
const V_MAX = 65.000;
const KP_MIN = 0.000;
const KP_MAX = 500.000;
const KD_MIN = 0.000;
const KD_MAX = 5.000;
const T_MIN = -18.000;
const T_MAX = 18.000;

let p_offset = 0.24;

let pan_p_in = 0.000 + p_offset;
let pan_v_in = 0.000;
let pan_kp_in = 20.000;
let pan_kd_in = 1.000;
let pan_t_in = 0.000;

let pan_p_out = 0.000;
let pan_v_out = 0.000;
let pan_t_out = 0.000;

let pan_p_step = 0.02;
let pan_p_target = 0.0;

let tilt_p_in = 0.000 + p_offset;
let tilt_v_in = 0.000;
let tilt_kp_in = 20.000;
let tilt_kd_in = 1.000;
let tilt_t_in = 0.000;

let tilt_p_out = 0.000;
let tilt_v_out = 0.000;
let tilt_t_out = 0.000;

let tilt_p_step = 0.02;
let tilt_p_target = 0.0;

let cw = 0;
let ccw = 0;
let cur_angle = 0;
let temp_angle = 0;
let turn_angle = 0.0;

let pan_motormode = 2;
let tilt_motormode = 2;
let run_flag = '';

let pan_no_response_count = 0;
let tilt_no_response_count = 0;

// let can_port_num = '/dev/ttyUSB0';
let can_port_num = 'COM5';
let can_baudrate = '115200';
let can_port = null;

let local_mqtt_host = 'localhost';
let localmqtt = '';

let localmqtt_message = '';
let pan_motor_control_message = '';
let tilt_motor_control_message = '';


let myLatitude = 37.4041;
let myLongitude = 127.1607;
let myRelativeAltitude = 10;
let myHeading = '';

let myRoll = 0;
let myPitch = 0;
let myYaw = 0;

let myNumSatellites = 0;

let target_latitude = '';
let target_longitude = '';
// let target_altitude = '';
let target_relative_altitude = '';

let motor_return_msg = '';
let tracker_location_msg = '';
let tracker_attitude_msg = '';
let tracker_status_msg = '';


// let sub_drone_data_topic = '/RF/TELE_HUB/drone';
// let sub_gps_location_topic = '/GPS/location';
let sub_motor_control_topic = '/Ant_Tracker/Control';
let sub_drone_data_topic = '/gcs/TELE_HUB/drone/rf/' + drone_info.drone;
let sub_gps_location_topic = '/GPS/location';
let sub_gps_attitude_topic = '/GPS/attitude';
let sub_gps_status_topic = '/GPS/status';

let pub_pan_motor_position_topic = '/Ant_Tracker/Motor_Pan';
let pub_tilt_motor_position_topic = '/Ant_Tracker/Motor_Tilt';

let switchCount = 0;

let sitl_state = true;
let sitl_mqtt_host = 'gcs.iotocean.org';
let sitlmqtt = '';

let sitlmqtt_message = '';
let sub_sitl_drone_data_topic = '/Mobius/KETI_GCS/Drone_Data/KETI_Simul_1';

//------------- Can communication -------------
function canPortOpening() {
    if (can_port == null) {
        can_port = new SerialPort({
            path: can_port_num,
            baudRate: parseInt(can_baudrate, 10),
        });

        can_port.on('open', canPortOpen);
        can_port.on('close', canPortClose);
        can_port.on('error', canPortError);
        can_port.on('data', canPortData);
    } else {
        if (can_port.isOpen) {
            can_port.close();
            can_port = null;
            setTimeout(canPortOpening, 2000);
        } else {
            can_port.open();
        }
    }
}

function canPortOpen() {
    console.log('[tracker_motor] canPort open. ' + can_port_num + ' Data rate: ' + can_baudrate);

    localMqttConnect(local_mqtt_host);
    if (sitl_state === true) { // SITL 드론 연동시 mqtt 연결
        sitlMqttConnect(sitl_mqtt_host);

    }
}

function canPortClose() {
    console.log('[tracker_motor] canPort closed.');

    setTimeout(canPortOpening, 2000);
}

function canPortError(error) {
    let error_str = error.toString();
    console.log('[tracker_motor] canPort error: ' + error.message);
    if (error_str.substring(0, 14) == "Error: Opening") {

    } else {
        console.log('[tracker_motor] canPort error : ' + error);
    }

    setTimeout(canPortOpening, 2000);
}
let _msg = '';
function canPortData(data) {
    _msg += data.toString('hex').toLowerCase();

    setInterval(() => {
        if (_msg.length >= 24) {
            if (_msg.substring(0, 10) === '0000000001' || _msg.substring(0, 10) === '0000000002') {
                motor_return_msg = _msg.substring(0, 24);
                _msg = _msg.substring(24, _msg.length);
                // console.log('motor_return_msg: ', motor_return_msg);
            }
        }
    }, 1);

    // let _msg = data.toString('hex').toLowerCase();
    // if (_msg.length >= 24) {
    //     if (_msg.substring(0, 10) === '0000000001' || _msg.substring(0, 10) === '0000000002') {
    //         motor_return_msg = _msg;
    //         // console.log('motor_return_msg: ', motor_return_msg);
    //     } else {
    //         motor_return_msg = '';
    //     }
    // } else {
    //     motor_return_msg = '';
    // }

}

canPortOpening();
//---------------------------------------------------


//------------- local mqtt connect ------------------
function localMqttConnect(host) {
    let connectOptions = {
        host: host,
        port: 1883,
        protocol: "mqtt",
        keepalive: 10,
        clientId: 'local_' + nanoid(15),
        protocolId: "MQTT",
        protocolVersion: 4,
        clean: true,
        reconnectPeriod: 2000,
        connectTimeout: 2000,
        rejectUnauthorized: false
    }

    localmqtt = mqtt.connect(connectOptions);

    localmqtt.on('connect', function () {
        localmqtt.subscribe(sub_drone_data_topic + '/#', () => {
            console.log('localmqtt subscribed -> ', sub_drone_data_topic);
        });
        localmqtt.subscribe(sub_motor_control_topic + '/#', () => {
            console.log('localmqtt subscribed -> ', sub_motor_control_topic);
        });
        localmqtt.subscribe(sub_gps_location_topic + '/#', () => {
            console.log('localmqtt subscribed -> ', sub_gps_location_topic);
        });
        localmqtt.subscribe(sub_gps_attitude_topic + '/#', () => {
            console.log('localmqtt subscribed -> ', sub_gps_attitude_topic);
        });
        localmqtt.subscribe(sub_gps_status_topic + '/#', () => {
            console.log('localmqtt subscribed -> ', sub_gps_status_topic);
        });
    });

    localmqtt.on('message', function (topic, message) {
        // console.log('topic, message => ', topic, message);

        if (topic == sub_motor_control_topic) { // 모터 제어 메세지 수신
            pan_motor_control_message = message.toString();
            tilt_motor_control_message = message.toString();
            // console.log(topic, pan_motor_control_message);
        } else if (topic.includes(sub_drone_data_topic)) { // 드론데이터 수신
            localmqtt_message = message.toString('hex');
            // console.log("Client1 topic => " + topic);
            // console.log("Client1 message => " + drone_message);

            try {
                let ver = localmqtt_message.substring(0, 2);
                let sysid = '';
                let msgid = '';
                let base_offset = 0;

                if (ver == 'fd') {//MAV ver.1
                    sysid = localmqtt_message.substring(10, 12).toLowerCase();
                    msgid = localmqtt_message.substring(18, 20) + localmqtt_message.substring(16, 18) + localmqtt_message.substring(14, 16);
                    base_offset = 20;
                } else { //MAV ver.2
                    sysid = localmqtt_message.substring(6, 8).toLowerCase();
                    msgid = localmqtt_message.substring(10, 12).toLowerCase();
                    base_offset = 12;
                }

                let sys_id = parseInt(sysid, 16);
                let msg_id = parseInt(msgid, 16);

                if (msg_id === 33) { // MAVLINK_MSG_ID_GLOBAL_POSITION_INT
                    var time_boot_ms = mavPacket.substring(base_offset, base_offset + 8).toLowerCase()
                    base_offset += 8
                    let lat = localmqtt_message.substring(base_offset, base_offset + 8).toLowerCase().toString();
                    base_offset += 8;
                    let lon = localmqtt_message.substring(base_offset, base_offset + 8).toLowerCase();
                    base_offset += 8;
                    let alt = localmqtt_message.substring(base_offset, base_offset + 8).toLowerCase();
                    base_offset += 8;
                    let relative_alt = localmqtt_message.substring(base_offset, base_offset + 8).toLowerCase();

                    target_latitude = Buffer.from(lat, 'hex').readInt32LE(0).toString() / 10000000;
                    target_longitude = Buffer.from(lon, 'hex').readInt32LE(0).toString() / 10000000;
                    target_altitude = Buffer.from(alt, 'hex').readInt32LE(0).toString() / 1000;
                    target_relative_altitude = Buffer.from(relative_alt, 'hex').readInt32LE(0).toString() / 1000;

                    calcTargetPanAngle(target_latitude, target_longitude);
                    calcTargetTiltAngle(target_latitude, target_longitude, target_relative_altitude);
                    // console.log('target_latitude, target_longitude, target_altitude, target_relative_altitude', target_latitude, target_longitude, target_altitude, target_relative_altitude);
                }
            }
            catch (e) {
                console.log('[tracker_motor] local mqtt connect Error', e);
            }
        } else if (topic === sub_gps_location_topic) { // 픽스호크로부터 받아오는 트래커 위치 좌표
            tracker_location_msg = JSON.parse(message.toString());
            // myLatitude = tracker_location_msg.lat;
            // myLongitude = tracker_location_msg.lon;
            // myRelativeAltitude = tracker_location_msg.relative_alt;
            myHeading = tracker_location_msg.hdg;
            // console.log('tracker_location_msg: ', myLatitude, myLongitude, myRelativeAltitude, myHeading);
        } else if (topic === sub_gps_attitude_topic) {
            tracker_attitude_msg = JSON.parse(message.toString());
            myRoll = tracker_attitude_msg.roll;
            myPitch = tracker_attitude_msg.pitch;
            myYaw = tracker_attitude_msg.yaw;
            // console.log('tracker_location_msg: ', myRoll, myPitch, myYaw);
        } else if (topic === sub_gps_status_topic) {
            tracker_status_msg = JSON.parse(message.toString());
            myNumSatellites = tracker_status_msg.num_satellites;
            // console.log('tracker_location_msg: ', myNumSatellites);
        }
    });

    localmqtt.on('error', function (err) {
        console.log('[tracker_motor] local mqtt connect error ' + err.message);
        localmqtt = null;
        setTimeout(localMqttConnect, 1000, local_mqtt_host);
    });

    runMotor();
}
//---------------------------------------------------

function runMotor() {
    setTimeout(() => {
        pan_motor_control_message = 'init';
        tilt_motor_control_message = 'init';
    }, 3000);

    setInterval(() => {
        localmqtt.publish(pub_pan_motor_position_topic, (pan_p_out * 180 / Math.PI).toString(), () => {
            // console.log('[pan] send Motor angle to GCS value: ', p_out * 180 / Math.PI)
        });
        localmqtt.publish(pub_tilt_motor_position_topic, (tilt_p_out * 180 / Math.PI).toString(), () => {
            // console.log('[pan] send Motor angle to GCS value: ', p_out * 180 / Math.PI)
        });
    }, 500);

    // pan motor control message
    setTimeout(() => {
        setInterval(() => {
            let panControl = () => {
                if (pan_motor_control_message == 'on') {
                    EnterMotorMode(PAN_CAN_ID);
                    pan_motormode = 1;
                    pan_motor_control_message = '';
                }
                else if (pan_motor_control_message == 'off') {
                    ExitMotorMode(PAN_CAN_ID);
                    pan_motormode = 0;
                    pan_motor_control_message = '';
                    run_flag = '';
                }
                else if (pan_motor_control_message == 'zero') {
                    Zero(PAN_CAN_ID);
                    pan_p_in = 0 + p_offset;
                    pan_motor_control_message = '';
                }
                else if (pan_motor_control_message == 'init') {
                    if (pan_motormode !== 1) {
                        EnterMotorMode(PAN_CAN_ID);
                        pan_motormode = 1;
                        initActionPan(PAN_CAN_ID);
                        pan_motor_control_message = '';
                    } else {
                        initActionPan(PAN_CAN_ID);
                        pan_motor_control_message = '';
                    }
                }

                if (pan_motormode === 1) {
                    if (pan_motor_control_message == 'pan_up') {
                        pan_p_in = pan_p_in + pan_p_step;
                    }
                    else if (pan_motor_control_message == 'pan_down') {
                        pan_p_in = pan_p_in - pan_p_step;
                    }
                    else if (pan_motor_control_message == 'stop') {
                        pan_motor_control_message = '';
                        run_flag = '';
                    }
                    else if (pan_motor_control_message.includes('go')) {
                        pan_p_target = (parseInt(pan_motor_control_message.toString().replace('pan_go', '')) * 0.0174533) + p_offset;

                        if (pan_p_target < pan_p_in) {
                            pan_p_in = pan_p_in - pan_p_step;
                        }
                        else if (pan_p_target > pan_p_in) {
                            pan_p_in = pan_p_in + pan_p_step;
                        }
                    }
                    else if (pan_motor_control_message == 'run') {
                        run_flag = 'go';
                        if (pan_p_target < pan_p_in) {
                            pan_p_in = pan_p_in - pan_p_step;
                        }
                        else if (pan_p_target > pan_p_in) {
                            pan_p_in = pan_p_in + pan_p_step;
                        }
                    }

                    pan_p_in = constrain(pan_p_in, P_MIN, P_MAX);

                    pack_cmd(PAN_CAN_ID, pan_p_in, pan_v_in, pan_kp_in, pan_kd_in, pan_t_in);


                    pan_no_response_count++;

                    if (motor_return_msg !== '') {
                        unpack_reply();
                        pan_no_response_count = 0;

                        motor_return_msg = '';
                        // console.log('[pan] -> ', pan_p_target, pan_p_in, pan_p_out, pan_v_out, pan_t_out);
                    }
                }

                if (pan_no_response_count > 48) {
                    console.log('[pan] No_response: ', pan_no_response_count);
                    pan_no_response_count = 0;
                    pan_motormode = 2;
                }
            }

            let tiltControl = () => {
                if (tilt_motor_control_message == 'on') {
                    EnterMotorMode(TILT_CAN_ID);
                    tilt_motormode = 1;
                    tilt_motor_control_message = '';
                }
                else if (tilt_motor_control_message == 'off') {
                    ExitMotorMode(TILT_CAN_ID);
                    tilt_motormode = 0;
                    tilt_motor_control_message = '';
                    run_flag = '';
                }
                else if (tilt_motor_control_message == 'zero') {
                    Zero(TILT_CAN_ID);
                    tilt_p_in = 0 + p_offset;
                    tilt_motor_control_message = '';
                }
                else if (tilt_motor_control_message == 'init') {
                    if (tilt_motormode !== 1) {
                        EnterMotorMode(TILT_CAN_ID);
                        tilt_motormode = 1;
                        initActionTilt(TILT_CAN_ID);
                        tilt_motor_control_message = '';
                    } else {
                        initActionTilt(TILT_CAN_ID);
                        tilt_motor_control_message = '';
                    }
                }

                if (tilt_motormode === 1) {
                    if (tilt_motor_control_message == 'tilt_up') {
                        tilt_p_in = tilt_p_in + tilt_p_step;
                    }
                    else if (tilt_motor_control_message == 'tilt_down') {
                        tilt_p_in = tilt_p_in - tilt_p_step;
                    }
                    else if (tilt_motor_control_message == 'stop') {
                        tilt_motor_control_message = '';
                        run_flag = '';
                    }
                    else if (tilt_motor_control_message.includes('go')) {
                        tilt_p_target = (parseInt(tilt_motor_control_message.toString().replace('go', '')) * 0.0174533) + p_offset;

                        if (tilt_p_target < tilt_p_in) {
                            tilt_p_in = tilt_p_in - tilt_p_step;
                        }
                        else if (tilt_p_target > tilt_p_in) {
                            tilt_p_in = tilt_p_in + tilt_p_step;
                        }
                    }
                    else if (tilt_motor_control_message == 'run') {
                        run_flag = 'go';
                        if (tilt_p_target < tilt_p_in) {
                            tilt_p_in = tilt_p_in - tilt_p_step;
                        }
                        else if (tilt_p_target > tilt_p_in) {
                            tilt_p_in = tilt_p_in + tilt_p_step;
                        }
                    }

                    tilt_p_in = constrain(tilt_p_in, P_MIN, P_MAX);

                    pack_cmd(TILT_CAN_ID, tilt_p_in, tilt_v_in, tilt_kp_in, tilt_kd_in, tilt_t_in);

                    tilt_no_response_count++;

                    if (motor_return_msg !== '') {
                        unpack_reply();
                        tilt_no_response_count = 0;

                        motor_return_msg = '';
                        // console.log('[tilt] -> ', tilt_p_target, tilt_p_in, tilt_p_out, tilt_v_out, tilt_t_out);
                    }
                }

                if (tilt_no_response_count > 48) {
                    console.log('[tilt] no_response_count', tilt_no_response_count);
                    tilt_no_response_count = 0;
                    tilt_motormode = 2;
                }
            }

            let turn = switchCount++ % 2;

            if (turn) {
                tiltControl();
            }
            else {
                panControl();
            }
        }, 12);
    }, 1000);

    setInterval(() => {
        if (pan_motormode === 2) {
            ExitMotorMode(PAN_CAN_ID);

            setTimeout(() => {
                if (motor_return_msg !== '') {
                    unpack_reply();

                    motor_return_msg = '';
                    pan_p_in = pan_p_out + p_offset;

                    console.log('[pan] ExitMotorMode', pan_p_in, pan_p_out, pan_v_out, pan_t_out);
                }
            }, 500)
        }
    }, 1000);

    setInterval(() => {
        if (tilt_motormode === 2) {
            ExitMotorMode(TILT_CAN_ID);

            setTimeout(() => {
                if (motor_return_msg !== '') {
                    unpack_reply();

                    motor_return_msg = '';
                    tilt_p_in = tilt_p_out + p_offset;

                    console.log('[tilt] ExitMotorMode', tilt_p_in, tilt_p_out, tilt_v_out, tilt_t_out);
                }
            }, 500)
        }
    }, 1000);

}

let constrain = (_in, _min, _max) => {
    if (_in < _min) {
        return _min;
    }
    else if (_in > _max) {
        return _max;
    }
    else {
        return _in;
    }
}

let initActionPan = () => {
    setTimeout(() => {
        pan_motor_control_message = 'zero';
        setTimeout(() => {
            pan_motor_control_message = 'pan_up';
            setTimeout(() => {
                pan_motor_control_message = 'pan_down';
                setTimeout(() => {
                    pan_motor_control_message = 'stop';
                }, 2000);
            }, 2000);
        }, 1000);
    }, 500);
}

let initActionTilt = () => {
    setTimeout(() => {
        tilt_motor_control_message = 'zero';
        setTimeout(() => {
            tilt_motor_control_message = 'tilt_up';
            setTimeout(() => {
                tilt_motor_control_message = 'tilt_down';
                setTimeout(() => {
                    tilt_motor_control_message = 'stop';
                }, 2000);
            }, 2000);
        }, 1000);
    }, 500);
}

let float_to_uint = (x, x_min, x_max, bits) => {
    let span = x_max - x_min;
    let offset = x_min;
    let pgg = 0;
    if (bits === 12) {
        pgg = (x - offset) * 4095.0 / span;
    }
    else if (bits === 16) {
        pgg = (x - offset) * 65535.0 / span;
    }

    return parseInt(pgg);
}

let uint_to_float = (x_int, x_min, x_max, bits) => {
    let span = x_max - x_min;
    let offset = x_min;
    let pgg = 0;
    if (bits === 12) {
        pgg = parseFloat(x_int) * span / 4095.0 + offset;
    }
    else if (bits === 16) {
        pgg = parseFloat(x_int) * span / 65535.0 + offset;
    }

    return parseFloat(pgg);
}

function pack_cmd(CAN_ID, p_in, v_in, kp_in, kd_in, t_in) {
    let p_des = constrain(p_in, P_MIN, P_MAX);
    let v_des = constrain(v_in, V_MIN, V_MAX);
    let kp = constrain(kp_in, KP_MIN, KP_MAX);
    let kd = constrain(kd_in, KD_MIN, KD_MAX);
    let t_ff = constrain(t_in, T_MIN, T_MAX);

    let p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    let v_int = float_to_uint(v_des, P_MIN, P_MAX, 12);
    let kp_int = float_to_uint(kp, P_MIN, P_MAX, 12);
    let kd_int = float_to_uint(kd, P_MIN, P_MAX, 12);
    let t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    let p_int_hex = p_int.toString(16).padStart(4, '0');
    let v_int_hex = v_int.toString(16).padStart(3, '0');
    let kp_int_hex = kp_int.toString(16).padStart(3, '0');
    let kd_int_hex = kd_int.toString(16).padStart(3, '0');
    let t_int_hex = t_int.toString(16).padStart(3, '0');

    let msg_buf = CAN_ID + p_int_hex + v_int_hex + kp_int_hex + kd_int_hex + t_int_hex;
    //console.log('Can Port Send Data ===> ' + msg_buf);

    can_port.write(Buffer.from(msg_buf, 'hex'), () => {
        // console.log('can write =>', msg_buf);
    });
}

let unpack_reply = () => {
    let id = parseInt(motor_return_msg.substring(9, 10), 16);
    let p_int = parseInt(motor_return_msg.substring(10, 14), 16);
    let v_int = parseInt(motor_return_msg.substring(14, 17), 16);
    let i_int = parseInt(motor_return_msg.substring(17, 20), 16);
    if (id === 1) {
        pan_p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
        pan_v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
        pan_t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
    } else if (id === 2) {
        tilt_p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
        tilt_v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
        tilt_t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
    }
}

//--------------- CAN special message ---------------
function EnterMotorMode(CAN_ID) {
    can_port.write(Buffer.from(CAN_ID + 'FFFFFFFFFFFFFFFC', 'hex'));
}

function ExitMotorMode(CAN_ID) {
    can_port.write(Buffer.from(CAN_ID + 'FFFFFFFFFFFFFFFD', 'hex'));
}

function Zero(CAN_ID) {
    can_port.write(Buffer.from(CAN_ID + 'FFFFFFFFFFFFFFFE', 'hex'));
}
//---------------------------------------------------

function calcTargetPanAngle(targetLatitude, targetLongitude) {
    // console.log('[pan] myLatitude, myLongitude, myRelativeAltitude: ', myLatitude, myLongitude, myRelativeAltitude);
    // console.log('[pan] targetLatitude, targetLongitude: ', targetLatitude, targetLongitude);

    let radmyLatitude = myLatitude * Math.PI / 180;
    let radTargetLatitude = targetLatitude * Math.PI / 180;
    let radMyLongitude = myLongitude * Math.PI / 180;
    let radTargetLongitude = targetLongitude * Math.PI / 180;

    let y = Math.sin(radTargetLongitude - radMyLongitude) * Math.cos(radTargetLatitude);
    let x = Math.cos(radmyLatitude) * Math.sin(radTargetLatitude) - Math.sin(radmyLatitude) * Math.cos(radTargetLatitude) * Math.cos(radTargetLongitude - radMyLongitude);
    let θ = Math.atan2(y, x); // azimuth angle (radians)

    turn_angle = (θ * 180 / Math.PI + 360) % 360; // azimuth angle (convert to degree)

    let trun_pan_angle = turn_angle - myHeading;
    // if (myHeading > 180) {
    //     trun_pan_angle = turn_angle + (myHeading - 180)
    // } else {
    //     trun_pan_angle = turn_angle - myHeading
    // }
    console.log('turn_angle: ', turn_angle);
    console.log('trun_pan_angle: ', trun_pan_angle);
    console.log('---------------------------');
    if (run_flag === 'reset') {
        run_flag = 'go';
        pan_motor_control_message = 'run';
    }
    else if (run_flag === 'go') {
        if (parseInt(Math.abs(cur_angle)) === 360 || parseInt(Math.abs(cur_angle)) === 720) {
            pan_motor_control_message = 'zero';
            cur_angle = 0;
            run_flag = 'reset';
        }

        if (trun_pan_angle < 0) {
            temp_angle = trun_pan_angle + 360;
        }
        else {
            temp_angle = trun_pan_angle;
        }

        if (temp_angle - cur_angle < 0) {
            cw = 360 - cur_angle + temp_angle;
            ccw = (360 - cw) * (-1);
        }
        else {
            if (temp_angle - cur_angle >= 360) {
                cw = temp_angle - cur_angle - 360;
                ccw = (360 - cw) * (-1);
            }
            else {
                cw = temp_angle - cur_angle;
                ccw = (360 - cw) * (-1);
            }
        }

        if (Math.abs(cw) <= Math.abs(ccw)) {
            pan_p_target = (cur_angle + cw) * 0.0174533 + p_offset;
        }
        else {
            pan_p_target = (cur_angle + ccw) * 0.0174533 + p_offset;
        }
        cur_angle = (pan_p_target - p_offset) * 180 / Math.PI;
    }
}

function calcTargetTiltAngle(targetLatitude, targetLongitude, targetAltitude) {
    // console.log('[tilt] myLatitude, myLongitude, myRelativeAltitude: ', myLatitude, myLongitude, myRelativeAltitude);
    // console.log('[tilt] targetLatitude, targetLongitude, targetAltitude: ', targetLatitude, targetLongitude, targetAltitude);

    let dist = getDistance(myLatitude, myLongitude, targetLatitude, targetLongitude)
    let x = dist;
    let y = targetAltitude - myRelativeAltitude;

    let θ = Math.atan2(y, x);

    // console.log('x, y, θ: ', x, y, θ * 180 / Math.PI);
    tilt_p_target = θ + p_offset;
}

function getDistance(lat1, lon1, lat2, lon2) {
    if ((lat1 == lat2) && (lon1 == lon2))
        return 0;

    var radLat1 = Math.PI * lat1 / 180;
    var radLat2 = Math.PI * lat2 / 180;
    var theta = lon1 - lon2;
    var radTheta = Math.PI * theta / 180;
    var dist = Math.sin(radLat1) * Math.sin(radLat2) + Math.cos(radLat1) * Math.cos(radLat2) * Math.cos(radTheta);
    if (dist > 1)
        dist = 1;

    dist = Math.acos(dist);
    dist = dist * 180 / Math.PI;
    dist = dist * 60 * 1.1515 * 1.609344 * 1000;

    return dist;
}


//------------- sitl mqtt connect ------------------
function sitlMqttConnect(host) {
    let connectOptions = {
        host: host,
        port: 1883,
        protocol: "mqtt",
        keepalive: 10,
        clientId: 'sitl_' + nanoid(15),
        protocolId: "MQTT",
        protocolVersion: 4,
        clean: true,
        reconnectPeriod: 2000,
        connectTimeout: 2000,
        rejectUnauthorized: false
    }

    sitlmqtt = mqtt.connect(connectOptions);

    sitlmqtt.on('connect', function () {
        sitlmqtt.subscribe(sub_sitl_drone_data_topic + '/#', () => {
            console.log('[sitl] sitlmqtt subscribed -> ', sub_sitl_drone_data_topic);
        });
    });

    sitlmqtt.on('message', function (topic, message) {
        // console.log('[sitl] topic, message => ', topic, message);

        if (topic.includes(sub_sitl_drone_data_topic)) {
            sitlmqtt_message = message.toString('hex');
            // console.log("Client1 topic => " + topic);
            // console.log("Client1 message => " + sitlmqtt_message);

            try {
                let ver = sitlmqtt_message.substring(0, 2);
                let sysid = '';
                let msgid = '';
                let base_offset = 0;

                if (ver == 'fd') {//MAV ver.1
                    sysid = sitlmqtt_message.substring(10, 12).toLowerCase();
                    msgid = sitlmqtt_message.substring(18, 20) + sitlmqtt_message.substring(16, 18) + sitlmqtt_message.substring(14, 16);
                    base_offset = 28;
                } else { //MAV ver.2
                    sysid = sitlmqtt_message.substring(6, 8).toLowerCase();
                    msgid = sitlmqtt_message.substring(10, 12).toLowerCase();
                    base_offset = 20;
                }

                let sys_id = parseInt(sysid, 16);
                let msg_id = parseInt(msgid, 16);

                if (msg_id === 33) { // MAVLINK_MSG_ID_GLOBAL_POSITION_INT
                    let lat = sitlmqtt_message.substring(base_offset, base_offset + 8).toLowerCase().toString();
                    base_offset += 8;
                    let lon = sitlmqtt_message.substring(base_offset, base_offset + 8).toLowerCase();
                    base_offset += 8;
                    let alt = sitlmqtt_message.substring(base_offset, base_offset + 8).toLowerCase();
                    base_offset += 8;
                    let relative_alt = sitlmqtt_message.substring(base_offset, base_offset + 8).toLowerCase();

                    target_latitude = Buffer.from(lat, 'hex').readInt32LE(0).toString() / 10000000;
                    target_longitude = Buffer.from(lon, 'hex').readInt32LE(0).toString() / 10000000;
                    target_altitude = Buffer.from(alt, 'hex').readInt32LE(0).toString() / 1000;
                    target_relative_altitude = Buffer.from(relative_alt, 'hex').readInt32LE(0).toString() / 1000;

                    calcTargetPanAngle(target_latitude, target_longitude);
                    calcTargetTiltAngle(target_latitude, target_longitude, target_relative_altitude);
                    // console.log('target_latitude, target_longitude, target_altitude, target_relative_altitude', target_latitude, target_longitude, target_altitude, target_relative_altitude);
                }
            }
            catch (e) {
                console.log('[sitl] sitl mqtt connect Error', e);
            }
        }
    });

    sitlmqtt.on('error', function (err) {
        console.log('[sitl] sitl mqtt connect error ' + err.message);
        sitlmqtt = null;
        setTimeout(sitlMqttConnect, 1000, sitl_mqtt_host);
    });
}
//---------------------------------------------------
