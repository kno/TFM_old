
"use strict";

let MotorPWM = require('./MotorPWM.js');
let RawMagnetic = require('./RawMagnetic.js');
let RawRC = require('./RawRC.js');
let RuddersCommand = require('./RuddersCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let ThrustCommand = require('./ThrustCommand.js');
let Compass = require('./Compass.js');
let HeightCommand = require('./HeightCommand.js');
let YawrateCommand = require('./YawrateCommand.js');
let RC = require('./RC.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let Supply = require('./Supply.js');
let MotorStatus = require('./MotorStatus.js');
let ServoCommand = require('./ServoCommand.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let Altimeter = require('./Altimeter.js');
let MotorCommand = require('./MotorCommand.js');
let Altitude = require('./Altitude.js');
let ControllerState = require('./ControllerState.js');
let RawImu = require('./RawImu.js');

module.exports = {
  MotorPWM: MotorPWM,
  RawMagnetic: RawMagnetic,
  RawRC: RawRC,
  RuddersCommand: RuddersCommand,
  VelocityXYCommand: VelocityXYCommand,
  VelocityZCommand: VelocityZCommand,
  ThrustCommand: ThrustCommand,
  Compass: Compass,
  HeightCommand: HeightCommand,
  YawrateCommand: YawrateCommand,
  RC: RC,
  AttitudeCommand: AttitudeCommand,
  Supply: Supply,
  MotorStatus: MotorStatus,
  ServoCommand: ServoCommand,
  PositionXYCommand: PositionXYCommand,
  HeadingCommand: HeadingCommand,
  Altimeter: Altimeter,
  MotorCommand: MotorCommand,
  Altitude: Altitude,
  ControllerState: ControllerState,
  RawImu: RawImu,
};
