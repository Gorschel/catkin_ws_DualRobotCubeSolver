
"use strict";

let RestartController = require('./RestartController.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let StopController = require('./StopController.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetSpeed = require('./SetSpeed.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let StartController = require('./StartController.js')

module.exports = {
  RestartController: RestartController,
  SetComplianceMargin: SetComplianceMargin,
  SetCompliancePunch: SetCompliancePunch,
  SetTorqueLimit: SetTorqueLimit,
  StopController: StopController,
  TorqueEnable: TorqueEnable,
  SetSpeed: SetSpeed,
  SetComplianceSlope: SetComplianceSlope,
  StartController: StartController,
};
