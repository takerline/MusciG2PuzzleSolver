
"use strict";

let Popup = require('./Popup.js')
let RawRequest = require('./RawRequest.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let GetProgramState = require('./GetProgramState.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let AddToLog = require('./AddToLog.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Load = require('./Load.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetSafetyMode = require('./GetSafetyMode.js')

module.exports = {
  Popup: Popup,
  RawRequest: RawRequest,
  IsProgramRunning: IsProgramRunning,
  GetProgramState: GetProgramState,
  GetLoadedProgram: GetLoadedProgram,
  AddToLog: AddToLog,
  IsProgramSaved: IsProgramSaved,
  IsInRemoteControl: IsInRemoteControl,
  Load: Load,
  GetRobotMode: GetRobotMode,
  GetSafetyMode: GetSafetyMode,
};
