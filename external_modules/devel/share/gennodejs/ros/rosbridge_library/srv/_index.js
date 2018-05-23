
"use strict";

let TestEmpty = require('./TestEmpty.js')
let TestNestedService = require('./TestNestedService.js')
let TestResponseOnly = require('./TestResponseOnly.js')
let TestMultipleRequestFields = require('./TestMultipleRequestFields.js')
let TestMultipleResponseFields = require('./TestMultipleResponseFields.js')
let TestRequestAndResponse = require('./TestRequestAndResponse.js')
let TestRequestOnly = require('./TestRequestOnly.js')
let AddTwoInts = require('./AddTwoInts.js')
let TestArrayRequest = require('./TestArrayRequest.js')
let SendBytes = require('./SendBytes.js')

module.exports = {
  TestEmpty: TestEmpty,
  TestNestedService: TestNestedService,
  TestResponseOnly: TestResponseOnly,
  TestMultipleRequestFields: TestMultipleRequestFields,
  TestMultipleResponseFields: TestMultipleResponseFields,
  TestRequestAndResponse: TestRequestAndResponse,
  TestRequestOnly: TestRequestOnly,
  AddTwoInts: AddTwoInts,
  TestArrayRequest: TestArrayRequest,
  SendBytes: SendBytes,
};
