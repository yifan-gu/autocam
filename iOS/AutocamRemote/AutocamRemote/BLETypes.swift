import Foundation
import CoreBluetooth

// These must exactly match your ESP32’s AutocamRemoteService/Characteristics:
let AutocamRemoteServiceUUID  = CBUUID(string: "f49f531a-9cba-4ada-905c-68699d400122")
// “B330” → 16-bit → 0000B330-0000-1000-8000-00805F9B34FB
let AutocamSendCharUUID       = CBUUID(string: "B330")
// “B331” → 16-bit → 0000B331-0000-1000-8000-00805F9B34FB
let AutocamRecvCharUUID       = CBUUID(string: "B331")

/// Exactly mirror your ESP32’s RemoteDataSend layout:
///   uint16_t throttleValue;
///   uint16_t steeringValue;
///   float    yawSpeed;
///   float    pitchSpeed;
///   uint8_t  driveMode;
///   uint8_t  toggleState;
///   uint8_t uwbSelector;
/// Total = 2 + 2 + 1 + 4 + 4 + 1 + 1 + 1 = 16 bytes, little-endian.
struct RemoteDataSend {
  var throttleValue: Int16   // 1000…2000
  var steeringValue: Int16   // 1000…2000
  var yawSpeed: Float32      // –4…+4
  var pitchSpeed: Float32    // –4…+4
  var driveMode: UInt8       // 0=manual,1=follow,2=cinema
  var toggleState: UInt8     // multi-click code
  var uwbSelector: UInt8     // 0 or 1
  var padding: UInt8         // To make it align with the arduino packet size.
}

extension RemoteDataSend {
  /// Pack fields (little-endian) into a 16-byte Data blob.
  func toData() -> Data {
    var t   = throttleValue.littleEndian
    var s   = steeringValue.littleEndian
    var y   = yawSpeed.bitPattern.littleEndian
    var p   = pitchSpeed.bitPattern.littleEndian
    var d   = driveMode
    var tgl = toggleState
    var u   = uwbSelector.littleEndian
    var pa  = padding.littleEndian

    return Data(bytes: &t,   count: MemoryLayout.size(ofValue: t))
         + Data(bytes: &s,   count: MemoryLayout.size(ofValue: s))
         + Data(bytes: &y,   count: MemoryLayout.size(ofValue: y))
         + Data(bytes: &p,   count: MemoryLayout.size(ofValue: p))
         + Data(bytes: &d,   count: MemoryLayout.size(ofValue: d))
         + Data(bytes: &tgl, count: MemoryLayout.size(ofValue: tgl))
         + Data(bytes: &u,   count: MemoryLayout.size(ofValue: u))
         + Data(bytes: &pa,  count: MemoryLayout.size(ofValue: pa))
  }
}

/// Mirror the ESP32’s RemoteDataRecv struct on the peripheral side:
///   uint8_t state;
///   uint8_t driveMode;
///   uint8_t toggleState;
///   uint8_t uwbSelector;
/// Total = 1 + 1 + 1 + 1 = 4 bytes, little-endian.
struct RemoteDataRecv {
  var state: UInt8
  var driveMode: UInt8
  var toggleState: UInt8
  var uwbSelector: UInt8
}
