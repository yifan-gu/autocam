import Foundation
import CoreBluetooth
import Combine

final class BLEPeripheralManager: NSObject, ObservableObject {
    // MARK: – Published state for SwiftUI
    @Published var centralIsSubscribed = false
    
    // MARK: – Underlying CBPeripheralManager & GATT handles
    private var peripheralManager:     CBPeripheralManager!
    private var sendCharacteristic:    CBMutableCharacteristic!
    private var recvCharacteristic:    CBMutableCharacteristic!
    
    /// Store the latest “send” packet (so a future read or notify can return it).
    private var currentSendPacket = RemoteDataSend(
        throttleValue: 1500,
        steeringValue: 1500,
        yawSpeed: 0,
        pitchSpeed: 0,
        driveMode: 0,
        toggleState: 0,
        uwbSelector: 1
    )
    
    override init() {
        super.init()
        // Instantiate a CBPeripheralManager on the main queue.
        peripheralManager = CBPeripheralManager(delegate: self, queue: nil)
    }
    
    // MARK: – Public API for UI to update “send” data
    func updateSendPacket(_ packet: RemoteDataSend) {
        // Update our local copy:
        currentSendPacket = packet
        
        // 1) If a Central has subscribed and notifications are allowed, push the update:
        if centralIsSubscribed {
            let data = packet.toData()
            let didSend = peripheralManager.updateValue(
                data,
                for: sendCharacteristic,
                onSubscribedCentrals: nil // push to all subscribed centrals
            )
            if !didSend {
                // If it returns false, the transmit queue is full. You can buffer & retry in didSubscribe/didUnsubscribe.
                print("🔔 Failed to send notify (queue full). Will retry in didSubscribe/didReceiveRead)…")
            }
        }
    }
}

// MARK: – CBPeripheralManagerDelegate

extension BLEPeripheralManager: CBPeripheralManagerDelegate {
  func peripheralManagerDidUpdateState(_ peripheral: CBPeripheralManager) {
    switch peripheral.state {
    case .poweredOn:
      print("🟢 Peripheral Manager powered on. Setting up GATT…")
      setupServiceAndCharacteristics()
      startAdvertising()
    default:
      print("⚠️ Peripheral Manager state: \(peripheral.state.rawValue)")
    }
  }

  /// 1) Create two characteristics (Send: Read|Notify, Recv: Write)
  /// 2) Add them to a primary service
  /// 3) Publish that service
  private func setupServiceAndCharacteristics() {
    // 1) “Send” characteristic: Central can Read it and Subscribe for notifications
    sendCharacteristic = CBMutableCharacteristic(
      type: AutocamSendCharUUID,
      properties: [.read, .notify],
      value: nil,                              // Start with nil; we return currentSendPacket on read
      permissions: [.readable]
    )

    // 2) “Recv” characteristic: Central can Write to us
    recvCharacteristic = CBMutableCharacteristic(
      type: AutocamRecvCharUUID,
      properties: [.write],
      value: nil,                              // We don’t push initial value
      permissions: [.writeable]
    )

    // 3) Create the primary service
    let service = CBMutableService(
      type: AutocamRemoteServiceUUID,
      primary: true
    )
    service.characteristics = [sendCharacteristic, recvCharacteristic]

    // 4) Add the service to the peripheral manager
    peripheralManager.add(service)
    print("✅ GATT service added with characteristics.")
  }

  /// Begin advertising as “Autocam Remote” with our service UUID
  private func startAdvertising() {
    let advertisingData: [String: Any] = [
      CBAdvertisementDataLocalNameKey: "AutocamRemote",
      CBAdvertisementDataServiceUUIDsKey: [AutocamRemoteServiceUUID]
    ]
    peripheralManager.startAdvertising(advertisingData)
    print("🚀 Started advertising as AutocamRemote.")
  }

  /// Called when Central subscribes to the “Send” characteristic
  func peripheralManager(_ peripheral: CBPeripheralManager,
                         central: CBCentral,
                         didSubscribeTo characteristic: CBCharacteristic) {
    if characteristic.uuid == AutocamSendCharUUID {
      centralIsSubscribed = true
      print("🔔 Central subscribed for notifications on Send characteristic.")
    }
  }

  /// Called when Central unsubscribes
  func peripheralManager(_ peripheral: CBPeripheralManager,
                         central: CBCentral,
                         didUnsubscribeFrom characteristic: CBCharacteristic) {
    if characteristic.uuid == AutocamSendCharUUID {
      centralIsSubscribed = false
      print("🔕 Central unsubscribed from Send characteristic.")
    }
  }

  /// Handle read requests on the “Send” characteristic: return currentSendPacket
  func peripheralManager(_ peripheral: CBPeripheralManager,
                         didReceiveRead request: CBATTRequest) {
    guard request.characteristic.uuid == AutocamSendCharUUID else {
      peripheral.respond(to: request, withResult: .requestNotSupported)
      return
    }

    let data = currentSendPacket.toData()
    if request.offset > data.count {
      peripheral.respond(to: request, withResult: .invalidOffset)
      return
    }

    // Copy the requested chunk
    request.value = data.subdata(in: request.offset..<data.count)
    peripheral.respond(to: request, withResult: .success)
    print("📖 Sent currentSendPacket via Read request.")
  }

  /// Handle write requests on the “Recv” characteristic: parse Incoming data
  func peripheralManager(_ peripheral: CBPeripheralManager,
                         didReceiveWrite requests: [CBATTRequest]) {
    for req in requests {
      guard req.characteristic.uuid == AutocamRecvCharUUID,
            let data = req.value
      else {
        peripheral.respond(to: req, withResult: .requestNotSupported)
        continue
      }

      // Expect 16 bytes: { state, driveMode, toggleState, uwbSelector } as Int32 each
      if data.count == MemoryLayout<Int32>.size * 4 {
        var offset = 0
        func readInt32() -> Int32 {
          let slice = data.subdata(in: offset..<(offset+4))
          offset += 4
          return Int32(littleEndian: slice.withUnsafeBytes { $0.load(as: Int32.self) })
        }
        let stateVal     = readInt32()
        let driveModeVal = readInt32()
        let toggleVal    = readInt32()
        let uwbVal       = readInt32()

        print("✍️ Received RemoteDataRecv— state=\(stateVal), driveMode=\(driveModeVal), toggle=\(toggleVal), uwb=\(uwbVal)")

        // Update your @Published properties or post a Notification so SwiftUI can react:
        DispatchQueue.main.async {
          // e.g. update some @Published vars in your BLEPeripheralManager, or call a callback.
          // For simplicity, we’ll just print for now. In a real app, you’d keep these values in @Published properties.
        }

        peripheral.respond(to: req, withResult: .success)
      } else {
        peripheral.respond(to: req, withResult: .invalidAttributeValueLength)
      }
    }
  }

  /// If updateValue(…) fails (queue full), this is called when space frees up.
  func peripheralManagerIsReady(toUpdateSubscribers peripheral: CBPeripheralManager) {
    // You could retry any pending notifications here.
    print("🔄 Peripheral manager is ready to send pending notifications.")
  }
}
