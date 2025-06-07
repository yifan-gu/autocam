import Foundation
import CoreBluetooth
import Combine

// You’ll need these bit-masks to interpret `state`:
let SERVER_STATE_SENSOR_READY: UInt8 = 1 << 0
let SERVER_STATE_REMOTE_READY: UInt8 = 1 << 1

final class BLEPeripheralManager: NSObject, ObservableObject {
    // MARK: – Published state for SwiftUI LEDs
    @Published var centralIsSubscribed  = false
    @Published var sensorReady          = false
    @Published var remoteReady          = false
    @Published var receivedDriveMode    = 0
    @Published var uwbSelectorReceived  = 0

    // MARK: – CoreBluetooth
    private var peripheralManager:  CBPeripheralManager!
    private var sendCharacteristic: CBMutableCharacteristic!
    private var recvCharacteristic: CBMutableCharacteristic!

    // MARK: – Send buffering + loop
    private var currentSendPacket = RemoteDataSend(
        throttleValue: 1500,
        steeringValue: 1500,
        yawSpeed: 0,
        pitchSpeed: 0,
        driveMode: 0,
        toggleState: 0,
        uwbSelector: 1,
        padding: 0
    )
    private var lastRecvPacket: RemoteDataRecv?
    private var loopCancellable: AnyCancellable?
    private let loopInterval      = 1.0 / 50.0    // 50 Hz
    private let heartbeatInterval = 0.2           // 200 ms
    private var lastSendTime      = Date.distantPast
    private var inputDirty = false

    override init() {
        super.init()
        peripheralManager = CBPeripheralManager(delegate: self, queue: DispatchQueue.main)
    }

    // MARK: – Public API
    /// Call this from your SwiftUI whenever you want to change the outgoing packet.
    func updateSendPacket(_ pkt: RemoteDataSend) {
        currentSendPacket = pkt
        inputDirty = true
        // no immediate send — our loop will pick it up
    }

    // MARK: – Loop startup
    private func startMainLoop() {
        guard loopCancellable == nil else { return }
        loopCancellable = Timer.publish(every: loopInterval, on: .main, in: .common)
            .autoconnect()
            .sink { [weak self] _ in
                self?.tick()
            }
    }

    private func tick() {
        // 1) send input every 20ms (50hz) if there's a new input, or every 200ms for heartbeat.
        sendIfNeeded()

        // 3) push any newly‐received status into @Published
        updateUIFromRemoteDataRecv()
    }

    private func sendIfNeeded() {
        guard centralIsSubscribed else { return }

        let now = Date()
        let interval = now.timeIntervalSince(lastSendTime)

        // send immediately if UI changed, or as a heartbeat every 200 ms
        if inputDirty || interval >= heartbeatInterval {
            let data = currentSendPacket.toData()
            if peripheralManager.updateValue(data,
                                             for: sendCharacteristic,
                                             onSubscribedCentrals: nil) {
                lastSendTime = now
                inputDirty   = false
                currentSendPacket.toggleState = 0
            }
        }
    }

    private func sendHeartbeatIfNeeded() {
        let now = Date()
        guard centralIsSubscribed,
              now.timeIntervalSince(lastSendTime) >= heartbeatInterval
        else { return }
        

        let data = currentSendPacket.toData()
        if peripheralManager.updateValue(data, for: sendCharacteristic, onSubscribedCentrals: nil) {
            lastSendTime = now
        }
    }

    private func updateUIFromRemoteDataRecv() {
        guard let recv = lastRecvPacket else { return }
        // map bits → published LEDs
        sensorReady         = (recv.state & SERVER_STATE_SENSOR_READY) != 0
        remoteReady         = (recv.state & SERVER_STATE_REMOTE_READY) != 0
        receivedDriveMode   = Int(recv.driveMode)
        uwbSelectorReceived = Int(recv.uwbSelector)
    }
}


// MARK: – CBPeripheralManagerDelegate

extension BLEPeripheralManager: CBPeripheralManagerDelegate {
  func peripheralManagerDidUpdateState(_ peripheral: CBPeripheralManager) {
    switch peripheral.state {
    case .poweredOn:
      print("🟢 Peripheral Manager powered on. Setting up GATT and starting the main loop…")
      setupServiceAndCharacteristics()
      startAdvertising()
      startMainLoop()
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

      // Expect 16 bytes: { state, driveMode, toggleState, uwbSelector } as UInt8 each
      if data.count == MemoryLayout<UInt8>.size * 4 {
        var offset = 0
        func readUInt8() -> UInt8 {
          let slice = data.subdata(in: offset..<(offset+1))
          offset += 1
          return UInt8(littleEndian: slice.withUnsafeBytes { $0.load(as: UInt8.self) })
        }
        let stateVal     = readUInt8()
        let driveModeVal = readUInt8()
        let uwbVal       = readUInt8()
        let padding      = readUInt8()

        print("✍️ Received RemoteDataRecv— state=\(stateVal), driveMode=\(driveModeVal), uwb=\(uwbVal)")
          
        // Package it and store it so our next tick() can publish it
        let recv = RemoteDataRecv(
            state:       UInt8(stateVal),
            driveMode:   UInt8(driveModeVal),
            uwbSelector: UInt8(uwbVal),
            padding:     UInt8(padding)
        )
        // still on main queue, so:
        lastRecvPacket = recv

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
