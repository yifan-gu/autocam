import SwiftUI

@main
struct AutocamRemoteApp: App {
  // 1) Create exactly one peripheral manager for the lifetime of the app
  @StateObject private var blePeripheral = BLEPeripheralManager()

  var body: some Scene {
    WindowGroup {
      ContentView()
        .environmentObject(blePeripheral)
    }
  }
}
