import SwiftUI
import UIKit  // Needed for haptic feedback

struct ContentView: View {
    @EnvironmentObject var blePeripheral: BLEPeripheralManager

    // Normalized joystick outputs (–1…+1)
    @State private var steeringNorm: CGFloat = 0    // left joystick X
    @State private var throttleNorm: CGFloat = 0    // right joystick Y
    @State private var yawNorm: CGFloat = 0         // center joystick X
    @State private var pitchNorm: CGFloat = 0       // center joystick Y

    // Other UI state
    @State private var driveModeTrigger: Int = 0          // 0=Manual, 1=Follow, 2=Cinema
    @State private var uwbSelectorTrigger: Int = 0
    @State private var toggleState: Int = 0

    // Placeholder for sensor connectivity.
    private var sensorReady: Bool {
        return blePeripheral.sensorReady
    }
    
    private var remoteReady: Bool {
        return blePeripheral.remoteReady
    }
    
    private var uwbSelector: Int {
        return blePeripheral.uwbSelectorReceived
    }
    
    private var isNotReady: Bool {
        return !blePeripheral.sensorReady || !blePeripheral.remoteReady
    }

    // Compute a one‐letter label for the current driveMode
    private var driveModeLetter: String {
        switch blePeripheral.receivedDriveMode {
        case 0: return "M"  // Manual
        case 1: return "F"  // Follow
        default: return "C" // Cinema
        }
    }

    // Custom color #FFAC1C
    private let accentColor = Color(red: 255/255, green: 172/255, blue: 28/255)

    var body: some View {
        VStack(spacing: 0) {
            // ┌──────────────────────────────────────────────────────────────────────────┐
            // │  Top area: Four LED+Label columns, with Connection status in the middle  │
            // └──────────────────────────────────────────────────────────────────────────┘
            HStack(spacing: 24) {
                // 1) Sensor column
                VStack(spacing: 4) {
                    Circle()
                        .fill(sensorReady ? Color.green : Color.red)
                        .frame(width: 24, height: 24)
                    Text("Sensor")
                        .font(.caption)
                        .foregroundColor(.white)
                }

                // 2) Remote column
                VStack(spacing: 4) {
                    Circle()
                        .fill(remoteReady ? Color.green : Color.red)
                        .frame(width: 24, height: 24)
                    Text("Remote")
                        .font(.caption)
                        .foregroundColor(.white)
                }

                // 3) Connection status (expands to fill center space)
                VStack {
                    if blePeripheral.centralIsSubscribed {
                        Text("🟢 Autocam Server Subscribed")
                            .font(.headline)
                            .lineLimit(1)
                            .minimumScaleFactor(0.5)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity, alignment: .center)
                    } else {
                        Text("⚪ Waiting for Autocam Server")
                            .font(.headline)
                            .lineLimit(1)
                            .minimumScaleFactor(0.5)
                            .foregroundColor(.white)
                            .frame(maxWidth: .infinity, alignment: .center)
                    }
                }
                .frame(maxWidth: .infinity)

                // 4) UWB column (blue circle with “1” or “0”)
                VStack(spacing: 4) {
                    ZStack {
                        Circle()
                            .fill(Color.blue)
                            .frame(width: 24, height: 24)
                        Text("\(uwbSelector)")
                            .font(.caption2)
                            .foregroundColor(.white)
                    }
                    Text("UWB")
                        .font(.caption)
                        .foregroundColor(.white)
                }

                // 5) Drive Mode column (blue circle with “M”, “F”, or “C”)
                VStack(spacing: 4) {
                    ZStack {
                        Circle()
                            .fill(Color.blue)
                            .frame(width: 24, height: 24)
                        Text(driveModeLetter)
                            .font(.caption2)
                            .foregroundColor(.white)
                    }
                    Text("Drive Mode")
                        .font(.caption)
                        .foregroundColor(.white)
                }
            }
            .padding(.top, 20)      // 20 points top padding
            .padding(.bottom, 8)    // small bottom inset

            Spacer()
            Spacer()

            // ┌──────────────────────────────────────────────┐
            // │  Next row: Gimbal Fn, UWB toggle, Drive Mode │
            // └──────────────────────────────────────────────┘
            VStack(alignment: .leading, spacing: 20) {
                HStack {
                    Spacer()

                    // Gimbal Fn on the LEFT
                    VStack(alignment: .center, spacing: 4) {
                        Text("Gimbal Fn")
                            .font(.subheadline)
                            .foregroundColor(.white)
                        Button("Tap") {
                            // Haptic feedback
                            let generator = UIImpactFeedbackGenerator(style: .medium)
                            generator.impactOccurred()
                            registerGimbalClick()
                        }
                        .buttonStyle(.bordered)
                        .tint(accentColor) // set text to #FFAC1C
                        .frame(width: 100, height: 40)
                    }

                    Spacer()

                    // UWB selector button
                    VStack(alignment: .center, spacing: 4) {
                        Text("UWB Selector")
                            .font(.subheadline)
                            .foregroundColor(.white)
                        Button("Tap") {
                            // Haptic feedback
                            let generator = UIImpactFeedbackGenerator(style: .medium)
                            generator.impactOccurred()
                            registerUwbSelectorClick()
                        }
                        .buttonStyle(.bordered)
                        .tint(accentColor) // set text to #FFAC1C
                        .frame(width: 100, height: 40)
                    }

                    Spacer()

                    // Drive Mode on the RIGHT (multi-click logic)
                    VStack(alignment: .center, spacing: 4) {
                        Text("Drive Mode")
                            .font(.subheadline)
                            .foregroundColor(.white)
                        Button("Tap") {
                            // Haptic feedback
                            let generator = UIImpactFeedbackGenerator(style: .medium)
                            generator.impactOccurred()
                            registerDriveModeClick()
                        }
                        .buttonStyle(.bordered)
                        .tint(accentColor) // set text to #FFAC1C
                        .frame(width: 100, height: 40)
                    }

                    Spacer()
                }
                .frame(maxWidth: .infinity)
            }
            .padding(.horizontal, 20)
            .padding(.top, 8)

            Spacer()

            // ┌──────────────────────────────┐
            // │  Bottom area: 3 Joysticks    │
            // └──────────────────────────────┘
            HStack {
                // Left joystick → steering
                JoystickView(
                    xValue: $steeringNorm,
                    yValue: .constant(0),
                    size: 120,
                    knobSize: 50,
                    is2D: true
                )
                .onChange(of: steeringNorm) { _ in
                    updateAutocamRemoteInputPacket()
                }

                Spacer()

                // Middle joystick → yaw/pitch
                JoystickView(
                    xValue: $yawNorm,
                    yValue: $pitchNorm,
                    size: 120,
                    knobSize: 50,
                    is2D: true
                )
                .onChange(of: yawNorm) { _ in updateAutocamRemoteInputPacket() }
                .onChange(of: pitchNorm) { _ in updateAutocamRemoteInputPacket() }

                Spacer()

                // Right joystick → throttle
                JoystickView(
                    xValue: .constant(0),
                    yValue: $throttleNorm,
                    size: 120,
                    knobSize: 50,
                    is2D: true
                )
                .onChange(of: throttleNorm) { _ in
                    updateAutocamRemoteInputPacket()
                }
            }
            .padding(.horizontal, 20)
            .padding(.bottom, 20)
        }
        .background(Color(red: 128/255, green: 0, blue: 128/255).ignoresSafeArea())
        .preferredColorScheme(.dark) // force dark appearance
        .onChange(of: isNotReady) { newValue in
          // newValue == true whenever *either* flag went false
          if newValue {
            print("🚨 Sensor or Remote dropped – resetting drive mode")
            driveModeTrigger = 0
            updateAutocamRemoteInputPacket()
          }
        }
    }

    // MARK: – Gimbal Multi-Click

    @State private var gimbalLastClickTime: Date = Date.distantPast
    @State private var gimbalClickCount: Int = 0
    private let gimbalClickTimeout: TimeInterval = 0.5

    private func registerGimbalClick() {
        let now = Date()
        if now.timeIntervalSince(gimbalLastClickTime) < gimbalClickTimeout {
            gimbalClickCount += 1
        } else {
            gimbalClickCount = 1
        }
        gimbalLastClickTime = now

        DispatchQueue.main.asyncAfter(deadline: .now() + gimbalClickTimeout) {
            if Date().timeIntervalSince(self.gimbalLastClickTime) >= self.gimbalClickTimeout {
                if self.gimbalClickCount == 1 {
                    self.toggleState = 1 // ACTIVE_TRACK_TOGGLED
                } else if self.gimbalClickCount == 2 {
                    self.toggleState = 2 // GIMBAL_RECENTER_TOGGLED
                } else {
                    self.toggleState = 3 // CAMERA_RECORDING_TOGGLED
                }
                self.updateAutocamRemoteInputPacket()
                self.gimbalClickCount = 0
                self.toggleState = 0
            }
        }
    }
    
    // MARK: – UWB Selector Multi-Click
    
    @State private var uwbSelectorLastClickTime: Date = Date.distantPast
    @State private var uwbSelectorClickCount: Int = 0
    private let uwbSelectorClickTimeout: TimeInterval = 0.5

    private func registerUwbSelectorClick() {
        let now = Date()
        if now.timeIntervalSince(uwbSelectorLastClickTime) < uwbSelectorClickTimeout {
            uwbSelectorClickCount += 1
        } else {
            uwbSelectorClickCount = 1
        }
        uwbSelectorLastClickTime = now

        DispatchQueue.main.asyncAfter(deadline: .now() + uwbSelectorClickTimeout) {
            if Date().timeIntervalSince(self.uwbSelectorLastClickTime) >= self.uwbSelectorClickTimeout {
                // 1 tap → 0, 2 taps → 1, 3+ taps → 2
                if self.uwbSelectorClickCount == 1 {
                    self.uwbSelectorTrigger = 0
                } else if self.uwbSelectorClickCount == 2 {
                    self.uwbSelectorTrigger = 1
                } else {
                    self.uwbSelectorTrigger = 2
                }
                self.updateAutocamRemoteInputPacket()
                self.uwbSelectorClickCount = 0
            }
        }
    }

    // MARK: – Drive Mode Multi-Click
    
    @State private var driveModeLastClickTime: Date = Date.distantPast
    @State private var driveModeClickCount: Int = 0
    private let driveModeClickTimeout: TimeInterval = 0.5

    private func registerDriveModeClick() {
        let now = Date()
        if now.timeIntervalSince(driveModeLastClickTime) < driveModeClickTimeout {
            driveModeClickCount += 1
        } else {
            driveModeClickCount = 1
        }
        driveModeLastClickTime = now

        DispatchQueue.main.asyncAfter(deadline: .now() + driveModeClickTimeout) {
            if Date().timeIntervalSince(self.driveModeLastClickTime) >= self.driveModeClickTimeout {
                // 1 tap → Manual, 2 taps → Follow, 3+ taps → Cinema
                if self.driveModeClickCount == 1 {
                    self.driveModeTrigger = 0 // MANUAL
                } else if self.driveModeClickCount == 2 {
                    self.driveModeTrigger = 1 // FOLLOW
                } else {
                    self.driveModeTrigger = 2 // CINEMA
                }
                self.updateAutocamRemoteInputPacket()
                self.driveModeClickCount = 0
            }
        }
    }

    // MARK: – Helpers for mapping & sending

    private func steeringNormToValue() -> Double {
        let mid: Double = 1500
        let range: Double = 500
        // Invert so “left” gives higher steering
        return min(max(mid + Double(steeringNorm) * range * -1, 1000), 2000)
    }

    private func throttleNormToValue() -> Double {
        let mid: Double = 1500
        let range: Double = 500
        return min(max(mid + Double(throttleNorm) * range, 1000), 2000)
    }

    private func updateAutocamRemoteInputPacket() {
        let tVal = Int(throttleNormToValue())
        let sVal = Int(steeringNormToValue())
        let yVal = Float(yawNorm * 4.0)    // map from –1…+1 to –4…+4
        let pVal = Float(pitchNorm * -4.0)  // map from –1…+1 to +4…-4

        // build packet
        let packet = RemoteDataSend(
            throttleValue: Int16(tVal),
            steeringValue: Int16(sVal),
            yawSpeed:       Float32(yVal),
            pitchSpeed:     Float32(pVal),
            driveMode:      UInt8(driveModeTrigger),
            toggleState:    UInt8(toggleState),
            uwbSelector:    UInt8(uwbSelectorTrigger),
            padding: 0,
        )

        // **only** update the packet—no BLE send here
        blePeripheral.updateSendPacket(packet)
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
            .environmentObject(BLEPeripheralManager())
    }
}
