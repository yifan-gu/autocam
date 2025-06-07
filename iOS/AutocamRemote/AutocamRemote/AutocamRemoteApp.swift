import SwiftUI
import UIKit

/// 1) Your existing subclass:
class HiddenHomeIndicatorHostingController<Content: View>: UIHostingController<Content> {
  override var prefersHomeIndicatorAutoHidden: Bool { true }
}

/// 2) A wrapper that SwiftUI can instantiate for you:
struct HostingControllerWrapper<Content: View>: UIViewControllerRepresentable {
  let rootView: Content

  func makeUIViewController(context: Context) -> HiddenHomeIndicatorHostingController<Content> {
    HiddenHomeIndicatorHostingController(rootView: rootView)
  }

  func updateUIViewController(_ uiViewController: HiddenHomeIndicatorHostingController<Content>,
                              context: Context) {
    uiViewController.rootView = rootView
  }
}

@main
struct AutocamRemoteApp: App {
  @StateObject private var blePeripheral = BLEPeripheralManager()

  var body: some Scene {
    WindowGroup {
      // 3) Use the wrapper instead of ContentView() directly
      HostingControllerWrapper(
        rootView: ContentView()
          .environmentObject(blePeripheral)
      )
      .ignoresSafeArea() // if you want your content to go edge‐to‐edge
    }
  }
}
