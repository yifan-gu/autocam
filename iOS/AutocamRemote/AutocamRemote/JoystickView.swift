import SwiftUI

/// A circular joystick that reports normalized x/y in [–1, +1].
/// - `size`: overall diameter of the joystick area.
/// - `knobSize`: diameter of the draggable knob.
/// - `is2D`: if true, the knob moves in both X/Y; if false, one axis is locked depending on which binding is constant.
struct JoystickView: View {
    @Binding var xValue: CGFloat    // normalized −1…+1
    @Binding var yValue: CGFloat    // normalized −1…+1
    let size: CGFloat
    let knobSize: CGFloat
    let is2D: Bool

    // Actual visual offset of the knob
    @State private var dragOffset: CGSize = .zero

    var body: some View {
        ZStack {
            // 1) Background “track” circle
            Circle()
                .stroke(lineWidth: 2)
                .foregroundColor(Color.gray.opacity(0.5))
                .frame(width: size, height: size)

            // 2) Draggable knob (color #FFAC1C)
            Circle()
                .fill(Color(red: 255/255, green: 172/255, blue: 28/255))
                .frame(width: knobSize, height: knobSize)
                .offset(dragOffset)
                .gesture(
                    DragGesture()
                        .onChanged { value in
                            // ---------------------------------------------------
                            // STEP A: Compute “raw” translation from gesture’s start
                            // ---------------------------------------------------
                            var rawX = value.translation.width / 3
                            var rawY = value.translation.height / 3

                            // If not 2D, lock one axis if its binding is constant:
                            if !is2D {
                                if xValueBindingIsConstant { rawX = 0 }
                                if yValueBindingIsConstant { rawY = 0 }
                            }

                            // ---------------------------------------------------
                            // STEP B: Clamp rawX/rawY so the knob never leaves the circle
                            // ---------------------------------------------------
                            let half = size / 2
                            let maxRadius = half - (knobSize / 2)
                            let dist = sqrt(rawX * rawX + rawY * rawY)
                            if dist > maxRadius {
                                let scale = maxRadius / dist
                                rawX *= scale
                                rawY *= scale
                            }

                            // ---------------------------------------------------
                            // STEP C: Compute a linear “rawNorm” (–1…+1) for output
                            // ---------------------------------------------------
                            let rawNormX = min(1, max(-1, rawX / maxRadius))
                            let rawNormY = min(1, max(-1, rawY / maxRadius))

                            // ---------------------------------------------------
                            // STEP D: Apply a cubic curve to rawNorm → “displayNorm”
                            //         This makes the knob “resist” small drags:
                            //             displayNorm = (rawNorm)³
                            // ---------------------------------------------------
                            let displayNormX = rawNormX
                            let displayNormY = rawNormY

                            // ---------------------------------------------------
                            // STEP E: Update visual offset in points:
                            //         displayNorm * maxRadius = how far the knob actually moves.
                            // ---------------------------------------------------
                            dragOffset = CGSize(
                                width: displayNormX * maxRadius,
                                height: displayNormY * maxRadius
                            )

                            // ---------------------------------------------------
                            // STEP F: Update the linear output bindings (user sees ±1 linearly).
                            //         Note: invert Y so up → +1.
                            // ---------------------------------------------------
                            xValue = rawNormX
                            yValue = -rawNormY
                        }
                        .onEnded { _ in
                            // ---------------------------------------------------
                            // On release, animate the knob back to center with a stiff, well-damped spring.
                            // ---------------------------------------------------
                            withAnimation(.interpolatingSpring(stiffness: 200, damping: 25)) {
                                dragOffset = .zero
                                xValue = 0
                                yValue = 0
                            }
                        }
                )
        }
        // Force the view’s size to be exactly “size”.
        .frame(width: size, height: size)
    }

    // Helpers to detect if a Binding was provided as a Constant(...).
    // If the user wrote “yValue: .constant(0)”, then yValueBindingIsConstant is true.
    private var xValueBindingIsConstant: Bool {
        Mirror(reflecting: _xValue).children.first?.label == "constant"
    }
    private var yValueBindingIsConstant: Bool {
        Mirror(reflecting: _yValue).children.first?.label == "constant"
    }
}
