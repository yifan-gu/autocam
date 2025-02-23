char const* index_html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Autocam Controller</title>
    <style>
      html,
      body {
        height: 100%;
        margin: 0;
        padding: 0;
        overflow: hidden;
        user-select: none; /* Disable text selection */
      }

      body {
        display: flex;
        justify-content: center;
        align-items: center;
        background-color: #2c3e50;
        position: relative;
        touch-action: none; /* Prevent default pinch-zoom on the webpage */
      }
      * {
        -webkit-tap-highlight-color: transparent; /* Disable blue highlight for all elements */
        user-select: none; /* Disable text selection */
        -webkit-user-select: none; /* Safari support */
        -ms-user-select: none; /* IE/Edge support */
      }

      button:disabled,
      input[type="range"]:disabled {
        outline: none;
      }
      button {
        touch-action: manipulation; /* Improve touch behavior */
      }
      canvas {
        position: absolute;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        touch-action: none; /* Prevent default pinch-zoom on the canvas */
      }

      .controls {
        display: none; /* Hide controls */
      }

      .location-panel {
        position: absolute;
        top: 0px;
        left: 0px;
        background-color: rgba(44, 62, 80, 0.1);
        color: white;
        padding: 10px 10px;
        border-radius: 8px;
        font-family: "Roboto Mono", monospace;
        font-size: 10px;
        box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
        display: flex;
        flex-direction: column;
        width: 86px;
      }

      .location-panel .label {
        margin-bottom: 0px;
        text-align: left; /* Ensure the label aligns to the left */
      }

      .location-panel .location-value {
        display: flex;
        justify-content: space-between; /* Align text within the row */
        align-items: center;
        width: 100%; /* Ensure the line spans the full width */
        white-space: nowrap; /* Prevent line breaks */
        margin-bottom: 5px; /* Add space between rows */
      }

      .location-panel .location-value .coord {
        flex: 1; /* Take up available space */
        text-align: right; /* Align numeric values to the right */
        margin-right: 2px;
      }

      .current-location {
        color: #3498db;
      }
      .toggle-container {
        margin: 5px auto;
        width: 60px;
        height: 30px;
        background-color: #7f8c8d;
        border-radius: 15px;
        cursor: pointer;
        transition: background-color 0.3s ease;
        position: relative;
      }

      .toggle-container.active {
        background-color: #3498db;
      }

      .toggle-knob {
        position: absolute;
        top: 2px;
        left: 2px;
        width: 26px;
        height: 26px;
        background-color: white;
        border-radius: 50%;
        transition: transform 0.3s ease, left 0.3s ease;
      }

      .toggle-container.active .toggle-knob {
        left: calc(100% - 28px);
      }
      .mode-label {
        margin-top: 5px;
        color: white;
        text-align: center;
        overflow: hidden;
        text-overflow: ellipsis;
        white-space: nowrap;
      }

      .info-panel {
        position: absolute;
        top: 0px;
        right: 0px;
        background-color: rgba(44, 62, 80, 0.9);
        color: white;
        padding: 10px 10px;
        border-radius: 8px;
        font-family: "Roboto Mono", monospace;
        font-size: 10px;
        box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
        display: grid;
        grid-template-columns: 60px 60px;
        gap: 0px; /* No extra spacing */
      }

      .columns {
        width: 120px;
        display: grid;
        grid-template-columns: 60px 60px; /* Two equal columns */
      }

      .column {
        text-align: left; /* Align all text to the left */
      }

      .column span {
        display: flex; /* Use flexbox for alignment */
        justify-content: space-between; /* Align label to the left and value to the right */
        text-align: right; /* Align the value content to the right */
        margin-bottom: 5px; /* Add spacing between rows */
        white-space: nowrap; /* Prevent wrapping for consistent layout */
      }
      .label {
        text-align: left; /* Align the label to the left */
        margin-right: 0px; /* Add spacing between the label and value */
      }

      .distance-value {
        text-align: right; /* Align the value to the right */
        margin-right: 7px;
      }
      .heading-value {
        text-align: right; /* Align the value to the right */
        margin-right: 0px;
      }

      .steering-throttle {
        grid-column: 1 / -1; /* Make these span the full width below the columns */
        display: flex;
        flex-direction: column; /* Stack them vertically */
        margin-top: 5px; /* Add space between the rows and these values */
      }

      .steering-throttle span {
        text-align: left; /* Align the labels and numbers to the left */
        margin-bottom: 5px; /* Add spacing between steering and throttle */
      }

      #steering-value,
      #throttle-value {
        margin: 0; /* Remove spacing between these two elements */
      }

      .info-panel span {
        margin-bottom: 0px;
      }
      /* New styles for LED indicators */
      .led-indicators {
        display: flex;
        justify-content: flex-start; /* Align LEDs to the left */
        margin: 5px 0; /* Adjust spacing as needed */
      }

      .led-container {
        display: flex;
        flex-direction: row; /* Arrange LED and label side by side */
        align-items: center;
        margin-right: 10px;
      }

      .led {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        background-color: red;
      }

      .led-label {
        font-size: 10px;
        color: #ecf0f1;
        margin-left: 5px; /* Space between LED and label */
      }

      #parameters-button {
        font-size: 10px; /* Smaller text */
        border-radius: 5px;
      }

      #reset-zoom {
        position: absolute;
        bottom: 50%; /* Center vertically */
        transform: translateY(50%);
        left: 0px; /* Position near the left edge */
        padding: 10px 15px;
        font-size: 40px;
        font-family: "Roboto Mono", monospace;
        color: white;
        background-color: rgba(52, 152, 219, 0.1); /* Semi-transparent background */
        border: none;
        border-radius: 8px;
        cursor: pointer;
        box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
        transition: background-color 0.3s ease;
      }

      #reset-zoom:hover {
        background-color: rgba(41, 128, 185, 0.1); /* Adjust hover transparency */
      }

      #reset-zoom:active {
        background-color: rgba(31, 95, 127, 0.1); /* Adjust active transparency */
      }

      .slider-container-horizontal {
        position: absolute;
        bottom: 5%;
        width: 90%;
        display: flex;
        flex-direction: column;
        align-items: center;
      }
      .slider-container-vertical {
        position: absolute;
        right: 10%;
        width: 10px;
        display: flex;
        flex-direction: column;
        align-items: center;
      }
      .vertical-slider {
        -webkit-appearance: none;
        appearance: none;
        width: 420px;
        height: 50px;
        background: rgba(189, 195, 199, 0.1); /* Semi-transparent background */
        outline: none;
        border-radius: 10px;
        transform: rotate(-90deg);
        transform-origin: 50% 50%;
        cursor: pointer;
      }
      .vertical-slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 80px;
        height: 80px;
        background: #3498db;
        border-radius: 50%;
        cursor: pointer;
      }
      .horizontal-slider {
        -webkit-appearance: none;
        appearance: none;
        width: 80%;
        height: 50px;
        background: rgba(189, 195, 199, 0.1); /* Semi-transparent background */
        outline: none;
        border-radius: 10px;
        transform: rotate(180deg); /* Flip the slider */
        cursor: pointer;
      }
      .horizontal-slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 80px;
        height: 80px;
        background: #3498db;
        border-radius: 50%;
        cursor: pointer;
      }
      .horizontal-slider:disabled::-webkit-slider-thumb,
      .vertical-slider:disabled::-webkit-slider-thumb {
        background: #bdc3c7; /* Light gray for disabled state */
        cursor: not-allowed; /* Change cursor to indicate non-interactivity */
      }
    </style>
  </head>
  <body>
    <canvas id="coordinate-system"></canvas>

    <div class="location-panel" id="location-panel">
      <div class="current-location" id="current-location">
        <div class="label">current:</div>
        <div class="location-value">
          <span class="paren">(</span>
          <span id="current-x" class="coord">00.00</span>
          <span class="paren">,</span>
          <span id="current-y" class="coord">00.00</span>
          <span class="paren">)</span>
        </div>
      </div>
      <div class="target-location" id="target-location">
        <div class="label">target:</div>
        <div class="location-value">
          <span class="paren">(</span>
          <span id="target-x" class="coord">00.00</span>
          <span class="paren">,</span>
          <span id="target-y" class="coord">00.00</span>
          <span class="paren">)</span>
        </div>
      </div>
      <div class="toggle-container" id="toggle-button">
        <div class="toggle-knob"></div>
      </div>
      <div class="mode-label" id="mode-label">Manual Mode</div>
    </div>

    <div class="info-panel" id="info-panel">
      <!-- Distance and Heading Columns -->
      <div class="columns">
        <div class="column">
          <span>
            <span class="label">d:</span>
            <span class="distance-value" id="distance">00.00</span>
          </span>
        </div>
        <div class="column">
          <span>
            <span class="label">h:</span>
            <span class="heading-value" id="heading">000.00</span>
          </span>
        </div>
      </div>
      <!-- Steering and Throttle -->
      <div class="steering-throttle">
        <span id="steering-value">Steering: 1500</span>
        <span id="throttle-value">Throttle: 1500</span>
        <div class="led-indicators">
          <div class="led-container">
            <div class="led" id="sensor-led"></div>
            <span class="led-label">Sensor</span>
          </div>
          <div class="led-container">
            <div class="led" id="remote-led"></div>
            <span class="led-label">Remote</span>
          </div>
        </div>
        <button id="parameters-button" onclick="window.location.href='/parameters'">Parameters</button>
      </div>
    </div>

    <button id="reset-zoom">⟳</button>

    <div class="slider-container-horizontal">
      <input type="range" id="steering-slider" min="1000" max="2000" value="1500" class="horizontal-slider" />
    </div>
    <div class="slider-container-vertical">
      <input type="range" id="throttle-slider" min="1000" max="2000" value="1500" class="vertical-slider" />
    </div>

    <script>
      document.addEventListener("dblclick", (e) => e.preventDefault());
      document.addEventListener("contextmenu", (e) => e.preventDefault());

      const canvas = document.getElementById("coordinate-system");
      const ctx = canvas.getContext("2d");

      let scale = 1;
      const scaleStep = 0.2;
      const minScale = 0.25;
      const maxScale = 5;
      const defaultScale = 1;

      const midSliderValue = 1500;

      let data = {
        currentX: 0,
        currentY: 0,
        targetX: 0,
        targetY: 0,
        distance: 0,
        heading: 0,
        steering: 0,
        throttle: 0,
        state: 0,
      };

      let driveMode = 0;

      function updateInfoPanel() {
        document.getElementById("distance").textContent = `${data.distance}`;
        document.getElementById("heading").textContent = `${data.heading}`;
      }

      function updateLocationPanel() {
        document.getElementById("current-x").textContent = `${data.currentX}`;
        document.getElementById("current-y").textContent = `${data.currentY}`;

        if (driveMode == 0) {
          document.getElementById("target-x").textContent = `${data.targetX}`;
          document.getElementById("target-y").textContent = `${data.targetY}`;
        }
      }

      function updateSteeringThrottlePanel() {
        document.getElementById("steering-value").textContent = `Steering: ${data.steering}`;
        document.getElementById("throttle-value").textContent = `Throttle: ${data.throttle}`;
      }

      function updateSliders() {
        if (driveMode == 1) {
          document.getElementById("steering-slider").value = data.steering;
          document.getElementById("throttle-slider").value = data.throttle;
        }
      }

      const toggleButton = document.getElementById("toggle-button");
      const modeLabel = document.getElementById("mode-label");

      toggleButton.addEventListener("click", () => {
        driveMode = driveMode == 0 ? 1 : 0;
        toggleButton.classList.toggle("active");
        modeLabel.textContent = driveMode == 1 ? "Auto Mode" : "Manual Mode";

        // Send mode and coordinates to the server
        if (ws.readyState === WebSocket.OPEN) {
          const modeMessage = driveMode == 1 ? "mode=auto" : "mode=manual";
          ws.send(modeMessage);

          if (driveMode == 1) {
            // Send current x and y as target coordinates
            const coordinatesMessage = `x=${data.currentX}&y=${data.currentY}`;
            ws.send(coordinatesMessage);
          } else {
            // Reset the slider.
            steeringSlider.value = midSliderValue;
            throttleSlider.value = midSliderValue;
          }
        }

        // Update UI
        document.getElementById("target-location").style.color = driveMode == 1 ? "#7f8c8d" : "white";

        drawCoordinateSystem();

        // Enable or disable sliders based on mode
        const sliders = document.querySelectorAll("#steering-slider, #throttle-slider");
        sliders.forEach((slider) => {
          slider.disabled = driveMode == 1; // Disable input when auto mode is on
        });
      });

      let lastTouchDistance = null;
      let isTouchingToggle = false;

      // Handle pinch-to-zoom and ensure it doesn't interfere with toggle button
      canvas.addEventListener("touchstart", (event) => {
        if (event.touches.length === 1) {
          const touch = event.touches[0];
          const rect = toggleButton.getBoundingClientRect();
          isTouchingToggle = touch.pageX >= rect.left && touch.pageX <= rect.right && touch.pageY >= rect.top && touch.pageY <= rect.bottom;
        }
        if (event.touches.length === 2) {
          lastTouchDistance = Math.hypot(event.touches[1].pageX - event.touches[0].pageX, event.touches[1].pageY - event.touches[0].pageY);
        }
      });

      canvas.addEventListener("touchmove", (event) => {
        if (isTouchingToggle || event.touches.length !== 2) return;

        const touch1 = event.touches[0];
        const touch2 = event.touches[1];
        const distance = Math.hypot(touch2.pageX - touch1.pageX, touch2.pageY - touch1.pageY);

        if (lastTouchDistance !== null) {
          const scaleChange = (distance - lastTouchDistance) / 100;
          scale = Math.min(maxScale, Math.max(minScale, scale + scaleChange));
          drawCoordinateSystem();
        }
        lastTouchDistance = distance;
      });

      canvas.addEventListener("touchend", (event) => {
        if (event.touches.length === 0) {
          isTouchingToggle = false;
          lastTouchDistance = null;
        }
      });

      function drawCar(x, y) {
        drawTriangle(x, y, "white");
      }

      function drawTarget(x, y) {
        drawTriangle(x, y, "#7f8c8d");
      }

      function drawCurrent(x, y) {
        drawTriangle(x, y, "#3498db");
      }

      function drawTriangle(x, y, color) {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(x, y - 15);
        ctx.lineTo(x - 15, y + 15);
        ctx.lineTo(x + 15, y + 15);
        ctx.closePath();
        ctx.fill();
      }

      function drawCoordinateSystem() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;

        const width = canvas.width;
        const height = canvas.height;
        const centerX = width / 2;
        const centerY = height / 2;
        const meterToPixel = (height / 10) * scale;

        ctx.clearRect(0, 0, width, height);

        ctx.strokeStyle = "#ecf0f1";
        ctx.lineWidth = 2;

        ctx.beginPath();
        ctx.moveTo(0, centerY);
        ctx.lineTo(width, centerY);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(centerX, 0);
        ctx.lineTo(centerX, height);
        ctx.stroke();

        const baseStep = scale >= 3 ? 0.2 : scale >= 1.5 ? 0.5 : scale >= 0.5 ? 1 : 2;
        const step = baseStep;

        function formatNumber(num) {
          return Math.abs(num) < 1e-10 ? "0" : num.toFixed(1).replace(/\.0$/, "");
        }

        const maxX = Math.ceil(width / (2 * meterToPixel));
        for (let i = -maxX; i <= maxX; i += step) {
          if (Math.abs(i) < 1e-10) continue;
          const x = centerX + i * meterToPixel;
          const label = formatNumber(i);
          ctx.font = "14px Arial";
          ctx.fillStyle = "#ecf0f1";
          ctx.textAlign = "center";
          ctx.fillText(label, x, centerY + 15);
        }

        const maxY = Math.ceil(height / (2 * meterToPixel));
        for (let i = -maxY; i <= maxY; i += step) {
          if (Math.abs(i) < 1e-10) continue;
          const y = centerY - i * meterToPixel;
          const label = formatNumber(i);
          ctx.font = "14px Arial";
          ctx.fillStyle = "#ecf0f1";
          ctx.textAlign = "right";
          ctx.fillText(label, centerX - 10, y + 5);
        }

        if (driveMode == 1) {
          const targetX = centerX + data.targetX * meterToPixel;
          const targetY = centerY - data.targetY * meterToPixel;
          drawTarget(targetX, targetY);
        }

        const currentX = centerX + data.currentX * meterToPixel;
        const currentY = centerY - data.currentY * meterToPixel;
        drawCurrent(currentX, currentY);

        drawCar(centerX, centerY);
      }

      const resetZoomButton = document.getElementById("reset-zoom");

      resetZoomButton.addEventListener("click", () => {
        scale = defaultScale;
        drawCoordinateSystem();
      });

      drawCoordinateSystem();

      // WebSocket connection and polling logic
      let ws;
      const reconnectInterval = 500; // Reconnection interval (ms)
      const pollInterval = 10; // Polling interval (ms)

      const steeringValue = document.getElementById("steering-value");
      const steeringSlider = document.getElementById("steering-slider");
      const throttleValue = document.getElementById("throttle-value");
      const throttleSlider = document.getElementById("throttle-slider");

      const connectWebSocket = () => {
        ws = new WebSocket(`ws://${location.host}/ws`);

        ws.onopen = () => {
          console.log("WebSocket connected");

          // Start polling server for data every `pollInterval` ms
          setInterval(() => {
            if (ws.readyState === WebSocket.OPEN) {
              ws.send("REQUEST_DATA"); // Send request to server
            }
          }, pollInterval);
        };

        ws.onmessage = (event) => {
          console.log("Message from server:", event.data);

          // Parse the JSON response and update UI
          try {
            data = JSON.parse(event.data);
            updateSliders();
            updateInfoPanel();
            updateLocationPanel();
            updateSteeringThrottlePanel();
            updateLEDs();
            drawCoordinateSystem();
          } catch (err) {
            console.error("Error parsing message:", err);
          }
        };

        ws.onclose = () => {
          console.log("WebSocket disconnected. Attempting to reconnect...");
          setTimeout(connectWebSocket, reconnectInterval);
        };

        ws.onerror = (error) => {
          console.error("WebSocket error:", error);
          ws.close(); // Ensure the connection is closed before reconnecting
        };
      };

      // Establish initial WebSocket connection
      connectWebSocket();

      // Function to smoothly reset a slider to the center
      const resetSlider = (slider, param, duration = 300) => {
        const startValue = parseInt(slider.value, 10);
        const targetValue = midSliderValue;
        const startTime = performance.now();

        const animateReset = (currentTime) => {
          const elapsed = currentTime - startTime;
          const progress = Math.min(elapsed / duration, 1);
          const newValue = Math.round(startValue + (targetValue - startValue) * progress);

          slider.value = newValue;

          // Send intermediate value
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(`${param}=${newValue}`);
          }

          if (progress < 1) {
            requestAnimationFrame(animateReset);
          } else {
            // Ensure the final value is sent as exactly in the middle.
            slider.value = targetValue;
            if (ws.readyState === WebSocket.OPEN) {
              ws.send(`${param}=${targetValue}`);
            }
          }
        };

        requestAnimationFrame(animateReset);
      };

      // Event listeners for steering slider
      steeringSlider.addEventListener("input", () => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(`steering=${steeringSlider.value}`);
        }
      });

      steeringSlider.addEventListener("mouseup", () => {
        resetSlider(steeringSlider, "steering");
      });

      steeringSlider.addEventListener("touchend", () => {
        resetSlider(steeringSlider, "steering");
      });

      // Event listeners for throttle slider
      throttleSlider.addEventListener("input", () => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(`throttle=${throttleSlider.value}`);
        }
      });

      throttleSlider.addEventListener("mouseup", () => {
        resetSlider(throttleSlider, "throttle");
      });

      throttleSlider.addEventListener("touchend", () => {
        resetSlider(throttleSlider, "throttle");
      });

      const STATE_NOT_READY = 0;
      const STATE_SENSOR_READY = 1;
      const STATE_REMOTE_CONTROLLER_READY = 2;

      function updateLEDs() {
        const sensorLed = document.getElementById("sensor-led");
        const remoteLed = document.getElementById("remote-led");

        sensorLed.style.backgroundColor = data.state & STATE_SENSOR_READY ? "green" : "red";
        remoteLed.style.backgroundColor = data.state & STATE_REMOTE_CONTROLLER_READY ? "green" : "#red";
      }

      // Variables for double tap continuous zoom gesture
      let lastTapTime = 0;
      let isDoubleTapZooming = false;
      let startDoubleTapZoomY = 0;

      // When a touch starts, check for a double tap
      canvas.addEventListener("touchstart", (event) => {
        if (event.touches.length === 1) {
          const currentTime = new Date().getTime();
          if (currentTime - lastTapTime < 300) {
            // Double tap detected—enter continuous zoom mode.
            isDoubleTapZooming = true;
            startDoubleTapZoomY = event.touches[0].pageY;
          }
          lastTapTime = currentTime;
        }
      });

      // When a touch moves, if in continuous zoom mode, adjust the scale based on vertical movement.
      canvas.addEventListener("touchmove", (event) => {
        // If in double tap zoom mode and only one finger is active, perform continuous zoom.
        if (isDoubleTapZooming && event.touches.length === 1) {
          const currentY = event.touches[0].pageY;
          const scaleChange = (currentY - startDoubleTapZoomY) / 100;
          scale = Math.min(maxScale, Math.max(minScale, scale + scaleChange));
          startDoubleTapZoomY = currentY; // update for incremental zoom changes
          drawCoordinateSystem();
        }
      });

      // When a touch ends, disable continuous zoom mode.
      canvas.addEventListener("touchend", (event) => {
        if (event.touches.length === 0) {
          isDoubleTapZooming = false;
        }
      });
    </script>
  </body>
</html>
)rawliteral";

char const* parameters_page_html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <!-- Disable pinch zoom by setting maximum-scale to 1.0 and user-scalable to no -->
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
    <title>Parameters</title>
    <style>
      /* Disable scrolling and text selection on the entire document */
      html,
      body {
        overflow: hidden;
        user-select: none;
        -webkit-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
      }
      body {
        font-family: Arial, sans-serif;
        padding: 10px; /* Reduced overall padding */
        touch-action: none; /* Prevent default pinch-zoom on the webpage */
      }
      /* Header container for the back button, title, and reset button */
      .header {
        position: relative;
        text-align: center;
        margin-bottom: 10px;
      }
      /* Small back button positioned at the left of the header */
      .back-button {
        position: absolute;
        left: 0;
        top: 50%;
        transform: translateY(-50%);
        font-size: 12px; /* Small text */
        padding: 4px 8px; /* Small padding */
        border-radius: 5px;
        background-color: #eeeeee;
        color: #007bff;
        border: none;
        cursor: pointer;
      }
      .back-button:hover {
        background-color: #e0e0e0; /* Slightly darker on hover */
      }
      /* Reset button styled similarly to the back button, positioned at the right */
      .reset-button {
        position: absolute;
        right: 0;
        top: 50%;
        transform: translateY(-50%);
        font-size: 12px; /* Small text */
        padding: 4px 8px; /* Small padding */
        border-radius: 5px;
        background-color: #eeeeee;
        color: #007bff;
        border: none;
        cursor: pointer;
      }
      .reset-button:hover {
        background-color: #e0e0e0; /* Slightly darker on hover */
      }
      h2 {
        font-size: 16px; /* Smaller heading text */
        margin: 0; /* Remove default margin */
      }
      .slider-container {
        margin: 10px 0; /* Smaller margin between sliders */
        width: 100%;
      }
      .slider-container label {
        display: block;
        font-size: 14px; /* Smaller label text */
        margin-bottom: 3px; /* Reduced bottom margin for labels */
      }
      .slider {
        width: 100%;
        height: 8px;
        background: #ddd;
        border-radius: 4px;
        outline: none;
        transition: 0.2s;
      }
      input[type="submit"] {
        display: block;
        width: 100%;
        padding: 8px; /* Reduced padding */
        font-size: 14px; /* Smaller button text */
        background-color: #007bff;
        color: white;
        border: none;
        cursor: pointer;
        border-radius: 5px;
        margin-top: 10px; /* Smaller top margin */
      }
      input[type="submit"]:hover {
        background-color: #0056b3;
      }
      #status {
        font-size: 12px; /* Smaller status text */
        margin-top: 10px;
      }
    </style>
  </head>
  <body>
    <div class="header">
      <button type="button" class="back-button" onclick="window.location.href='/'">Back</button>
      <h2>Autocam Parameters</h2>
      <button type="button" class="reset-button" onclick="resetSliders()">Reset</button>
    </div>
    <form id="parameters-form">
      <div class="slider-container">
        <label>Delta: <span id="delta-value">0.5</span></label>
        <input class="slider" type="range" name="delta" min="0" max="2" step="0.01" value="0.5" oninput="updateValue('delta', this.value)" />
      </div>
      <div class="slider-container">
        <label>Kp_t: <span id="Kp_t-value">100</span></label>
        <input class="slider" type="range" name="Kp_t" min="0" max="200" step="0.1" value="100" oninput="updateValue('Kp_t', this.value)" />
      </div>
      <div class="slider-container">
        <label>Ki_t: <span id="Ki_t-value">0</span></label>
        <input class="slider" type="range" name="Ki_t" min="0" max="10" step="0.1" value="0" oninput="updateValue('Ki_t', this.value)" />
      </div>
      <div class="slider-container">
        <label>Kd_t: <span id="Kd_t-value">0</span></label>
        <input class="slider" type="range" name="Kd_t" min="0" max="10" step="0.1" value="0" oninput="updateValue('Kd_t', this.value)" />
      </div>
      <div class="slider-container">
        <label>Kp_s: <span id="Kp_s-value">10</span></label>
        <input class="slider" type="range" name="Kp_s" min="0" max="20" step="0.1" value="10" oninput="updateValue('Kp_s', this.value)" />
      </div>
      <div class="slider-container">
        <label>Ki_s: <span id="Ki_s-value">0</span></label>
        <input class="slider" type="range" name="Ki_s" min="0" max="10" step="0.1" value="0" oninput="updateValue('Ki_s', this.value)" />
      </div>
      <div class="slider-container">
        <label>Kd_s: <span id="Kd_s-value">0</span></label>
        <input class="slider" type="range" name="Kd_s" min="0" max="10" step="0.1" value="0" oninput="updateValue('Kd_s', this.value)" />
      </div>
      <div class="slider-container">
        <label>Max Throttle: <span id="maxMoveThrottle-value">300</span></label>
        <input class="slider" type="range" name="maxMoveThrottle" min="0" max="500" step="1" value="300" oninput="updateValue('maxMoveThrottle', this.value)" />
      </div>
      <div class="slider-container">
        <label>Min Throttle: <span id="minMoveThrottle-value">-300</span></label>
        <input class="slider" type="range" name="minMoveThrottle" min="0" max="500" step="1" value="300" oninput="updateValue('minMoveThrottle', this.value)" />
      </div>
      <div class="slider-container">
        <label>Max Steering: <span id="maxMoveSteering-value">500</span></label>
        <input class="slider" type="range" name="maxMoveSteering" min="0" max="500" step="1" value="500" oninput="updateValue('maxMoveSteering', this.value)" />
      </div>
      <div class="slider-container">
        <label>Min Steering: <span id="minMoveSteering-value">-500</span></label>
        <input class="slider" type="range" name="minMoveSteering" min="0" max="500" step="1" value="500" oninput="updateValue('minMoveSteering', this.value)" />
      </div>
      <input type="submit" value="Update" />
    </form>
    <p id="status"></p>
    <script>
      // Update the display. For the two "min" sliders, show the negative.
      function updateValue(id, value) {
        let displayValue = value;
        if (id === "minMoveThrottle" || id === "minMoveSteering") {
          displayValue = -value;
        }
        document.getElementById(id + "-value").textContent = displayValue;
        // Clear any successful message when a slider is changed.
        document.getElementById("status").textContent = "";
      }
      // Reset all sliders to their current values as in the form.
      function resetSliders() {
        // Reset the form to its initial state.
        document.getElementById("parameters-form").reset();
        // Update all slider displays.
        document.querySelectorAll("input[type='range']").forEach((slider) => {
          updateValue(slider.name, slider.value);
        });
      }
      // Function to fetch current parameters from the server and update the form inputs.
      function fetchParameters() {
        fetch("/get_parameters")
          .then((response) => response.json())
          .then((data) => {
            document.getElementById("delta-value").textContent = data.delta;
            document.querySelector("input[name='delta']").value = data.delta;

            document.getElementById("Kp_t-value").textContent = data.Kp_t;
            document.querySelector("input[name='Kp_t']").value = data.Kp_t;

            document.getElementById("Ki_t-value").textContent = data.Ki_t;
            document.querySelector("input[name='Ki_t']").value = data.Ki_t;

            document.getElementById("Kd_t-value").textContent = data.Kd_t;
            document.querySelector("input[name='Kd_t']").value = data.Kd_t;

            document.getElementById("Kp_s-value").textContent = data.Kp_s;
            document.querySelector("input[name='Kp_s']").value = data.Kp_s;

            document.getElementById("Ki_s-value").textContent = data.Ki_s;
            document.querySelector("input[name='Ki_s']").value = data.Ki_s;

            document.getElementById("Kd_s-value").textContent = data.Kd_s;
            document.querySelector("input[name='Kd_s']").value = data.Kd_s;

            document.getElementById("maxMoveThrottle-value").textContent = data.maxMoveThrottle;
            document.querySelector("input[name='maxMoveThrottle']").value = data.maxMoveThrottle;

            // For the min sliders, display the negative value but set the slider to the absolute value.
            document.getElementById("minMoveThrottle-value").textContent = data.minMoveThrottle;
            document.querySelector("input[name='minMoveThrottle']").value = Math.abs(data.minMoveThrottle);

            document.getElementById("maxMoveSteering-value").textContent = data.maxMoveSteering;
            document.querySelector("input[name='maxMoveSteering']").value = data.maxMoveSteering;

            document.getElementById("minMoveSteering-value").textContent = data.minMoveSteering;
            document.querySelector("input[name='minMoveSteering']").value = Math.abs(data.minMoveSteering);
          })
          .catch((err) => {
            console.error("Error fetching parameters:", err);
          });
      }
      // When the page loads, fetch the current parameter values.
      window.onload = function () {
        fetchParameters();
      };
      // Form submission to update parameters on the ESP32.
      document.getElementById("parameters-form").onsubmit = async (e) => {
        e.preventDefault();
        let formData = new FormData(e.target);
        // For the min sliders, convert the positive value to negative before sending.
        if (formData.has("minMoveThrottle")) {
          let val = Number(formData.get("minMoveThrottle"));
          formData.set("minMoveThrottle", -val);
        }
        if (formData.has("minMoveSteering")) {
          let val = Number(formData.get("minMoveSteering"));
          formData.set("minMoveSteering", -val);
        }
        fetch("/update_parameters", { method: "POST", body: formData })
          .then((response) => response.text())
          .then((data) => {
            document.getElementById("status").textContent = "Parameters updated successfully";
            // Optionally, re-fetch parameters to update the UI.
            fetchParameters();
          })
          .catch((err) => {
            console.error("Error updating parameters:", err);
          });
      };
    </script>
  </body>
</html>
)rawliteral";
