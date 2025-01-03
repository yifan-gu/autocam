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
                margin-top: 10px;
                width: 80px;
                height: 40px;
                background-color: #7f8c8d;
                border-radius: 20px;
                cursor: pointer;
                transition: background-color 0.3s ease;
                position: relative;
                left: 3px;
            }

            .toggle-container.active {
                background-color: #3498db;
            }

            .toggle-knob {
                position: absolute;
                top: 2px;
                left: 2px;
                width: 36px;
                height: 36px;
                background-color: white;
                border-radius: 50%;
                transition: transform 0.3s ease, left 0.3s ease;
            }

            .toggle-container.active .toggle-knob {
                left: calc(100% - 38px);
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
        </script>
    </body>
</html>
)rawliteral";
