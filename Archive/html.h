#ifndef CODE_H
#define CODE_H

#include <string>

const std::string htmlContent = R"(
<!DOCTYPE html>
<html lang="en">

<head>
    <title>Control Panel</title>
    <script src="https://cdn.jsdelivr.net/npm/@tailwindcss/browser@4"></script>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #f4f4f4;
            color: #333;
            margin: 0;
            padding: 0;
            background-color: oklch(14.1% 0.005 285.823);
            width: 100vw;
            height: 100vh;
        }

        .topbar {
            width: 100%;
            height: 60px;
            background-color: oklch(21% 0.006 285.885);
            display: inline-flex;
            align-items: center;
            justify-content: center;
        }

        .topbar p {
            width: 100%;
            text-align: center;
            margin: auto;
            color: white;
            font-size: 25px;
            font-weight: bold;
        }

        .content {
            height: calc(100% - 60px);
            width: 100%;
            display: flex;
            justify-content: flex-start;
            align-items: center;
            padding-top: 25px;
            padding-bottom: 25px;
            gap: 20px;
            flex-wrap: wrap;
            flex-direction: column;
        }

        .control {
            display: flex;
            align-items: center;
            flex-wrap: wrap;
            flex-direction: column;
        }

        .control p {
            color: white;
            font-size: large;
            font-weight: bold;
        }

        .slider-c {
            display: flex;
            gap: 15px;
        }
    </style>
</head>

<body>
    <div class="topbar">
        <p>Deliver Bot Control Panel</p>
    </div>
    <div class="content">
        <div class="control">
            <p>DC Motor Power Control</p>
            <div class="slider-c">
                <input type="range" min="0" max="255" value="0" class="slider" id="motor">
                <p style="font-weight: 100 !important; font-size: medium !important;" id="motor-speed">0</p>
            </div>
        </div>
        <div class="control">
            <p>Servo Angle Control</p>
            <div class="slider-c">
                <input type="range" min="0" max="180" value="0" class="slider" id="servo">
                <p style="font-weight: 100 !important; font-size: medium !important;" id="servo-speed">0</p>
            </div>
        </div>
        <div class="control">
            <p>Ultrasonic Distance</p>
            <p style="font-size: x-large; font-weight: 100;"><span id="distance">0</span> cm</p>
        </div>
    </div>
    <script>
        const motorSlider = document.getElementById("motor");
        const motorValue = document.getElementById("motor-speed");
        motorSlider.addEventListener("input", function () {
            motorValue.innerText = this.value;
        });
        motorSlider.addEventListener("change", function () {
            const xhr = new XMLHttpRequest();
            xhr.open("POST", "/control/motor", true);
            xhr.setRequestHeader("Content-Type", "application/json");
            xhr.send(JSON.stringify({ motor: this.value }));
            xhr.onreadystatechange = function () {
                if (xhr.readyState === XMLHttpRequest.DONE) {
                    if (xhr.status === 200) {
                        console.log("Motor value sent successfully.");
                    } else {
                        console.error("Error sending motor value:", xhr.statusText);
                    }
                }
            };
        });

        const servoSlider = document.getElementById("servo");
        const servoValue = document.getElementById("servo-speed");
        servoSlider.addEventListener("input", function () {
            servoValue.innerText = this.value;
        });
        servoSlider.addEventListener("change", function () {
            const xhr = new XMLHttpRequest();
            xhr.open("POST", "/control/servo", true);
            xhr.setRequestHeader("Content-Type", "application/json");
            xhr.send(JSON.stringify({ servo: this.value }));
            xhr.onreadystatechange = function () {
                if (xhr.readyState === XMLHttpRequest.DONE) {
                    if (xhr.status === 200) {
                        console.log("Servo value sent successfully.");
                    } else {
                        console.error("Error sending servo value:", xhr.statusText);
                    }
                }
            };
        });

        function updateDistance() {
            const xhr = new XMLHttpRequest();
            xhr.open("GET", "/control/distance", true);
            xhr.onreadystatechange = function () {
                if (xhr.readyState === XMLHttpRequest.DONE) {
                    if (xhr.status === 200) {
                        const response = JSON.parse(xhr.responseText);
                        document.getElementById("distance").innerText = response.distance;
                    } else {
                        console.error("Error fetching distance:", xhr.statusText);
                    }
                }
            };
            xhr.send();
        }
        // setInterval(updateDistance, 500); // Update distance every second


    </script>
</body>

</html>
)";

#endif // CODE_H