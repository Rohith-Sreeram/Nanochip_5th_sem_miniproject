#define BLYNK_TEMPLATE_ID "TMPL3z0c8I2ni"
#define BLYNK_TEMPLATE_NAME  "CNC machine monitoring system"
#define BLYNK_AUTH_TOKEN   "T0XZlpIa28NH47Xxk9Oj9mn8ZCzXWLAz"

#include <Wire.h>
#include <MPU6050.h>
#include <BlynkSimpleEsp32.h>

MPU6050 mpu;

// Multinomial regression model
void score(double *input, double *output) {
    double temp[3];
    temp[0] = 3.976030550692553 + input[0] * 0.7649538069543562 + input[1] * -0.10793970623322668;
    temp[1] = -0.5988606224831408 + input[0] * 4.246686140717931 + input[1] * 5.574331172301329;
    temp[2] = -3.377169928209426 + input[0] * -5.011639947672288 + input[1] * -5.4663914660681066;

    for (int i = 0; i < 3; i++) {
        output[i] = temp[i];
    }
}

// Find index of maximum value
int argmax(double *arr, int size) {
    int idx = 0;
    for (int i = 1; i < size; i++) {
        if (arr[i] > arr[idx]) idx = i;
    }
    return idx;
}

// Class names
const char* class_names[3] = {"Degraded", "Faulty", "Healthy"};

// WiFi credentials
char ssid[] = "Sreeram";
char pass[] = "72187218";

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22); // SDA, SCL
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    Serial.println("MPU6050 initialized (skipping testConnection).");
}

void loop() {
    Blynk.run();

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // Read raw MPU6050 data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to g and °/s
    double ax_g = ax / 16384.0;
    double ay_g = ay / 16384.0;
    double az_g = az / 16384.0;

    double gx_dps = gx / 131.0;
    double gy_dps = gy / 131.0;
    double gz_dps = gz / 131.0;

    // Compute net acceleration & angular velocity
    double net_accel = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    double net_gyro  = sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);

    // Prepare model input
    double input[2] = {net_accel, net_gyro};
    double output[3];
    score(input, output);

    // Predict health
    int predicted_class = argmax(output, 3);
    const char* health_status = class_names[predicted_class];

    // Send data to Blynk
    Blynk.virtualWrite(V0, net_accel);
    Blynk.virtualWrite(V1, net_gyro);
    Blynk.virtualWrite(V2, health_status);

    // Debug prints
    Serial.print("Net Accel: "); Serial.print(net_accel, 3);
    Serial.print(" g, Net Gyro: "); Serial.print(net_gyro, 3);
    Serial.print(" °/s, Health: "); Serial.println(health_status);

    delay(500); // send data every 0.5s
}
