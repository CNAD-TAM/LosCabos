// ========================= Hardware =========================

#define IN1 8
#define IN2 9
#define ENA 5

#define IN3 10
#define IN4 11
#define ENB 6

// ========================= Parámetros del robot =========================

const float WHEEL_BASE = 0.18;   // distancia entre ruedas (m)
const float WHEEL_RADIUS = 0.03; // radio rueda (m)
const float MAX_WHEEL_RAD_S = 20.0; // velocidad angular max rueda

// ========================= Utils =========================

int omegaToPWM(float w)
{
    float pwm = (w / MAX_WHEEL_RAD_S) * 255.0;

    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    return (int)pwm;
}

void driveMotor(int in1, int in2, int en, int pwm)
{
    if (pwm >= 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(en, pwm);
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(en, -pwm);
    }
}

// ========================= Differential Kinematics =========================

void setVelocity(float v, float w)
{
    float vL = v - (w * WHEEL_BASE / 2.0);
    float vR = v + (w * WHEEL_BASE / 2.0);

    float wL = vL / WHEEL_RADIUS;
    float wR = vR / WHEEL_RADIUS;

    int pwmL = omegaToPWM(wL);
    int pwmR = omegaToPWM(wR);

    driveMotor(IN1, IN2, ENA, pwmL);
    driveMotor(IN3, IN4, ENB, pwmR);
}

// ========================= JSON Parser =========================

void parseCommand(String json)
{
    int vIndex = json.indexOf("\"v\"");
    int wIndex = json.indexOf("\"w\"");

    if (vIndex == -1 || wIndex == -1)
        return;

    int vStart = json.indexOf(":", vIndex) + 1;
    int vEnd = json.indexOf(",", vStart);
    if (vEnd == -1) vEnd = json.indexOf("}", vStart);

    int wStart = json.indexOf(":", wIndex) + 1;
    int wEnd = json.indexOf("}", wStart);

    float v = json.substring(vStart, vEnd).toFloat();
    float w = json.substring(wStart, wEnd).toFloat();

    setVelocity(v, w);
}

// ========================= Setup =========================

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);

    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);

    Serial.begin(115200);
}

// ========================= Loop =========================

void loop()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        parseCommand(cmd);
    }
}
