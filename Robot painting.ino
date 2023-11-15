const int numberOfFunctionCalls = 5;
const float robotSpeed = 10; // in m/msec
bool paint;

// array of the directions the robot will follow
// 0--->forward, 1--> Right , 2-->Left , 3-->circle with radius
const int arrDirs[19] = {0, 2, 2, 2, 2, 2, 0, 2, 2, 3, 2, 1, 1, 1, 1, 1, 1, 0, 1};

// array of length of the lines the robot will paint in meter
const float arrLength[19] = {5, 2, 0.5, 1, 0.5, 1, 1, 2.5, 2, 0.5, 1, 2.5, 2, 0.5, 1, 0.5, 1, 1, 2.5};

// array that indicates if it will paint or not
const bool arrBoolean[19] = {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1};

// array of the time required in each in milliseconds
float arrTime[19];

int currentTime = 0;
int startedLast = 0;
int k = 0;

const int relayPin = 2;
bool switchedDirection = false;

void moveRight();
void moveLeft();
void moveForward();
void moveCircle();
void setup()
{
    pinMode(relayPin, OUTPUT);
    for (int i = 0; i < 19; i++)
    {
        arrTime[i] = arrLength[i] / robotSpeed;
    }
    Serial.begin(9600);
}

void loop()
{
    moveForward();
    if (!paint)
        return;
    currentTime = millis();

    if (k < 19 && arrBoolean[k])
        digitalWrite(relayPin, HIGH);
    else
        digitalWrite(relayPin, LOW);

    if (k < 19 && !switchedDirection)
    {
        switchedDirection = true;
        switch (arrDirs[k])
        {
        case 0: // move forward
            moveForward();

            break;
        case 1: // move right            
            moveRight();
            
            break;
        case 2: // move left
            moveLeft();
            break;
        case 3: // move circle
            moveCircle();
            break;
        }
    }
    if ((k < 19) && (currentTime - startedLast >= arrTime[k]))
    {
        k++;
        startedLast = millis();
        switchedDirection = false;
    }
}
