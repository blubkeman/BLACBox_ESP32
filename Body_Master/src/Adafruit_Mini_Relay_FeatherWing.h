#ifndef FeatherWing_Mini_Relay_h
#define FeatherWing_Mini_Relay_h

class Adafruit_Mini_Relay_FeatherWing : public SetupEvent
{
public:
  Adafruit_Mini_Relay_FeatherWing() {}
  ~Adafruit_Mini_Relay_FeatherWing() {}

  virtual void setup() override
  {
    pinMode(RELAY_SET_PIN, OUTPUT);
    pinMode(RELAY_UNSET_PIN, OUTPUT);  // Ignored when using the unlatching relay
    unset();
  }

  #if defined(LATCHING)
  void set()
  {
    digitalWrite(RELAY_SET_PIN, HIGH);
    delay(10);
    digitalWrite(RELAY_SET_PIN, LOW);
    relaySet = true;
  }

  void unset()
  {
    digitalWrite(RELAY_UNSET_PIN, LOW);
    delay(10);
    digitalWrite(RELAY_UNSET_PIN, HIGH);
    relaySet = false;
  }

  #elif defined(UNLATCHING)
  void set()
  {
    digitalWrite(RELAY_SET_PIN, HIGH);
  }

  void unset()
  {
    digitalWrite(RELAY_SET_PIN, LOW);
  }
  #endif

protected:
  #if defined(LATCHING)
  bool relaySet;
  #endif
};

#endif
