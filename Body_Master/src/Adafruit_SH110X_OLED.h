#ifndef Adafruit_SH110X_OLED_h
#define Adafruit_SH110X_OLED_h

#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "JoystickController.h"

enum Page_e
{
  CONTROLLER_NONE,
  CONTROLLER_ACTIVE
};

enum Alignment_e
{
  LEFT,
  CENTERLEFT, 	// Center, align more left when padding spaces is odd;  ex: "123 "
  CENTERRIGHT,	// Center, align more right when padding spaces is odd; ex: " 123"
  RIGHT
};

class SH110X_OLED : public SetupEvent, public AnimatedEvent
{
public:
  SH110X_OLED() {}
  ~SH110X_OLED() {}

  virtual void setup() override
  {
    if ( !fDisplay.begin(0x3C, true) ) {
        DEBUG_PRINTLN("OLED display failed to start");
        return;
    }
    DEBUG_PRINTLN("OLED display ready");

    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);

    fDisplay.display();                   // Show the Adafruit splash screen for 1 second
    delay(1000);

    fDisplay.clearDisplay();              // Clear the screen
    fDisplay.setRotation(1);              // Landscape, buttons on left side
    fDisplay.setTextSize(2);              // Text size 2 = 4 rows, 10 characters
    fDisplay.setTextColor(SH110X_WHITE);  // Color choices are white and black
    fDisplay.setCursor(0,0);              // Put cursor in upper left corner

    clearLines();                         // Initialize each display line
    displayPage();                        // Display a blank page
  }

  virtual void animate() override
  {
	if      ( getButtonA() ) { setLine(fLine4, "A", LEFT); }
	else if ( getButtonB() ) { setLine(fLine4, "B", LEFT); }
	else if ( getButtonC() ) { setLine(fLine4, "C", LEFT); }
	else                     { clearLine(fLine4); }

    long now = millis();
    if ( now > (fDisplayTimer + 100) )
    {
      displayPage();
      fDisplayTimer = now;
    }
  }

  uint8_t getPage()
  {
	return fPage;
  }

  void setControllerNone()
  {
	//  __________
	// |          |
	// |    NO    |
	// |CONTROLLER|
	// |__________|

	clearLines();
	setLine(fLine2, "NO", CENTERLEFT);
	setLine(fLine3, "CONTROLLER", LEFT);
	fPage = CONTROLLER_NONE;
  }

  void setControllerActive()
  {
	//  __________
	// |          |
	// |DRIVE CNTL|
	// |  ACTIVE  |
	// |__________|

	clearLines();
	setLine(fLine2, "DRIVE CNTL", LEFT);
	setLine(fLine3, "ACTIVE", CENTERLEFT);
	fPage = CONTROLLER_ACTIVE;
  }

/*
  void setDriveDisplay(JoystickController::State* state, JoystickController::Event* event, uint8_t fSpeedProfile)
  {
	//  __________
	// |DRIVE CNTL|
	// |Spd:SPRINT|
	// |-127/-128 |
	// |__BRAKE___|

	clearLines();
	
	// Line 1
	
	setLine(fLine1, "DRIVE CNTL", LEFT);

	// Line 2

	setLine(fLine2, "Spd:", LEFT);
	switch (fSpeedProfile)
	{
	  case 0 : { setLine(fLine2, "WALK", LEFT, 5); break; }
	  case 1 : { setLine(fLine2, "JOB", LEFT, 5); break; }
	  case 2 : { setLine(fLine2, "RUN", LEFT, 5); break; }
	  case 3 : { setLine(fLine2, "SPRINT", LEFT, 5); break; }
	}

	// Line 3

	String strVal;
	int centerPos = (fLineSize / 2);

	setLine(fLine3, "/", LEFT, centerPos);
	strVal = (String)state->analog.stick.lx;
	setLine(fLine3, strVal, RIGHT, centerPos-4, 4);
	strVal = (String)state->analog.stick.ly;
	setLine(fLine3, strVal, LEFT, centerPos+1, 4);

	// Line 4

	if ( state->button.l1 )
	{
	  setLine(fLine4, "BRAKE", CENTERLEFT);
	}
  }

  void setDomeDisplay()
  {
	//  __________
	// |DOME CNTL |
	// |          |
	// |          |
	// |__________|
	clearLines();

	setLine(fLine1, "DOME CNTL", CENTERLEFT);
  }
 
  void setDomeDisplay()
  {
	setLine(1,"DOME CTRL ");
	setLine(2,"Dir:      ");
	setLine(3,"Spd:      ");
	setLine(4,"Brk:      ");
  }
*/

protected:
  Adafruit_SH1107 fDisplay = Adafruit_SH1107(64, 128, &Wire);
  long fDisplayTimer = 0;
  uint8_t fPage;
  static const int fLineSize = 10;
  static const int fArraySize = fLineSize + 1;
  char fLine1[fArraySize];
  char fLine2[fArraySize];
  char fLine3[fArraySize];
  char fLine4[fArraySize];

  void clearLine(char line[])
  {
	for (int i = 0; i < fLineSize; i++)
	{
	  line[i] = ' ';
	}
  }

  void clearLines()
  {
	clearLine(fLine1);
	clearLine(fLine2);
	clearLine(fLine3);
	clearLine(fLine4);
  }

  void setLine(char line[], String value, const Alignment_e align, int startPos = 0, int fillSize = fLineSize)
  {
	// Constrain the fillSize to fit within fLineSize given the startPos
	
	fillSize = min(fillSize, (fLineSize - startPos));

	// Make sure the string is not larger than the fillSize

	if ( value.length() > fillSize )
	{
	  value = value.substring(0, fillSize);
	}

	// Pad the string with spaces to accomplish the given alignment
	
	for (int i = value.length(); i < fillSize; i++)
	{
      if ( align == LEFT )
	  {
		value += ' ';		// append a space to the end of the string
	  }
	  else if ( align == RIGHT )
	  {
		value = ' ' + value;  // prepend a space to the front of the string
	  }
	  else if ( align == CENTERLEFT || align == CENTERRIGHT )
	  {
		value = ' ' + value + ' ';  // prepend and append a space to the front of the string
		i++;
	  }
	  else
	  {
		DEBUG_PRINTLN("SH110X_OLED::setLine - Invalid alignment");
		return;
	  }
	}

    // Centering adds the same number of spaces on each side of the string.
	// When centering an odd-number of characters in an even-numbered fillSize
	// or an even-number of characters in an odd-numbered fillSise, trim off
	// one space according to the favored alignment.

    if ( value.length() > fillSize )
	{
	  if ( align == CENTERLEFT )
	  {
	    value = value.substring(1, fillSize);
	  }
	  else if ( align == CENTERRIGHT )
	  {
	    value = value.substring(0, fillSize-1);
	  }
	}

    // Convert the final string to a char array

	char c[value.length()+1];
	value.toCharArray(c, sizeof(c));

    // Add the text to the line starting in the position specified.

	for (int i = 0; i < value.length(); i++)
	{
	  line[startPos] = c[i];
	  startPos++;
	}
  }

  void displayPage()
  {
    fDisplay.clearDisplay();
    fDisplay.setCursor(0,0); 

    fDisplay.println(fLine1);
    fDisplay.println(fLine2);
    fDisplay.println(fLine3);
    fDisplay.println(fLine4);

    fDisplay.display();
    delay(1);	
  }

  bool getButtonA()
  {
	if( !digitalRead(BUTTON_A) ) { return true; }
	return false;
  }

  bool getButtonB()
  {
	if( !digitalRead(BUTTON_B) ) { return true; }
	return false;
  }

  bool getButtonC()
  {
	if( !digitalRead(BUTTON_C) ) { return true; }
	return false;	
  }

/*
  void stringToCharArray(const String value, Alignment_e align char* pBuf, int maxSize)
  {
	maxSize = min(maxSize, fLineSize);

    if ( value.length() == maxSize )
	{
	  value.toCharArray(pBuf, maxSize);
	}

	char buf[fLineSize];
	value.toCharArray(buf,fLineSize);

	if ( value.length() < maxSize )
	{
	  for (int i = value.length(); i < maxSize; i++)
	  {
		if ((value.length() % 2) == 0 )
		{
		  value = ' ' + value;
		  i++;
		}
		value = value + ' ';
	  }
	}
	if ( value.length() > fLineSize )
	{
	  value = value.substring(0,9);
	}

  }
	  case line1 : { stringToCharArray(value, align, &fLine1, fLineSize); break; }
	  case line2 : { stringToCharArray(value, align, &fLine2, fLineSize); break; }
	  case line3 : { stringToCharArray(value, align, &fLine3, fLineSize); break; }
	  case line4 : { stringToCharArray(value, align, &fLine4, fLineSize); break; }
*/

/*
  }
 */

/*
  void displayPage()
  {
    fNoController = false;

    fDisplay.clearDisplay();
    fDisplay.setCursor(0,0); 

    fDisplay.println("CONTROLLER");
    fDisplay.print("B:"); fDisplay.println(getVerboseButtons());
    fDisplay.print("L:"); fDisplay.print(fGamepad.stick.lx);
    fDisplay.print("/");  fDisplay.println(fGamepad.stick.ly);
    fDisplay.print("R:"); fDisplay.print(fGamepad.stick.rx);
    fDisplay.print("/");  fDisplay.println(fGamepad.stick.ry);

    fDisplay.display();
    delay(1);
  }

  String getVerboseButtons()
  {
      String out = "";
      bool psBase = false;
      
      // Start with base buttons.

      if ( fGamepad.button.up )            { out = "UP"; }
      else if ( fGamepad.button.right )    { out = "RT"; }
      else if ( fGamepad.button.down )     { out = "DN"; }
      else if ( fGamepad.button.left )     { out = "LT"; }
      else if ( fGamepad.button.triangle ) { out = "TRI"; }
      else if ( fGamepad.button.circle )   { out = "CIR"; }
      else if ( fGamepad.button.cross )    { out = "CRS"; }
      else if ( fGamepad.button.square )   { out = "SQR"; }
      else if ( fGamepad.button.ps )       { out = "PS"; psBase = true; }

      // Add modifiers.

      if ( fGamepad.button.l1 )           { out += "+L1"; }
      else if ( fGamepad.button.r1 )      { out += "+R1"; }
      else if ( fGamepad.button.l2 )      { out += "+L2"; }
      else if ( fGamepad.button.r2 )      { out += "+R2"; }
      else if ( fGamepad.button.l3 )      { out += "+L3"; }
      else if ( fGamepad.button.r3 )      { out += "+R3"; }
      else if ( fGamepad.button.select )  { out += "+SELECT"; }
      else if ( fGamepad.button.share )   { out += "+SHARE"; }
      else if ( fGamepad.button.start )   { out += "+START"; }
      else if ( fGamepad.button.options ) { out += "+OPTIONS"; }
      else if ( fGamepad.button.ps && !psBase ) { out += "+PS"; }
      
      return out;
  }
*/
};

#endif
