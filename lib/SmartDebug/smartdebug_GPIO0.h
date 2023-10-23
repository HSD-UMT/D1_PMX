/*********************************************
 * smartdebug
 * Debug - Makros für die Arduino - IDE
 * 
 * 12.08.2018 Vers. 2.0
 * Der Hobbyelektroniker
 * der.hobbyelektroniker@gmail.com
 * https://www.youtube.com/c/HobbyelektronikerCh (Youtube - Kanal)
 * https://community.hobbyelektroniker.ch
 * https://www.hobbyelektroniker.ch
 *
 * Der Code ist lizenzfrei und darf von jedermann frei verwendet werden
 * Trotzdem freut es mich natürlich, wenn dieser Kommentar stehen bleibt ;-)
 * Feedback, Verbesserungen und Erweiterungen sind ebenfalls sehr erwünscht.
 *********************************************/


#ifdef _SMARTDEBUG
  // Hilfsfunktion für WAIT - Makro
  void dwait(String txt){
    // buffer leeren
    char ch;
    while (Serial.available()) ch = Serial.read();
    ch = 0;
  
    Serial.print(txt);
    Serial.println("\nPress 'c' to continue..."); 

    // auf 'c' warten
    do {
      if (Serial.available() > 0) ch = Serial.read();
    } while (ch != 'c'); 
    // buffer leeren
    while (Serial.available()) ch = Serial.read();
  }  

  // Initialisierung
  #define dinit(speed) Serial.begin(speed)

  // Textausgabe
  #define dprint(txt) Serial.print(txt)
  #define dprintln(txt) Serial.println(txt)
  #define dprintV(txt, val) Serial.print(txt); Serial.print(": "); Serial.print(val)
  #define dprintlnV(txt, val) Serial.print(txt); Serial.print(": "); Serial.println(val)
  
  // andere Befehle
  #define ddelay(ms) delay(ms)
  #define dwait(txt, condition) if (condition) dwait(txt)
#else
  // Leere Makro - Rümpfe, für den Fall, dass kein Debugging erwünscht ist
  #define dinit(speed)
  #define dprint(txt)
  #define dprintln(txt)
  #define dprintV(txt, val)
  #define dprintlnV(txt, val)
  #define ddelay(ms)
  #define dwait(txt, condition)
#endif
