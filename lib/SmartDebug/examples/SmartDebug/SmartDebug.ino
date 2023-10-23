/* smart_debug
 * 
 * 12.08.2018 Der Hobbyelektroniker
 * Vers. 1.0
 * Der Code ist lizenzfrei und darf von jedermann frei verwendet werden
 * Shrtened Version ;)
*/

// Debugging einschalten, zum ausschalten auskommentieren
// WICHTIG: zuerst define, dann include !!!
#define _SMARTDEBUG
#include "smartdebug.h"


void setup() {
  dinit(115200); // Initialisierung der seriellen Schnittstelle
  dwait("Breakpoint!", true);
}


void loop(){
	dprint("This has no LN.... ");
	dprintln("but that one does :)");
	
	byte b=random(0,255);
	
	dprintlnV("Random",b);
	
	ddelay(2000);
	
}