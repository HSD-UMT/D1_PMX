#ifndef ESP8266_wprintWasHere
#define ESP8266_wprintWasHere

#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

#ifndef WebDebugHTTPServerAdress
	#define WebDebugHTTPServerAdress 80
#endif

#ifndef WebDebugOff
	ESP8266WebServer websocketHttpServer(WebDebugHTTPServerAdress);
	WebSocketsServer webSocket=WebSocketsServer(1968);

	#include <webdebug_html.h>
#endif

void wprint(String s=""){
	#ifndef WebDebugOff
		if(!webSocket.connectedClients(true)){
			return;
		}
		webSocket.broadcastTXT(s);
	#endif
}

void wprintln(String s=""){
	#ifndef WebDebugOff
		if(!webSocket.connectedClients(true)){
			return;
		}
		webSocket.broadcastTXT(s+"\n");
	#endif
}

void webSocketEvent(uint8_t clientID,WStype_t type, uint8_t *payload, size_t length){
	#ifndef WebDebugOff
		switch(type){
			case WStype_DISCONNECTED:
				#ifdef _SMARTDEBUG
					dprint("\n[webSocketEvent] ");
					dprint(String(clientID));
					dprintln(" disconnected");
				#endif
				break;
			case WStype_CONNECTED:
				#ifdef _SMARTDEBUG
					IPAddress ip=webSocket.remoteIP(clientID);
					dprint("\n[webSocketEvent] ");
					dprint(String(clientID));
					dprint(" connected from ");
					dprintln(String(ip[0])+"."+String(ip[1])+"."+String(ip[2])+"."+String(ip[3]));
				#endif

				IPAddress locIP=WiFi.localIP();
				wprint("You are connected to ");
				wprint(WiFi.hostname());
				wprint(" (");
				wprint(String(locIP[0])+"."+String(locIP[1])+"."+String(locIP[2])+"."+String(locIP[3]));
				wprintln(")");
				
				wprint("Compile Date: ");
				wprint(String(__DATE__));
				wprint(" ");
				wprintln(String(__TIME__));
				break;
		}
	#endif
}

void deliverDebugWebpage(){
	#ifndef WebDebugOff
		websocketHttpServer.send(200,"text/html",webDebugHTML);
	#endif
}

String wprintInit(String PageAdress=""){
	#ifndef WebDebugOff
		int l=PageAdress.length();

		if(l){
			if(l>1 && PageAdress!="/"){
				if(PageAdress.charAt(l-1)=='/'){
					PageAdress=PageAdress.substring(0,l-1);
				}
				if(PageAdress.charAt(0)=='/'){
					PageAdress=PageAdress.substring(1);
				}
				PageAdress="/"+PageAdress;
			}
		}else{
			PageAdress="/";
		}
		
		websocketHttpServer.on(PageAdress,deliverDebugWebpage);
		websocketHttpServer.begin();

		webSocket.begin();
		webSocket.onEvent(webSocketEvent);
		
		
		
		String url="http://"+WiFi.hostname()+":"+String(WebDebugHTTPServerAdress)+PageAdress;
		String urlIP="http://"+WiFi.localIP().toString()+":"+String(WebDebugHTTPServerAdress)+PageAdress;

		#ifdef _SMARTDEBUG
			dprintln("\n[wprintInit] wprint initialized :)");
			dprint("[wprintInit] Access WebDebug Page here:\n[wprintInit] "+url);
			dprintln("\n[wprintInit] ..or..\n[wprintInit] "+urlIP);
		#endif
		return url;
	#else
		return "WebDebug is disabled";
	#endif
}
	
void wprintLoop(){
	#ifndef WebDebugOff
		websocketHttpServer.handleClient();
		webSocket.loop();
	#endif
}

#endif
