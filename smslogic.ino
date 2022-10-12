/**
   ESP32 + DHT22 Example for Wokwi
   
   https://wokwi.com/arduino/projects/322410731508073042
*/

int stav = 0;

//pouzitelne promene
double cas = 0; // minuty od startu
double caspadu = 900000; // minuty od startu
double vyska = 0; // nadmorska vyska
double maxvyska = 0; 
double minvyska = 40000; 
double speed = 0; // rychlost
double maxspeed = 0; 
double napeti = 3.7; // napeti baterky (3.7V a klesa)
double tlak = 0; // jsem pod tlakem
double lat = 49;
double lon = 14;
double teplota_venku = 20; // ještě že jsem vevnitř, venku mrzne!  
double teplota_uvnitr = 0; // 
double vlhkost = 80; // je tu hodně vlhko!
bool pada = false;
int msignal = 0; //cislo od 0 do 31, 
                //0-9 = hodne spatný
                //10-14 = OK
                //15-19 = dobrý
                //20-30 = výborný


void adjust() {
  cas++;
  if (cas > 150) {
    vyska = vyska-300;
    speed = speed +0.2;

  } else {
    vyska = vyska + 300;
    speed = speed - 0.2;
  }
  teplota_venku = teplota_venku - 0.5;
  napeti = napeti - 0.05;
  delay(100);
}


void setup() {
  Serial.begin(115200); // Any baud rate should work
}

void loop() {
  adjust();
  
  // ---------------------------------
  // KOD ODESILANI SMS
  // JSEM PANÁČEK:
  char p1[10];
  char p2[10];
  char poloha[25];
  dtostrf(lat, 1, 5, p1);
  dtostrf(lon, 1, 5, p2);
  sprintf(poloha, "%s,%s", p1,p2);

  // odesli teplotu, kdyz klesne teplota pod nulu 
  if (teplota_venku < 0) {
    odesli(0, "Je mi zima, mrzne :-P");
  };


  if (vyska > maxvyska) {
    maxvyska = vyska;
  };

  if (maxvyska > vyska + 100) {
    odesli(1, "Jsem v nejvyssi vysce, letim dolu!");
    pada = true;
  };


  if (speed > maxspeed) {
    maxspeed = speed;
  };

  if (vlhkost < 40) {
    odesli(4, "Vlhkost klesla pod 40!");
  };

  if (napeti < 3.0) {
    odesli(5, "Dochazi mi stava!");
  };

  if (vyska > 12345) {
    odesli(6, "Jsem ve 12 345 metrech nad zemi!");
  };

  if (vyska > 20000) {
    odesli(7, "Uz jsem nastoupal 20km, je to fuska!");
  };

  if (vyska > 30000) {
    odesli(8, "Jsem ve 30 km, je tu krasne!");
  };

  if (vyska < 2000 and pada) { //JE TO V KILOMETRECH
    odesli(10, "Uz se blizim, poloha ", poloha);
  };

  if (vyska < 700 and pada) {
    if (!(stav & (1 << 11))) {
      Serial.println("NASTAVENO");
      caspadu = cas + 20;
    }
    odesli(11, "PRIPRAVTE SE HLEDAT!", poloha);

  };

  if (cas > caspadu) {
    odesli(12, "Spadl jsem na zem", poloha);
    caspadu = cas + 60;
  }

  if (cas > 60) {
    odesli(13, "Uz letim hodinu, nudim se a kousu si draty");
  };

  if (cas > 120) {
    odesli(14, "Dve hodiny od startu, mam hlad a neni tu KFCcko");
  };



  // KONEC KODU OSDESILANI SMS
  // ---------------------------------
}

void odesli(int cislo, String text) {
    odesli(cislo, text, "");
  }

void odesli(int cislo, String text, String poloha) {
    if (!(stav & (1 << cislo))) {
      stav += 1 << cislo;
      Serial.println(text +poloha);
    }
  }
