/*
Codifica ottimizzata, con riduzione del tempo di servizio del 50%.
Per migliorare il tempo di servizio sono state apportate diverse modifiche, l'attivazione dell'interrupt attraverso la 
configurazione diretta dei registri coinvolti nella routine, la riduzione del passaggio dei parametri per valore tra funzione 
e funzione con l'utilizzo di costanti, l'eliminazione della lettura dello stato logico Digital output dell' IR Receiver, 
poichè irrilevante ai fini della decodifica del segnale. 

La codifica ad interrupt, emula un ricevitore seriale asincrono che legge una trasmissione dati codificata in 
protocollo NEC proveniente da un modulo IR Receiver.
La periferica segnala alla CPU di aver bisogno di attenzione mediante un apposito segnale di interrupt request.
L'interrupt interrompe il normale funzionamento del processore, che abbandona il programma corrente e passa ad 
eseguire la procedura di risposta all' interrupt. 
Il programma utente riprenderà la sua esecuzione dal punto in cui è stato interrotto. 
Il vantaggio è che la CPU rimane sempre libera di svolgere il programma utente, finche non viene interrotta dalla periferica
per il tempo strettamente necessario al trasferimento dati. 
*/
const int dataPin=2;  
const int bitNumber = 32;  

// abbiamo bisogno di variabili volatili per la trasmissione gestite con interrupt
volatile uint16_t detectedTime, serviceTime;  
volatile int8_t State, bitCounter; 
volatile uint8_t outputVector[4]; 

// parametri di lavoro utilizzati dalle funzioni principali 

#define  FLOW 562   // stato logico basso per un periodo di 562,5 µs prima di ogni bit 
#define  FLOW_T 56  // FLOW / 10 
#define  ZERO 562   // stato logico alto per un periodo di 562,5 µs che corrisponde al bit 0 
#define  ZERO_T 56  // ZERO / 10 
#define  ONE 1687   // stato logico alto per un periodo di 1687,5 µs che corrisponde al bit 1 
#define  ONE_T 168  // UNO /10 
#define  LOWSTART 9000   // un primo stato logico basso per un periodo di 9000  µs
#define  LOWSTART_T 900  // LOWSTART / 10
#define  HIGHSTART 4500  // un ulteriore stato logico alto per un periodo di 4500 µs
#define  HIGHSTART_T 450 // HIGHSTART / 10

#define  READINGOK 6  // la lettura è completa, stato=6  
#define  FRAMINGER -1 // FRAMMING ERROR stato=-1

void setup() {
  Serial.begin(9600); // Impostazione del baudrate 

  // valutazione tempi della CPU 
 
  char tString[ 10 ];  
  Serial.print("F_CPU: ");
  Serial.println(ltoa(F_CPU, tString, 10 ));  
  Serial.print("CyclesMillis = ");
  Serial.println(microsecondsToClockCycles(1)); 

// azzeramento delle variabili e del vettore di output 

for(uint8_t i=0; i<5; i++)  outputVector[i]=0;  
State=bitCounter = detectedTime = 0; 


pinMode(dataPin, INPUT_PULLUP); // settaggio del pin come INPUT 

/* possiamo eliminare la funzione attachInterrupt(digitalPinToInterrupt(dataPin), IrReceiver, CHANGE); 
ed usare direttamente l'aggancio alla interrupt table, settando opportunamente i seguenti registri */

PCMSK2 |= _BV(PCINT18); // abilito il pin 2 cioè il dataPin all'interrupt attraverso  il registro PCMSK2
PCICR  |= _BV(PCIE2); // abilito il registro maschera PCMSK2 tramite il registro PCICR, quindi attivo l'interrupt


Serial.println ("");
Serial.println("Active interrupt... "); 
}

/*
La funzione principale loop() non ha un ruolo operativo determinate, essa si limita a leggere il valore dello stato in cui si trova l'automa 
a cui facciamo riferimento, ed informa l'utente sull'esito della lettura del dato dal modulo IR Receiver. 
*/
void loop() {

// nel caso in cui lo stato si trovi a -1 oppure a 6, viene segnalato un errore nella lettura del frame oppure effettuato l’output dei dati acquisiti .  

if(State <0) {Serial.println("Framming Error"); State=0; }
else if(State== READINGOK) {

// tutti i byte letti vengono invertiti poiché il processo li acquisisce rovesciati 

for(uint8_t i=0; i<4; i++)
{
outputVector[i]=ReverseByte(outputVector[i]);
}

Serial.println("");

Serial.println("Binary output: ");
Serial.print("Command: "); Serial.print(outputVector[2],BIN); 
Serial.print(" logical inverse "); Serial.print(outputVector[3],BIN); 

Serial.print(" Address: "); Serial.print(outputVector[0],BIN); 
Serial.print(" logical inverse "); Serial.println(outputVector[1],BIN); 
Serial.println("");
Serial.print("Service time: "); Serial.println(serviceTime);

State=0;
serviceTime=0;
}
}

/*
Questa è la funzione principale ed ha un importanza determinante nel processo di lettura dal modulo IR Receiver.
Essa viene richiamata dall'interrupt ad ogni cambio di livello logico, misura i tempi di attesa ed effettua un controllo di compatibilità 
con i tempi “nominali” considerando un determinato margine di tolleranza. 
In relazione allo stato che memorizza ad ogni chiamata, interpreta i tempi secondo le specifiche del protocollo NEC, decodificando 
l'informazione ricevuta o segnalando un errore nel messaggio ricevuto.  
*/
ISR(PCINT2_vect) {

  uint16_t currentTime= micros();  

  detectedTime = currentTime-detectedTime; 
  
  // In relazione allo stato dell'automa, attiva le differenti funzioni con gli opportuni parametri 
  // come indicato dal diagramma degli stati. 

  switch(State) {
case 0: // attesa cambiamento HL, non conto il tempo (diagramma stato IDLE --> HLSW)
State++;
break; 

case 1: // attesa cambiamento LH, verifica che il tempo detectedTime sia compatibile con LOWSTART = 9000 µs (diagramma stato HLSW --> LLSW)
CheckTime_LOWSTART(detectedTime);
break; 

case 2: // attesa cambiamento HL, verifica che il tempo detectedTime sia compatibile HIGHSTART = 4500 µs (diagramma stato LLSW --> HLSWFR)
CheckTime_HIGHSTART(detectedTime);
break;

case 3:  // attesa cambiamento LH, verifica che il tempo detectedTime sia compatibile FLOW = 562 µs (diagramma stato HLSWFR --> LLSWFR)
CheckTime_FLOW(detectedTime);
break;

case 4: // attesa cambiamento HL, verifica che il tempo detectedTime sia compatibile ZERO = 562 µs oppure con ONE = 1687 µs (diagramma stato LLSWFR --> HLSWFR)
CheckBitTime(detectedTime);
break;

case 5: // attesa cambiamento LH, verifica che il tempo detectedTime sia compatibile FLOW = 562 µs end of frame (diagramma stato HLSWFR --> IDLE)
CheckTime_FLOW(detectedTime);

//Interruzione disattivata, l'acquisizione dell'informazione può essere effettuata una sola volta.  
//detachInterrupt(digitalPinToInterrupt(dataPin));
PCICR &= ~_BV(PCIE2); // disabilito il registro maschera PCMSK2 tramite il registro PCICR, quindi disattivo l'interrupt
break;

  }

  detectedTime=currentTime; 

  serviceTime +=micros()-currentTime; // memorizza il tempo passato da quando sono entrato a quando ho svolto tutte le funzioni (tempo servizio)
  
}

// funzione che individua i bit 0 oppure i bit 1 e li scrive nel rispettivo Byte, in relazione alla compatibilità dei tempi “nominali” ZERO ed ONE, 
// con un margine di tolleranza del  20%.
void CheckBitTime(uint16_t time) {
 
  uint8_t i = bitCounter/8; 
  uint8_t Byte = outputVector[i];
  Byte <<=1;
  bitCounter++;
  if(time < (ZERO+ZERO_T) && time > (ZERO-ZERO_T)); else if(time < (ONE+ONE_T) && time > (ONE-ONE_T)) Byte |=1; else State = FRAMINGER; 
  outputVector[i]=Byte; 
  if(bitCounter >= bitNumber){State=5; bitCounter = 0; }
  else State=3;
}

// funzione che individua segnali di start frame, pronto bit, end frame in relazione alla compatibilità dei tempi “nominali” HIGHSTART, LOWSTART, FLOW, 
// con un margine di tolleranza del  20%.
void CheckTime_FLOW(uint16_t time) {
  if(time<(FLOW+FLOW_T) && time>(FLOW-FLOW_T)) State++;  
  else State = FRAMINGER;
}

void CheckTime_LOWSTART(uint16_t time) {
  if(time<(LOWSTART+LOWSTART_T) && time>(LOWSTART-LOWSTART_T)) State++;  
  else State = FRAMINGER;
}

void CheckTime_HIGHSTART(uint16_t time) {
  if(time<(HIGHSTART+HIGHSTART_T) && time>(HIGHSTART-HIGHSTART_T)) State++;  
  else State = FRAMINGER;
}

// funzione che inverte i bit di una byte, es.  10000000 -> 00000001
uint8_t ReverseByte(uint8_t byteOld){
uint8_t byteInv;
  for(uint8_t i=0; i<8; i++)
  {
    byteInv <<= 1;
    byteInv |= (byteOld & 1);
    byteOld >>= 1;
  }
return byteInv;
}


