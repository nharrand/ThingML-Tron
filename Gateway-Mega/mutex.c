#define NB_SLOT 4
#define RESSOURCE 4

bool entrance_intents[RESSOURCE][NB_SLOT];
uint8_t turn[RESSOURCE];

void mutex_setup() {
   for (uint8_t i = 0; i < RESSOURCE; i++) {
      for (uint8_t j = 0; j < NB_SLOT; j++) {
         entrance_intents[i][j] = false;
      }
      turn[i] = 0;
   }
}

bool orn(bool * tab, uint8_t p) {
   bool res = false;
   for(uint8_t i = 0; i < NB_SLOT; i++) {
	if(i != p) {res |= tab[i];}
    }
   return res;
}

void mutex_lock(uint8_t r, uint8_t p) {
   //Serial.print("[Lock] r:");
   //Serial.print(r);
   //Serial.print(" p:");
   //Serial.println(p);
   entrance_intents[r][p] = true;
   while (orn(entrance_intents[r], p)) {
      if (turn[r] != 0) {
         entrance_intents[r][0] = false;
         while (turn[r] != 0) {
           // busy wait
         }
         entrance_intents[r][0] = true;
      }
   }
 }

void mutex_unlock(uint8_t r, uint8_t p) {
   turn[r] = (turn[r] + 1) % NB_SLOT;
   entrance_intents[r][p] = false;
   //Serial.print("[Unlock] r:");
   //Serial.print(r);
   //Serial.print(" p:");
   //Serial.println(p);
}
