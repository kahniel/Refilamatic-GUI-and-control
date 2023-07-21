#include <EEPROM.h>
#include <LiquidCrystal_I2C.h> 
#include <GyverStepper.h>
#include <EncButton2.h>
#include <max6675.h>
#include "GyverPID.h"

#define BUTL 25
#define BUTR 23
#define COOL 9
#define EMPT 5
#define HEAT 10
#define POT A0
#define PLACEHOLDER1 2000
#define thermoDO  6 // DO / SO / MISO
#define thermoCS  7 //  CS / SS
#define thermoCLK 8 // CLK / SCK

GyverPID regulator(3, 5, 8, 1000);
MAX6675 thermo(thermoCLK, thermoCS, thermoDO);
GStepper<STEPPER2WIRE> puller(1100, 22, 24, 26);
GStepper<STEPPER2WIRE> spin(1100, 8, 9);
GStepper<STEPPER2WIRE> vol(1100, 8, 9);
LiquidCrystal_I2C lcd(0x27, 16, 2);
EncButton2<EB_ENCBTN> enc(INPUT, 3, 2, 4);


// Спецсимволы для экрана
const int spnr = 0;
byte spnr_[] = {
  B00100,
  B01010,
  B10001,
  B00000,
  B11111,
  B01110,
  B00100,
  B00000
};
const int branch = 1;
byte branch_[] = {
  B00000,
  B00000,
  B00000,
  B00111,
  B11101,
  B00111,
  B00000,
  B00000
};
const int okBut = 2;
byte okBut_[] = {
  B00000,
  B00000,
  B00001,
  B00010,
  B10100,
  B01000,
  B00000,
  B11111
};
const int backBut = 3;
byte backBut_[] = {
  B00000,
  B00100,
  B01000,
  B11111,
  B01000,
  B00100,
  B00000,
  B11111
};
const int spnRight = 4;
byte spnRight_[] = {
  B00000,
  B00000,
  B01000,
  B01100,
  B01110,
  B01100,
  B01000,
  B00000
};
const int spnLeft = 5;
byte spnLeft_[] = {
  B00000,
  B00000,
  B00010,
  B00100,
  B01000,
  B00100,
  B00010,
  B00000
};
const int celc = 6;
byte celc_[] = {
  B10000,
  B00000,
  B01110,
  B10001,
  B10000,
  B10001,
  B01110,
  B00000
};
const int uno = 7;
byte uno_[] = {
  B01000,
  B11000,
  B01000,
  B01000,
  B01000,
  B01011,
  B01011,
  B00000
};


// Переменные работы с режимами
int mods[17][2]; // массив с данными режимов
// mods[0][1] – изменяемая в реальном времени температура создаваемого режима
// mods[1..7][1] – данные о температуре предустановленных режимов
// mods[8..16][0] == -1 – пользовательский режим с таким номером не установлен
// mods[8..16][1] – данные о температуре пользовательских режимов
int modq; // количество сохраненных режимов
int diam; // временно установленный диаметр для выбранного режима

// Переменные работы с меню
char state; // текущее состояние меню
char mod; // выбранный в меню режим
char pnt; // номер выделенного фрагмента второй строки экрана
char l, r; // левая и правая граница выделенного фрагмента
bool txtOn; // мигание выделенного фрагмента
char scrmem[16]; // текст второй строки экрана
int chChar = 1; // выбранный параметр удаляемого режима
int xnt;
int prevXnt;
int tnt;
int prevTnt;

// Переменные кнопок
bool butlSt, butrSt;

// Процедуры и переменные аппаратной части
int prevMil;
char stage;
int estart = -1;
int temp;
int d;
int mil;
int extrv;

// Функции и процедуры работы с режимами
char getStage() {
  mil = millis();
  if (digitalRead(EMPT)) {
    if (estart == -1) estart = mil;
    else {
      if (mil - estart >= PLACEHOLDER1) {
        if (stage != 'e') {
          stage = 'e';
          puller.disable();
          setUpState(1);
        }
      }
    }
  } else {
    estart = -1;
  }

  if (mods[mod][1] - temp > 2) {
    if (!digitalRead(EMPT) && stage != 'h') {
      stage = 'h';
      puller.disable();
      setUpState(1);
    }
    prevMil = mil;
    return;
  }
  
  digitalWrite(HEAT, 0);
  if (!digitalRead(EMPT) && stage != 's' && stage != 'r') {
    stage = 'r';
    puller.enable();
    puller.setSpeed(1100);
    prevMil = mil;
    setUpState(1);
  }
  if (stage == 's') {
    puller.disable();
  }
}


// Управление хранением mods в постоянной памяти
void modsSetUp() {
  for (int i = 1; i <= 7; ++i) mods[i][0] = 1;
  mods[1][1] = 190; // PLA
  mods[2][1] = 210; // PC
  mods[3][1] = 320; // PP
  mods[4][1] = 300; // PETG
  mods[5][1] = 180; // Neylon
  mods[6][1] = 280; // ABS
  mods[7][1] = 240; // ABS + PC
}
void modsBackUp() {
  EEPROM[0] = modq;
  int p = 1;
  for (int i = 8; i <= 16; ++i) {
    if (mods[i][0] == -1) {
      EEPROM[p] = 0;
      ++p;
    } else {
      EEPROM[p] = (mods[i][1] > 200) + 1;
      ++p;
      EEPROM[p] = (mods[i][1] > 200) ? (mods[i][1] - 200) : (mods[i][1]);
      ++p;
    }
  }
}
// Записать данных из постоянной памяти в mods
void modsFill() {
  modq = EEPROM[0];
  int p = 1;
  for (int i = 8; i <= 16; ++i) {
    if (EEPROM[p] == 0) {
      mods[i][0] = -1;
      ++p;
    } else {
      mods[i][0] = 1;
      mods[i][1] = (int)(EEPROM[p] - 1) * 200 + (int)(EEPROM[p + 1]);
      ++p;
      ++p;
    }
  }
}
// Очистить режимы
void modsClean() {
  modq = 0;
  for (int i = 8; i <= 16; ++i) mods[i][0] = -1;
  modsBackUp();
}
// Удалить режим m
void delMod(int m) {
  --modq;
  mods[m][0] = -1;
}

// Вернуть наименьший номер сохраненного режима
int firstMod() {
  int i = 8;
  while (mods[i][0] == -1) ++i;
  return i;
}
// Вернуть наименьший номер несохраненного режима
int firstFree() {
  int i = 8;
  while (mods[i][0] != -1) ++i;
  return i;
}
// Вернуть наибольший номер сохраненного режима
int lastMod() {
  int i = 16;
  while (mods[i][0] == -1) --i;
  return i;
}
// Перейти в направлении dir к ближайшему сохраненному режиму
int setMod(int dir) {
  mod += dir;
  while (7 < mod && mod < 17 && mods[mod][0] == -1 ) mod += dir;
  if (mod == 7) mod = lastMod();
  if (mod == 17) mod = firstMod();
  return mod;
}


// Функции и процедуры интерфейса

// Проверить, может ли быть выделен фрагмент pnt
bool badPnt() {
  if (state == 'm') return false;
  if (state == 'n') return false;
  if (state == 'd') return (pnt == 1 && modq == 1);
  if (state == 's') return false;
  if (state == 'r') return (stage != 's' && pnt == 2);
}

// Перезаписать вторую строку на экран
void rewrite() {
  int m = millis() / 10;
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; ++i) {
    if (!(l <= i && i <= r && txtOn == 0)) {
      if (scrmem[i] == '^') lcd.write(byte(spnr));
      else if (scrmem[i] == '<') lcd.write(byte(spnLeft));
      else if (scrmem[i] == '>') lcd.write(byte(spnRight));
      else if (scrmem[i] == '$') lcd.write(byte(backBut));
      else if (scrmem[i] == '@') lcd.write(byte(okBut));
      else if (scrmem[i] == '#') lcd.write(byte(celc));
      else if (scrmem[i] == ';') lcd.write(byte(uno));
      else lcd.write(scrmem[i]);
    } else lcd.write(' ');
  }
}

// Получить границы выделенного фрагмента
int getL(int pnt) {
  if (state == 'm') {
    int fst[] = {0, 7, 15};
    return fst[pnt];
  }
  if (state == 'n') {
    int fst[] = {0, 2, 8, 15};
    return fst[pnt];
  }
  if (state == 'd') {
    int fst[] = {0, 2, 15};
    return fst[pnt];
  }
  if (state == 's') {
    int fst[] = {0, 1, 10};
    return fst[pnt];
  }
  if (state == 'r' && stage != 'r') {
    int fst[] = {0, 2, 13};
    return fst[pnt];
  } else if (state == 'r') {
    int fst[] = {0};
    return fst[pnt];
  }
}
int getR(int pnt) {
  if (state == 'm') {
    int fst[] = {0, 12, 15};
    return fst[pnt];
  }
  if (state == 'n') {
    int fst[] = {0, 7, 13, 15};
    return fst[pnt];
  }
  if (state == 'd') {
    int fst[] = {0, 4, 15};
    return fst[pnt];
  }
  if (state == 's') {
    int fst[] = {0, 7, 14};
    return fst[pnt];
  }
  if (state == 'r' && stage != 'r') {
    int fst[] = {0, 11, 15};
    return fst[pnt];
  } else if (state == 'r') {
    int fst[] = {0};
    return fst[pnt];
  }
}

// Перейти к ближайшему pnt в направлении dir
void setPnt(int dir) {
  int qo;
  if (state == 'm') qo = 3;
  if (state == 'n') qo = 4;
  if (state == 'd') qo = 3;
  if (state == 's') qo = 3;
  if (state == 'r') qo = stage == 'r' ? 1 : 3;
  pnt = (pnt + dir + qo) % qo;
  while (badPnt()) pnt = (pnt + (dir == 0 ? 1 : dir) + qo) % qo;
  l = getL(pnt);
  r = getR(pnt);
  rewrite();
}

// Обновить вторую строку в памяти
void updateScrMem() {
  if (state == 'm') {
    scrmem[0] = '^';
    scrmem[1] = ' ';
    scrmem[2] = '0' + (mods[mod][1] / 100);
    scrmem[3] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[4] = '0' + (mods[mod][1] % 10);
    scrmem[5] = '#';
    scrmem[6] = ' ';
    scrmem[7] = '<';
    if (diam == 1) {
      scrmem[8] = ';';
      scrmem[9] = '7';
      scrmem[10] = '5';
    } else {
      scrmem[8] = '3';
      scrmem[9] = '.';
      scrmem[10] = '0';
    }
    scrmem[11] = '>';
    scrmem[12] = ' ';
    scrmem[13] = ' ';
    scrmem[14] = ' ';
    scrmem[15] = '@';
  }
  if (state == 'n') {
    scrmem[0] = '^';
    scrmem[1] = ' ';
    scrmem[2] = '<';
    scrmem[3] = '0' + (mods[mod][1] / 100);
    scrmem[4] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[5] = '0' + (mods[mod][1] % 10);
    scrmem[6] = '#';
    scrmem[7] = '>';
    scrmem[8] = ' ';
    scrmem[9] = '<';
    if (diam == 1) {
      scrmem[10] = ';';
      scrmem[11] = '7';
      scrmem[12] = '5';
    } else {
      scrmem[10] = '3';
      scrmem[11] = '.';
      scrmem[12] = '0';
    }
    scrmem[13] = '>';
    scrmem[14] = ' ';
    scrmem[15] = '@';
  }
  if (state == 'd') {
    scrmem[0] = '^';
    scrmem[1] = ' ';
    scrmem[2] = (modq == 1) ? ' ' : '<';
    scrmem[3] = '0' + (mod - 7);
    scrmem[4] = (modq == 1) ? ' ' : '>';
    scrmem[5] = ':';
    scrmem[6] = ' ';
    scrmem[7] = '0' + (mods[mod][1] / 100);
    scrmem[8] = '0' + ((mods[mod][1] / 10) % 10);
    scrmem[9] = '0' + (mods[mod][1] % 10);
    scrmem[10] = '#';
    scrmem[11] = ' ';
    scrmem[12] = ' ';
    scrmem[13] = ' ';
    scrmem[14] = ' ';
    scrmem[15] = '@';
  }
  if (state == 's') {
    scrmem[0] = '$';
    scrmem[1] = ' ';
    scrmem[2] = '[';
    scrmem[3] = 's';
    scrmem[4] = 'a';
    scrmem[5] = 'v';
    scrmem[6] = 'e';
    scrmem[7] = ']';
    scrmem[8] = ' ';
    scrmem[9] = ' ';
    scrmem[10] = '[';
    scrmem[11] = 'r';
    scrmem[12] = 'u';
    scrmem[13] = 'n';
    scrmem[14] = ']';
    scrmem[15] = ' ';
  }
  if (state == 'r') {
    if (stage == 'r') {
      scrmem[0] = ' ';
      scrmem[1] = ' ';
      scrmem[2] = ' ';
      scrmem[3] = '[';
      scrmem[4] = 'u';
      scrmem[5] = 'n';
      scrmem[6] = 'f';
      scrmem[7] = 'r';
      scrmem[8] = 'e';
      scrmem[9] = 'e';
      scrmem[10] = 'z';
      scrmem[11] = 'e';
      scrmem[12] = ']';
      scrmem[13] = ' ';
      scrmem[14] = ' ';
      scrmem[15] = ' ';
    } else {
      scrmem[0] = '$';
      scrmem[1] = ' ';
      scrmem[2] = '<';
      if (chChar == 1) {
        scrmem[3] = '0' + (temp / 100);
        scrmem[4] = '0' + ((temp / 10) % 10);
        scrmem[5] = '0' + (temp % 10);
        scrmem[6] = '/';
        scrmem[7] = '0' + (mods[mod][1] / 100);
        scrmem[8] = '0' + ((mods[mod][1] / 10) % 10);
        scrmem[9] = '0' + (mods[mod][1] % 10);
        scrmem[10] = '#';
      } else {
        scrmem[3] = '0' + 1;
        scrmem[4] = '.';
        scrmem[5] = '0' + 7;
        scrmem[6] = '0' + 3;
        scrmem[7] = '/';
        if (diam == 1) {
          scrmem[8] = ';';
          scrmem[9] = '7';
          scrmem[10] = '5';
        } else {
          scrmem[8] = '3';
          scrmem[9] = '.';
          scrmem[10] = '0';
        }
      }
      scrmem[11] = '>';
      scrmem[12] = ' ';
      scrmem[13] = (stage == 's') ? '[' : ' ';
      scrmem[14] = (stage == 's') ? '*' : ' ';
      scrmem[15] = (stage == 's') ? ']' : ' ';
    }
  }
  setPnt(0);
}

// Отреагировать на взаимодействие с выделенным фрагментом
void editPnt(int dir = 0) {
  if (state == 'm') {
    if (pnt == 0) {
      if (dir == 0) {
        mod = 1;
        setUpState(0);
        return;
      }
      if (dir == -1 && mod == 1) {
        if (modq == 9) {
          state = 'd';
        } else {
          state = 'n';
        }
        setUpState(0);
        return;
      }
      if (dir == -1) {
        mod -= 1;
        while (mods[mod][0] == -1) mod -= 1;
        setUpState(0);
        return;
      }
      if (dir == 1 && mod == lastMod()) {
        if (modq == 0) {
          state = 'n';
        } else {
          state = 'd';
        }
        setUpState(0);
        return;
      }
      if (dir == 1) {
        mod += 1;
        while (mods[mod][0] == -1) mod += 1;
        setUpState(0);
        return;
      }
    }
    if (pnt == 1) {
      diam = (diam == 1) ? 2 : 1;
      updateScrMem();
      txtOn = 1;
      rewrite();
      return;
    }
    if (pnt == 2 && dir == 0) {
      state = 'r';
      setUpState(0);
      return;
    }
  }
  if (state == 'n') {
    if (pnt == 0) {
      if (dir == 0) {
        state = 'm';
        mod = 1;
        setUpState(0);
        return;
      }
      if (dir == -1) {
        if (modq == 0) {
          state = 'm';
          mod = lastMod();
        } else {
          state = 'd';
        }
        setUpState(0);
        return;
      }
      if (dir == 1) {
        state = 'm';
        mod = 1;
        setUpState(0);
        return;
      }
    }
    if (pnt == 1) {
      if (dir == 0) {
        mods[0][1] = mods[0][1] + 10;
      } else {
        mods[0][1] = mods[0][1] + dir;
      }
      if (mods[0][1] > 350) mods[0][1] = 50;
      if (mods[0][1] < 50) mods[0][1] = 350;

      updateScrMem();
      txtOn = 1;
      rewrite();
      return;
    }
    if (pnt == 2) {
      diam = (diam == 1) ? 2 : 1;
      updateScrMem();
      txtOn = 1;
      rewrite();
      return;
    }
    if (pnt == 3 && dir == 0) {
      state = 's';
      setUpState(0);
      return;
    }
  }
  if (state == 'd') {
    if (pnt == 0) {
      if (dir == 0) {
        state = 'm';
        mod = 1;
        setUpState(0);
        return;
      }
      if (dir == -1) {
        state = 'm';
        mod = lastMod();
        setUpState(0);
        return;
      }
      if (dir == 1) {
        if (modq == 9) {
          state = 'm';
          mod = 1;
        } else {
          state = 'n';
        }
        setUpState(0);
        return;
      }
    }
    if (pnt == 1 && dir != 0) {
      setMod(dir);
    }
    if (pnt == 2 && dir == 0) {
      delMod(mod);
      modsBackUp();
      mod = 1;
      state = 'm';
      setUpState(0);
    }
    updateScrMem();
    rewrite();
    return;
  }
  if (state == 's') {
    if (pnt == 0 && dir == 0) {
      state = 'n';
      setUpState(1);
      return;
    }
    if (pnt == 1 && dir == 0) {
      ++modq;
      int newNum = firstFree();
      mods[newNum][0] = 1;
      mods[newNum][1] = mods[0][1];
      modsBackUp();
      mod = newNum;
      state = 'm';
      setUpState(0);
      return;
    }
    if (pnt == 2 && dir == 0) {
      state = 'r';
      setUpState(0);
      return;
    }
  }
  if (state == 'r' && stage != 'r') {
    if (pnt == 0 && dir == 0) {
      mod = 1;
      state = 'm';
      setUpState(0);
    }
    if (pnt == 1) {
      chChar = chChar == 1 ? 2 : 1;
    }
    if (pnt == 2 && dir == 0) {
      stage = 'r';
      puller.enable();
      setUpState(1);
    }
    updateScrMem();
    rewrite();
    return;
  } else if (state == 'r') {
    if (dir == 0) {
      stage = 's';
      setUpState(1);
    }
    updateScrMem();
    rewrite();
    return;
  }
}

// Настроить все для нового выбранного состояния меню
void setUpState(bool back) {
  if (state == 'm') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
    if (mod == 1) lcd.print("PLA");
    else if (mod == 2) lcd.print("PC");
    else if (mod == 3) lcd.print("PP");
    else if (mod == 4) lcd.print("PETG");
    else if (mod == 5) lcd.print("Neylon");
    else if (mod == 6) lcd.print("ABS");
    else if (mod == 7) lcd.print("ABS+PC");
    else {
      lcd.print("mode ");
      lcd.print(mod - 7);
    }
      lcd.print(":");
    pnt = 0;
    diam = 1;
    updateScrMem();
    rewrite();
  }
  if (state == 'n') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
    mod = 0;
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
      lcd.print("new mode ");
      lcd.print(firstFree() - 7);
      lcd.print(":");
    
    if (!back) {
      mods[0][1] = 50;
      diam = 1;
    }
    pnt = 0;
    updateScrMem();
    rewrite();
  }
  if (state == 'd') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
    mod = firstMod();
    chChar = 1;
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
      lcd.print("delete mode");
    
    pnt = 0;
    updateScrMem();
    rewrite();
  }
  if (state == 's') {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
      lcd.print("new mode ");
      lcd.print(firstFree() - 7);
      lcd.write(byte(branch));
    
    pnt = 0;
    updateScrMem();
    rewrite();
  }
  if (state == 'r') {
    if (!back) {
      regulator.setpoint = mods[mod][1];
      estart = -PLACEHOLDER1 - 10;
      getStage();
      pnt = 0;
    }
      updateScrMem();
      rewrite();
      lcd.setCursor(0, 0);
      lcd.setCursor(0, 0);
      lcd.write(byte(branch));
    if (mod == 0) lcd.print("new");
    else if (mod == 1) lcd.print("PLA");
    else if (mod == 2) lcd.print("PC");
    else if (mod == 3) lcd.print("PP");
    else if (mod == 4) lcd.print("PETG");
    else if (mod == 5) lcd.print("Neylon");
    else if (mod == 6) lcd.print("ABS");
    else if (mod == 7) lcd.print("ABS+PC");
    else {
      lcd.print(mod - 7);
    }
      lcd.write(byte(branch));
    if (stage == 'e') {
      lcd.print("empty            ");
    }
    if (stage == 'h') {
      lcd.print("heating            ");
    }
    if (stage == 'r') {
      lcd.print("running            ");
    }
    if (stage == 's') {
      lcd.print("stopped            ");
    }
  }
}

void checkInput() {
  enc.tick();
  if (enc.right()) {
    editPnt(1);
  }
  if (enc.left()) {
    editPnt(-1);
  }
  if (enc.click()) {
    editPnt();
  }
	if (digitalRead(BUTR) == 0 && butrSt == 1) {
		setPnt(1);
	}
	if (digitalRead(BUTL) == 0 && butlSt == 1) {
		setPnt(-1);
	}
	butlSt = digitalRead(BUTL);
	butrSt = digitalRead(BUTR);
}

void setup() {
  Serial.begin(9600); // настройка потока для связи с компьютером


  // Настройка подключенных устройств
  lcd.init();
  lcd.backlight();
  puller.setRunMode(KEEP_SPEED);
  spin.setRunMode(KEEP_SPEED);
  vol.setRunMode(KEEP_SPEED);
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255

	pinMode(BUTR, INPUT); // лево
	pinMode(BUTL, INPUT); // право
  pinMode(HEAT, OUTPUT);
  pinMode(COOL, OUTPUT);

  // Настройка спецсимволов
  lcd.createChar(spnr, spnr_);
  lcd.createChar(branch, branch_);
  lcd.createChar(okBut, okBut_);
  lcd.createChar(backBut, backBut_);
  lcd.createChar(spnRight, spnRight_);
  lcd.createChar(spnLeft, spnLeft_);
  lcd.createChar(celc, celc_);
  lcd.createChar(uno, uno_);

  // Заполнить mods из постоянной памяти
  modsSetUp();
  modsFill();

	butrSt = 0;
	butlSt = 0;
  xnt = 0;
  prevXnt = 0;
  tnt = 0;
  prevTnt = 0;
  state = 'm';
  mod = 1;
  setUpState(1);
  puller.setSpeed(1100);
}

void loop() {
  checkInput();
  if (state == 'r') {
    puller.tick();
    spin.tick();
    vol.tick();
    getStage();
  } else {
    digitalWrite(HEAT, 0);
    puller.disable();
    spin.disable();
    vol.disable();
  }
  

  tnt = millis();
  if (tnt - prevTnt >= 1000) {
    prevTnt = tnt;
    double t = thermo.readCelsius();
    if (temp != int(t)) {
      temp = t;
      if (state == 'r' && stage != 'r') updateScrMem();
    }
    if (state == 'r') {
      regulator.input = t;
      analogWrite(HEAT, regulator.getResultTimer());
    }
  }
  xnt = millis();
  if (xnt - prevXnt >= 500) {
    txtOn = !txtOn;
    prevXnt = xnt;
    if (!(state == 'r' && stage == 'r')) rewrite();
	}
}